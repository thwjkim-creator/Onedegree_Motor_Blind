#include <math.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "mqtt_client.h"
#include "cJSON.h"
#include "motor.h"
#include "encoder.h"
#include "motor_task.h"

static const char *TAG = "motor_task";

/* ── 제어 파라미터 ────────────────────────────────────────────── */
#define PI_DT_MS        10       /* 제어 주기 (ms) */
#define PI_MIN_DUTY     35       /* 실측 stall 듀티 ~35% 보다 살짝 위 */
#define PI_MAX_DUTY     100      /* 최대 듀티 (%) */
#define PI_TOLERANCE    25       /* 위치 허용 오차 (counts) */
#define PI_BRAKE_MS     150      /* 목표 도달 후 브레이크 시간 (ms) */

/* ── 외부 루프: 위치 P 게인 ───────────────────────────────────── */
#define MAX_VEL_CPS     1200.0f  /* 실측값 (Stage2 로그: Δcnt/Δt @ 100% duty) */
#define POS_KP          (MAX_VEL_CPS / COUNTS_PER_REV)

/* ── 내부 루프: 속도 PI 게인 ──────────────────────────────────── */
#define VEL_KP          ((float)PI_MAX_DUTY / MAX_VEL_CPS)
#define VEL_KI          0.50f
#define VEL_I_LIMIT     200.0f
#define VEL_LPF_ALPHA   0.30f
/* ─────────────────────────────────────────────────────────────── */

static QueueHandle_t             s_cmd_q;
static char                      s_sub_topic[64];
static char                      s_pub_topic[64];
static esp_mqtt_client_handle_t  s_mqtt_client;
static int32_t                   s_position_counts = 0;
static bool                      s_dir_inverted    = false;

/* 방향 반전 플래그 반영 — FORWARD/REVERSE 외 값(BRAKE, COAST)은 그대로 통과 */
static inline motor_dir_t apply_dir(motor_dir_t d)
{
    if (!s_dir_inverted) return d;
    if (d == MOTOR_FORWARD) return MOTOR_REVERSE;
    if (d == MOTOR_REVERSE) return MOTOR_FORWARD;
    return d;
}

/* ── MQTT ───────────────────────────────────────────────────────── */
static void mqtt_event_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data)
{
    esp_mqtt_event_handle_t ev = data;

    if (id == MQTT_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "MQTT 연결됨 → 구독: %s", s_sub_topic);
        esp_mqtt_client_subscribe(ev->client, s_sub_topic, 0);
        esp_mqtt_client_publish(ev->client, s_pub_topic, "{\"status\":\"online\"}", 0, 0, 0);

    } else if (id == MQTT_EVENT_DATA) {
        char buf[128] = {0};
        int  len = ev->data_len < (int)sizeof(buf) - 1
                 ? ev->data_len : (int)sizeof(buf) - 1;
        memcpy(buf, ev->data, len);

        cJSON *root = cJSON_Parse(buf);
        if (!root) { ESP_LOGW(TAG, "JSON 파싱 실패: %s", buf); return; }

        cJSON *j_rot = cJSON_GetObjectItem(root, "rot");
        cJSON *j_dir = cJSON_GetObjectItem(root, "dir");
        cJSON *j_spd = cJSON_GetObjectItem(root, "spd");

        if (cJSON_IsNumber(j_rot) && cJSON_IsNumber(j_dir)) {
            motor_cmd_t cmd = {
                .rot = (float)j_rot->valuedouble,
                .dir = j_dir->valueint,
                .spd = cJSON_IsNumber(j_spd) ? j_spd->valueint : 100,
            };
            ESP_LOGI(TAG, "수신: rot=%.2f  dir=%d  spd=%d", cmd.rot, cmd.dir, cmd.spd);
            xQueueOverwrite(s_cmd_q, &cmd);
        } else {
            ESP_LOGW(TAG, "rot/dir 필드 없음");
        }
        cJSON_Delete(root);
    }
}

static void publish_position(void)
{
    char payload[32];
    snprintf(payload, sizeof(payload), "{\"rot\":%.4f}",
             (float)s_position_counts / COUNTS_PER_REV);
    esp_mqtt_client_publish(s_mqtt_client, s_pub_topic, payload, 0, 0, 0);
    ESP_LOGI(TAG, "위치 발행: %s → %s", s_pub_topic, payload);
}

/* ── 캐스케이드 PI: 위치 P(외부) + 속도 PI(내부) ─────────────────── */
static void run_pi_task(int32_t target, int spd)
{
    float    max_vel    = MAX_VEL_CPS * spd / 100.0f;
    uint32_t timeout_ms = (uint32_t)(fabsf((float)target) / max_vel * 1000.0f * 2.0f) + 2000;

    ESP_LOGI(TAG, "캐스케이드 PI: target=%ld counts (%.3f rev) spd=%d%%  timeout=%lus",
             (long)target, (float)target / COUNTS_PER_REV, spd, (unsigned long)(timeout_ms / 1000));

    encoder_reset();

    int32_t  prev_count   = 0;
    float    actual_vel   = 0.0f;
    float    vel_integral = 0.0f;
    uint32_t elapsed      = 0;
    int64_t  prev_time    = esp_timer_get_time();

    while (elapsed < timeout_ms) {
        motor_cmd_t peek;
        if (xQueuePeek(s_cmd_q, &peek, 0)) {
            motor_stop();
            ESP_LOGI(TAG, "새 명령 감지 – 동작 중단");
            return;
        }

        int32_t cur     = encoder_get_count();
        float   pos_err = (float)(target - cur);

        if (fabsf(pos_err) < PI_TOLERANCE) {
            motor_set(MOTOR_BRAKE, 0);
            vTaskDelay(pdMS_TO_TICKS(PI_BRAKE_MS));
            motor_stop();
            ESP_LOGI(TAG, "완료: count=%ld  err=%.1f  (%.4f rev)",
                     (long)cur, pos_err, (float)cur / COUNTS_PER_REV);
            return;
        }

        float vel_setpoint = (pos_err >= 0.0f) ? max_vel : -max_vel;

        int64_t now = esp_timer_get_time();
        float   dt  = (float)(now - prev_time) * 1e-6f;
        if (dt < 1e-4f) dt = 1e-4f;
        float meas_vel = (float)(cur - prev_count) / dt;
        actual_vel = VEL_LPF_ALPHA * meas_vel + (1.0f - VEL_LPF_ALPHA) * actual_vel;
        prev_count = cur;
        prev_time  = now;

        float vel_err = vel_setpoint - actual_vel;
        vel_integral += vel_err * dt;
        if (vel_integral >  VEL_I_LIMIT) vel_integral =  VEL_I_LIMIT;
        if (vel_integral < -VEL_I_LIMIT) vel_integral = -VEL_I_LIMIT;

        float output = VEL_KP * vel_err + VEL_KI * vel_integral;

        int duty = (int)fabsf(output);
        if (fabsf(vel_setpoint) > 10.0f && duty < PI_MIN_DUTY) duty = PI_MIN_DUTY;
        if (duty > PI_MAX_DUTY) duty = PI_MAX_DUTY;

        motor_dir_t drv_dir = (pos_err >= 0.0f) ? MOTOR_FORWARD : MOTOR_REVERSE;
        motor_set(apply_dir(drv_dir), (uint32_t)duty);

        vTaskDelay(pdMS_TO_TICKS(PI_DT_MS));
        elapsed += PI_DT_MS;

        if (elapsed % 500 == 0) {
            ESP_LOGI(TAG,
                     "  %us  cnt=%ld  pos_err=%.0f  vel_sp=%.0f  vel=%.0f  duty=%d  vi=%.1f",
                     elapsed / 1000, (long)cur,
                     pos_err, vel_setpoint, actual_vel, duty, vel_integral);
        }
    }

    motor_stop();
    ESP_LOGW(TAG, "Timeout! count=%ld  target=%ld",
             (long)encoder_get_count(), (long)target);
}

/* ── 부팅 시 엔코더 방향 자동 감지 ────────────────────────────────
 * 100% 듀티로 최대 0.5초 구동하며 카운트 변화 감지.
 * 음수면 s_dir_inverted=true → 이후 모든 방향 명령이 반전됨. */
static void detect_encoder_dir(void)
{
    encoder_reset();

    /* 1) 100% 듀티로 구동하면서 카운트 변화 감지 (최대 0.5초) */
    motor_set(MOTOR_FORWARD, 100);
    int32_t count = 0;
    for (int i = 0; i < 50; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
        count = encoder_get_count();
        if (count > 20 || count < -20) break;
    }
    motor_stop();
    vTaskDelay(pdMS_TO_TICKS(50));

    if (count < -20) {
        s_dir_inverted = true;
        ESP_LOGW(TAG, "엔코더 방향 반전 감지 (count=%ld) → 자동 보정", (long)count);
    } else if (count > 20) {
        s_dir_inverted = false;
        ESP_LOGI(TAG, "엔코더 방향 정상 (count=%ld)", (long)count);
    } else {
        ESP_LOGW(TAG, "방향 감지 실패 (count=%ld) — 모터 연결 확인", (long)count);
        return;
    }

    /* 2) 반대 방향으로 카운트가 0 근처로 돌아올 때까지 복귀 (최대 0.5초) */
    motor_set(MOTOR_REVERSE, 100);
    for (int i = 0; i < 50; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
        int32_t c = encoder_get_count();
        if (c > -20 && c < 20) break;
    }
    motor_stop();
    vTaskDelay(pdMS_TO_TICKS(100));
    encoder_reset();
}

/* ── 모터 제어 태스크 ────────────────────────────────────────────── */
static void motor_task_fn(void *arg)
{
    motor_init();
    encoder_init();
    esp_log_level_set("motor", ESP_LOG_WARN);
    detect_encoder_dir();

    motor_cmd_t cmd;
    for (;;) {
        xQueueReceive(s_cmd_q, &cmd, portMAX_DELAY);

        if (cmd.dir == 0 || cmd.spd == 0) {
            motor_stop();
            ESP_LOGI(TAG, "정지");
            publish_position();
            continue;
        }

        int32_t target = (cmd.dir > 0)
            ?  (int32_t)(cmd.rot * COUNTS_PER_REV)
            : -(int32_t)(cmd.rot * COUNTS_PER_REV);

        /* 홈(0) 이하로 내려가지 않도록 역방향 이동량 클리핑 */
        if (target < 0 && s_position_counts + target < 0) {
            target = -s_position_counts;
            ESP_LOGW(TAG, "홈 위치 초과 방지: target → %ld counts", (long)target);
        }

        run_pi_task(target, cmd.spd);
        s_position_counts += encoder_get_count();
        publish_position();
    }
}

void motor_task_start(const char *sub_topic, const char *pub_topic,
                      esp_mqtt_client_handle_t mqtt_client)
{
    snprintf(s_sub_topic, sizeof(s_sub_topic), "%s", sub_topic);
    snprintf(s_pub_topic, sizeof(s_pub_topic), "%s", pub_topic);
    s_mqtt_client = mqtt_client;

    s_cmd_q = xQueueCreate(1, sizeof(motor_cmd_t));
    xTaskCreate(motor_task_fn, "motor", 4096, NULL, 5, NULL);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
}
