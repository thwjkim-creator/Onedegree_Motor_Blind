/* ================================================================
 * STEP 1 MOTOR TEST  —  단계 선택
 *
 *  TEST_STAGE 1  단순 엔코더 제어  ─  100% 듀티, 목표 카운트 도달 즉시 정지
 *  TEST_STAGE 2  PI 위치 제어     ─  오차 비례 속도 조절로 정밀 정지
 * 
 *  TEST_STAGE 3  MQTT 제어        ─  JSON {rot, dir} 명령
 *
 *  숫자만 바꾸고 idf.py flash 하면 됩니다.
 * ================================================================ */
#define TEST_STAGE  3

/* ── 공통 헤더 ─────────────────────────────────────────────────── */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "step1";

/* ────────────────────────────────────────────────────────────────
 * STAGE 1 : 단순 엔코더 ON/OFF 제어
 *   100% 듀티로 달리다가 목표 회전 수 초과 시 즉시 정지 (COAST)
 *   → 500ms 폴링 특성상 오버슈트 발생 가능
 * ──────────────────────────────────────────────────────────────── */
#if TEST_STAGE == 1
#include "motor.h"
#include "encoder.h"

#define MOTOR_DUTY_PCT  100
#define TIMEOUT_MS      10000

static void run_to(motor_dir_t dir, float revs)
{
    const char *s = (dir == MOTOR_FORWARD) ? "FORWARD" : "REVERSE";
    ESP_LOGI(TAG, "%s %.1f rev ...", s, revs);

    encoder_reset();
    motor_set(dir, MOTOR_DUTY_PCT);

    uint32_t elapsed = 0;
    while (!encoder_target_reached(revs) && elapsed < TIMEOUT_MS) {
        vTaskDelay(pdMS_TO_TICKS(500));
        elapsed += 500;
        ESP_LOGI(TAG, "  count=%ld  revs=%.3f", (long)encoder_get_count(), encoder_get_revolutions());
    }
    motor_stop();

    if (elapsed >= TIMEOUT_MS) {
        ESP_LOGW(TAG, "Timeout! 엔코더 배선 확인 (PPR=%d  gear=%d)",
                 ENCODER_PPR, ENCODER_GEAR_RATIO);
    } else {
        ESP_LOGI(TAG, "Done: count=%ld  revs=%.2f",
                 (long)encoder_get_count(), encoder_get_revolutions());
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== Stage 1: 단순 엔코더 제어 ===");
    ESP_LOGI(TAG, "MOTOR A=GPIO%d B=GPIO%d  ENC A=GPIO%d B=GPIO%d",
             MOTOR_A_GPIO, MOTOR_B_GPIO, ENCODER_A_GPIO, ENCODER_B_GPIO);

    motor_init();
    encoder_init();

    run_to(MOTOR_FORWARD, 1.0f);
    vTaskDelay(pdMS_TO_TICKS(500));
    run_to(MOTOR_REVERSE, 1.0f);

    ESP_LOGI(TAG, "=== Stage 1 완료 ===");
}

/* ────────────────────────────────────────────────────────────────
 * STAGE 2 : PI 위치 제어
 *
 *  encoder.c 는 채널 A 에지 x2 해상도를 사용하므로:
 *    출력축 1회전 = ENCODER_OUT_PPR * 2 = 4140 카운트
 *
 *  제어 원리:
 *    error = target_count - current_count  (부호 있음)
 *    output = Kp * error + Ki * ∫error dt
 *    output > 0 → FORWARD, output < 0 → REVERSE
 *    목표 근처에서 BRAKE 후 COAST → 정밀 정지
 *
 *  게인 튜닝 지침:
 *    PI_KP     크게 → 빠른 접근, 과도하면 오버슈트
 *    PI_KI     마찰로 인한 정지 직전 오차 보상, 과도하면 오실레이션
 *    PI_MIN_DUTY  모터가 실제 기동하는 최소 듀티 (하드웨어 의존)
 *    PI_TOLERANCE 정지 허용 오차 (카운트 단위, 줄이면 정확도↑ 수렴 시간↑)
 *
 *  배선 확인:
 *    MOTOR_FORWARD 구동 시 encoder count 가 양수 증가 → OK
 *    음수 감소 → encoder A/B GPIO 교체 또는 motor.h FORWARD/REVERSE 반전
 *    (시작 시 방향 확인 로그로 자동 감지)
 * ──────────────────────────────────────────────────────────────── */
#elif TEST_STAGE == 2
#include <math.h>
#include "motor.h"
#include "encoder.h"

/* ── PI 파라미터 (여기서 튜닝) ──────────────────────────────────── */
#define PI_KP           0.05f   /* 2000 cnt(0.48 rev) 오차 → 100% 듀티 */
#define PI_KI           0.01f   /* 적분 게인: 작게 설정해 조기 포화 방지 */
#define PI_I_LIMIT      1000.0f /* anti-windup 클램프: Ki*I_LIMIT=10% 최대 기여 */
#define PI_DT_MS        10      /* 제어 주기 (ms) */
#define PI_MIN_DUTY     100      /* 최소 구동 듀티 (%) — stall 실측치 ~54% 보다 높게 */
#define PI_MAX_DUTY     100      /* 최대 듀티 (%) */
#define PI_SOFTSTART_MS 100     /* 소프트 스타트: 이 시간 동안 MIN→MAX 선형 증가 */
#define PI_TOLERANCE    25      /* 허용 오차 (카운트) ≈ 0.006 rev ≈ 2.2° */
#define PI_BRAKE_MS     150     /* 도달 후 브레이크 지속 시간 */
#define TIMEOUT_MS      20000   /* 전체 타임아웃 (ms) */
/* ──────────────────────────────────────────────────────────────── */

/* target: 양수 = FORWARD 방향, 음수 = REVERSE 방향 */
static void run_pi(int32_t target, const char *label)
{
    ESP_LOGI(TAG, "PI [%s]  target=%ld counts (%.3f rev)",
             label, (long)target, (float)target / COUNTS_PER_REV);

    encoder_reset();
    float    integral = 0.0f;
    uint32_t elapsed  = 0;

    while (elapsed < TIMEOUT_MS) {
        int32_t cur = encoder_get_count();
        float   err = (float)(target - cur);

        if (fabsf(err) < PI_TOLERANCE) {
            /* 목표 도달: 전기 브레이크 후 코스트 정지 */
            motor_set(MOTOR_BRAKE, 0);
            vTaskDelay(pdMS_TO_TICKS(PI_BRAKE_MS));
            motor_stop();
            ESP_LOGI(TAG, "Done [%s]: count=%ld  err=%.1f  (%.4f rev)",
                     label, (long)encoder_get_count(),
                     (float)(target - encoder_get_count()),
                     (float)encoder_get_count() / COUNTS_PER_REV);
            return;
        }

        /* PI 계산 */
        integral += err * (PI_DT_MS / 1000.0f);
        if (integral >  PI_I_LIMIT) integral =  PI_I_LIMIT;
        if (integral < -PI_I_LIMIT) integral = -PI_I_LIMIT;

        float output = PI_KP * err + PI_KI * integral;

        /* 소프트 스타트: elapsed 가 PI_SOFTSTART_MS 에 도달할 때까지
         * 허용 최대 듀티를 MIN→MAX 로 선형 증가시켜 초기 급가속 억제 */
        int eff_max = (elapsed < PI_SOFTSTART_MS)
            ? (int)(PI_MIN_DUTY + (float)(PI_MAX_DUTY - PI_MIN_DUTY)
                    * elapsed / PI_SOFTSTART_MS)
            : PI_MAX_DUTY;

        int duty = (int)fabsf(output);
        if (duty < PI_MIN_DUTY) duty = PI_MIN_DUTY;
        if (duty > eff_max)     duty = eff_max;

        motor_dir_t dir = (output >= 0.0f) ? MOTOR_FORWARD : MOTOR_REVERSE;
        motor_set(dir, (uint32_t)duty);

        vTaskDelay(pdMS_TO_TICKS(PI_DT_MS));
        elapsed += PI_DT_MS;

        if (elapsed % 500 == 0) {
            ESP_LOGI(TAG, "  [%s] %us  cnt=%ld  err=%.0f  duty=%d  intg=%.0f",
                     label, elapsed / 1000, (long)cur, err, duty, integral);
        }
    }

    motor_stop();
    ESP_LOGW(TAG, "Timeout [%s]! count=%ld  target=%ld (엔코더 배선 확인)",
             label, (long)encoder_get_count(), (long)target);
}

void app_main(void)
{
    /* 앱 기동 직후 모터 드라이버 입력을 즉시 LOW로 잡아 글리치 시간 최소화.
     * 부트로더 구간(이 코드 이전)은 소프트웨어로 제어 불가 →
     * 완전 해결은 GPIO6/GPIO7에 10kΩ 풀다운 저항 추가 필요. */
    gpio_set_direction(MOTOR_A_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_B_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(MOTOR_A_GPIO, 0);
    gpio_set_level(MOTOR_B_GPIO, 0);

    ESP_LOGI(TAG, "=== Stage 2: PI 위치 제어 ===");
    ESP_LOGI(TAG, "MOTOR A=GPIO%d B=GPIO%d  ENC A=GPIO%d B=GPIO%d",
             MOTOR_A_GPIO, MOTOR_B_GPIO, ENCODER_A_GPIO, ENCODER_B_GPIO);
    ESP_LOGI(TAG, "COUNTS_PER_REV=%d  Kp=%.3f  Ki=%.3f  tol=±%d cnt",
             COUNTS_PER_REV, PI_KP, PI_KI, PI_TOLERANCE);

    motor_init();
    encoder_init();
    // vTaskDelay(pdMS_TO_TICKS(500));
    /* motor_set() 의 INFO 로그가 10ms 루프에서 스팸되지 않도록 억제 */
    esp_log_level_set("motor", ESP_LOG_WARN);

    /* ── PI 위치 제어 시퀀스 (3회 반복) ────────────────────────── */
    for (int i = 1; i <= 2; i++) {
        ESP_LOGI(TAG, "--- 반복 %d/3 ---", i);
        run_pi(+(int32_t)(0.5f * COUNTS_PER_REV), "FWD 1rev");
        vTaskDelay(pdMS_TO_TICKS(500));
        run_pi(-(int32_t)(0.5f * COUNTS_PER_REV), "REV 1rev");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    // run_pi(+(int32_t)(2.0f * COUNTS_PER_REV), "FWD 1rev");
    // vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "=== Stage 2 완료 ===");
}

/* ────────────────────────────────────────────────────────────────
 * STAGE 3 : MQTT + BLE 프로비저닝 + 캐스케이드 PI + OTA
 *   각 기능은 별도 모듈로 분리되어 있습니다:
 *     wifi_prov.c  — BLE 프로비저닝 + WiFi 연결
 *     ota.c        — GitHub HTTPS OTA 버전 체크 및 업데이트
 *     motor_task.c — 캐스케이드 PI 제어 + MQTT 명령 처리
 *   MQTT_URI, MQTT_TOPIC_PREFIX 만 여기서 수정하세요.
 * ──────────────────────────────────────────────────────────────── */
#elif TEST_STAGE == 3
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_ota_ops.h"
#include "driver/gpio.h"
#include "mqtt_client.h"
#include "motor.h"
#include "wifi_prov.h"
#include "ota.h"
#include "motor_task.h"

#define MQTT_URI             "mqtt://broker.hivemq.com:1883"
// #define MQTT_URI             "mqtt://172.21.101.31:1883"
#define MQTT_TOPIC_PREFIX    "onedegree/motor_blind/device"

void app_main(void)
{
    /* [0] 모터 핀 즉시 LOW — OTA 리부팅 후 PWM 글리치 방지 */
    gpio_config_t mc = {
        .pin_bit_mask = (1ULL << MOTOR_A_GPIO) | (1ULL << MOTOR_B_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&mc);
    gpio_set_level(MOTOR_A_GPIO, 0);
    gpio_set_level(MOTOR_B_GPIO, 0);

    /* [1] OTA 롤백 방지 — 이 앱을 유효 파티션으로 확정 */
    esp_ota_mark_app_valid_cancel_rollback();

    /* [2] NVS 초기화 */
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

    ESP_LOGI(TAG, "=== Stage 3: MQTT + 캐스케이드 PI (v%s) ===", CURRENT_FIRMWARE_VERSION);

    /* [3] WiFi (BLE 프로비저닝) — 반환 시 IP 획득 보장 */
    wifi_prov_init();

    /* [4] OTA 버전 확인 — 신버전이면 다운로드 후 재부팅 */
    ota_check_and_update();

    /* MAC → device_id + 구독/발행 토픽 생성 */
    uint8_t mac[6];
    char    device_id[16];
    char    sub_topic[64];
    char    pub_topic[64];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    snprintf(device_id,  sizeof(device_id),  "%02X%02X%02X", mac[3], mac[4], mac[5]);
    snprintf(sub_topic,  sizeof(sub_topic),  "%s/%s/cmd",    MQTT_TOPIC_PREFIX, device_id);
    snprintf(pub_topic,  sizeof(pub_topic),  "%s/%s/status", MQTT_TOPIC_PREFIX, device_id);
    ESP_LOGI(TAG, "device_id=%s  sub=%s", device_id, sub_topic);

    /* [5] MQTT 클라이언트 생성 + 모터 태스크 시작 */
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri    = MQTT_URI,
        .credentials.client_id = device_id,
    };
    esp_mqtt_client_handle_t mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    motor_task_start(sub_topic, pub_topic, mqtt_client);
    esp_mqtt_client_start(mqtt_client);

    ESP_LOGI(TAG, "대기 중... 토픽: %s", sub_topic);
    vTaskDelay(portMAX_DELAY);
}

#else
#error "TEST_STAGE 를 1, 2, 3 중 하나로 설정하세요."
#endif
