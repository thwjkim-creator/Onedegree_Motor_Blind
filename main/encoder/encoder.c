#include "encoder.h"
#include "driver/gpio.h"
#include "driver/gpio_filter.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>

static const char *TAG = "encoder";

/* Pulse count incremented/decremented by GPIO ISR.
 * Declared volatile because it is written in ISR context. */
static volatile int32_t s_count = 0;

/* PWM 스위칭 등 전기적 노이즈로 인한 짧은 글리치를 하드웨어가 걸러내도록 함
 * (pulses shorter than ~2 IO-MUX clock cycles are discarded) */
static gpio_glitch_filter_handle_t s_filter_a;
static gpio_glitch_filter_handle_t s_filter_b;

/* 소프트웨어 디바운스: 실제 최대속도(MAX_VEL_CPS≈1200cps → 최소 엣지 간격 ≈833µs)보다
 * 훨씬 짧고, PWM 노이즈 시간대(20kHz→50µs 주기)보다는 충분히 큰 값으로 설정 —
 * 진짜 엣지는 절대 걸러내지 않으면서 하드웨어 필터로 못 잡은 노이즈를 추가로 제거함 */
#define ENCODER_MIN_EDGE_INTERVAL_US  200
static volatile int64_t s_last_edge_us = 0;

/* Simple single-channel pulse counting on channel A.
 * Both rising and falling edges are counted (x2 resolution).
 * Direction is inferred from channel B level at the time of A edge.
 */
static void IRAM_ATTR encoder_isr(void *arg)
{
    int64_t now = esp_timer_get_time();
    if (now - s_last_edge_us < ENCODER_MIN_EDGE_INTERVAL_US) {
        return;
    }
    s_last_edge_us = now;

    int level_a = gpio_get_level(ENCODER_A_GPIO);
    int level_b = gpio_get_level(ENCODER_B_GPIO);

    /* Standard quadrature decode for x2 on channel A:
     * Rising A: B=0 → CW (+), B=1 → CCW (-)
     * Falling A: B=1 → CW (+), B=0 → CCW (-) */
    if (level_a == 1) {
        s_count += (level_b == 0) ? 1 : -1;
    } else {
        s_count += (level_b == 1) ? 1 : -1;
    }
}

void encoder_init(void)
{
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << ENCODER_A_GPIO) | (1ULL << ENCODER_B_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_ANYEDGE,  /* channel A triggers ISR */
    };
    ESP_ERROR_CHECK(gpio_config(&io_cfg));

    /* Only channel A drives the ISR; channel B is read inside ISR */
    gpio_set_intr_type(ENCODER_B_GPIO, GPIO_INTR_DISABLE);

    gpio_pin_glitch_filter_config_t filter_a_cfg = {
        .clk_src  = GLITCH_FILTER_CLK_SRC_DEFAULT,
        .gpio_num = ENCODER_A_GPIO,
    };
    ESP_ERROR_CHECK(gpio_new_pin_glitch_filter(&filter_a_cfg, &s_filter_a));
    ESP_ERROR_CHECK(gpio_glitch_filter_enable(s_filter_a));

    gpio_pin_glitch_filter_config_t filter_b_cfg = {
        .clk_src  = GLITCH_FILTER_CLK_SRC_DEFAULT,
        .gpio_num = ENCODER_B_GPIO,
    };
    ESP_ERROR_CHECK(gpio_new_pin_glitch_filter(&filter_b_cfg, &s_filter_b));
    ESP_ERROR_CHECK(gpio_glitch_filter_enable(s_filter_b));

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENCODER_A_GPIO, encoder_isr, NULL));

    ESP_LOGI(TAG, "Encoder initialized (A=GPIO%d, B=GPIO%d, PPR=%d, gear=%d, out_PPR=%d)",
             ENCODER_A_GPIO, ENCODER_B_GPIO, ENCODER_PPR,
             ENCODER_GEAR_RATIO, ENCODER_OUT_PPR);
}

int32_t encoder_get_count(void)
{
    return s_count;
}

void encoder_reset(void)
{
    s_count = 0;
}

float encoder_get_revolutions(void)
{
    /* x2 resolution on output shaft: counts per revolution = ENCODER_OUT_PPR * 2 = 4140 */
    return (float)s_count / (float)(ENCODER_OUT_PPR * 2);
}

bool encoder_target_reached(float target_revs)
{
    float revs = encoder_get_revolutions();
    return fabsf(revs) >= fabsf(target_revs);
}
