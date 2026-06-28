#include "motor.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "motor";

/* LEDC channel assignment
 * MOTOR_A uses LEDC_CHANNEL_0 (controls forward PWM)
 * MOTOR_B uses LEDC_CHANNEL_1 (controls reverse PWM)
 */
#define CH_A    LEDC_CHANNEL_0
#define CH_B    LEDC_CHANNEL_1
#define TIMER   LEDC_TIMER_0

void motor_init(void)
{
    /* Drive both GPIOs LOW before LEDC takes over.
     * Without this, ledc_channel_config() causes a brief HIGH glitch
     * that pulses the motor unexpectedly on startup. */
    gpio_config_t pre = {
        .pin_bit_mask = (1ULL << MOTOR_A_GPIO) | (1ULL << MOTOR_B_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&pre);
    gpio_set_level(MOTOR_A_GPIO, 0);
    gpio_set_level(MOTOR_B_GPIO, 0);

    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = TIMER,
        .duty_resolution = MOTOR_PWM_RESOLUTION,
        .freq_hz         = MOTOR_PWM_FREQ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    /* 소프트 리셋 후 LEDC 레지스터에 이전 듀티가 남아있을 수 있음.
     * 채널을 GPIO에 연결하기 전에 타이머를 정지해 순간 출력 방지. */
    ledc_timer_pause(LEDC_LOW_SPEED_MODE, TIMER);

    ledc_channel_config_t ch_a = {
        .gpio_num   = MOTOR_A_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = CH_A,
        .timer_sel  = TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_a));

    ledc_channel_config_t ch_b = {
        .gpio_num   = MOTOR_B_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = CH_B,
        .timer_sel  = TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_b));

    /* 두 채널 모두 duty=0 으로 설정 완료 후 타이머 재개 */
    ledc_timer_resume(LEDC_LOW_SPEED_MODE, TIMER);

    ESP_LOGI(TAG, "Motor initialized (A=GPIO%d, B=GPIO%d)", MOTOR_A_GPIO, MOTOR_B_GPIO);
}

void motor_set(motor_dir_t dir, uint32_t duty_percent)
{
    if (duty_percent > 100) duty_percent = 100;
    uint32_t duty = (MOTOR_PWM_MAX_DUTY * duty_percent) / 100;

    switch (dir) {
    case MOTOR_FORWARD:
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_A, duty);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_B, 0);
        ESP_LOGI(TAG, "Forward duty=%lu%%", duty_percent);
        break;
    case MOTOR_REVERSE:
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_A, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_B, duty);
        ESP_LOGI(TAG, "Reverse duty=%lu%%", duty_percent);
        break;
    case MOTOR_BRAKE:
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_A, MOTOR_PWM_MAX_DUTY);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_B, MOTOR_PWM_MAX_DUTY);
        ESP_LOGI(TAG, "Brake");
        break;
    case MOTOR_COAST:
    default:
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_A, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_B, 0);
        ESP_LOGI(TAG, "Coast");
        break;
    }

    ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_A);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_B);
}

void motor_stop(void)
{
    motor_set(MOTOR_COAST, 0);
}
