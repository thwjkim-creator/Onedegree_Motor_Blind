#include "encoder.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "encoder";

/* Pulse count incremented/decremented by GPIO ISR.
 * Declared volatile because it is written in ISR context. */
static volatile int32_t s_count = 0;

/* Simple single-channel pulse counting on channel A.
 * Both rising and falling edges are counted (x2 resolution).
 * Direction is inferred from channel B level at the time of A edge.
 */
static void IRAM_ATTR encoder_isr(void *arg)
{
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
