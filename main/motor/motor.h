#pragma once

#include "driver/ledc.h"

/* DRV8871: MOTOR_A=IN1, MOTOR_B=IN2
 * IN1=H IN2=L → Forward (blind opens)
 * IN1=L IN2=H → Reverse (blind closes)
 * IN1=L IN2=L → Coast
 * IN1=H IN2=H → Brake
 */
#define MOTOR_A_GPIO    6
#define MOTOR_B_GPIO    7

#define MOTOR_PWM_FREQ      20000   /* 20 kHz, above audible range */
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_10_BIT
#define MOTOR_PWM_MAX_DUTY  ((1 << 10) - 1)   /* 1023 */

typedef enum {
    MOTOR_FORWARD,
    MOTOR_REVERSE,
    MOTOR_BRAKE,
    MOTOR_COAST,
} motor_dir_t;

void motor_init(void);
void motor_set(motor_dir_t dir, uint32_t duty_percent); /* duty 0-100 */
void motor_stop(void);
