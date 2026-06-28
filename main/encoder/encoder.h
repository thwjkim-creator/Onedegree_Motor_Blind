#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Quadrature encoder on GPIO4 (A) and GPIO5 (B).
 * Motor spec: 3 PPR (6-pole magnetic ring), gear reduction 1:690.
 * ENCODER_OUT_PPR is the effective PPR measured at the output shaft.
 */
#define ENCODER_A_GPIO      4
#define ENCODER_B_GPIO      5
#define ENCODER_PPR         3               /* basic pulses per motor shaft revolution */
#define ENCODER_GEAR_RATIO  690             /* gear reduction ratio */
#define ENCODER_OUT_PPR     (ENCODER_PPR * ENCODER_GEAR_RATIO)  /* 2070 PPR at output shaft */
/* x2 quadrature (single-channel ISR fires on both edges of A): */
#define COUNTS_PER_REV      (ENCODER_OUT_PPR * 2)              /* 4140 counts per output-shaft rev */

void     encoder_init(void);
int32_t  encoder_get_count(void);
void     encoder_reset(void);
float    encoder_get_revolutions(void);

/* Returns true once the requested number of revolutions has elapsed
 * since the last encoder_reset() call. */
bool     encoder_target_reached(float target_revs);
