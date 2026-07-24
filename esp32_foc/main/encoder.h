/**
 * @file    encoder.h
 * @brief   Quadrature encoder reading via ESP32 PCNT (Pulse Counter)
 *
 * Uses 2 PCNT units for 2 encoders.
 * Each PCNT channel counts rising + falling edges on both A and B signals,
 * giving 4× resolution (e.g., 11 PPR → 44 counts/rev).
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

void encoder_init(void);
void encoder_get(int32_t *left, int32_t *right);
void encoder_reset(void);

#endif
