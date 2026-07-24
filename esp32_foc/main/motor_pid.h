/**
 * @file    motor_pid.h
 * @brief   PID speed controller for 2-wheel differential drive
 *
 * Uses ESP32 MCPWM for motor output and PCNT for encoder feedback.
 * PWM frequency: 20 kHz (above audible range)
 * PID update rate: 1 kHz (vMotorTask)
 */
#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#include <stdint.h>

void motor_init(void);
void motor_set_speed(float left_mm_s, float right_mm_s,
                     int32_t enc_l, int32_t enc_r);
void motor_estop(void);

#endif
