/**
 * @file    vacuum_fan.h
 * @brief   Vacuum fan motor — BLDC FOC constant power control
 *
 * GPIO 8 (PWM), GPIO 9 (TACH RPM input from hall sensor)
 * Fan: 24V BLDC, 15000-25000 RPM, 7 pole pairs
 * Control: constant power = Vbus * Iphase, anti-stall detection
 *
 * Same control logic as Dreame X50 / Roborock G30 fan:
 * - Normal: maintain constant suction power across battery voltage
 * - Stall: RPM drops while current rises → reduce power, retry 3x, then stop
 */
#ifndef VACUUM_FAN_H
#define VACUUM_FAN_H

#include <stdint.h>
#include <stdbool.h>

void fan_init(void);
void fan_set_power(uint8_t power_pct);  /* 0-100% */
void fan_stop(void);
uint16_t fan_get_rpm(void);
bool fan_is_stalled(void);

#endif
