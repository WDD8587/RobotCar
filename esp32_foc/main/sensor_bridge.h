/**
 * @file    sensor_bridge.h
 * @brief   UART bridge to STM32F103 sensor hub
 *
 * UART1: GPIO 9(TX), GPIO 10(RX), 115200 8N1
 * STM32 sends sensor_data_t frames every 50 ms.
 */
#ifndef SENSOR_BRIDGE_H
#define SENSOR_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"

void sensor_bridge_init(void);
void sensor_bridge_poll(void);
bool sensor_bridge_get_latest(sensor_data_t *data);

#endif
