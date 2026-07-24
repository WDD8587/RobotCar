/**
 * @file    spi_slave.h
 * @brief   ESP32-S3 SPI Slave driver for RobotCar
 *
 * Raspberry Pi is SPI master. ESP32-S3 is SPI slave.
 * Full-duplex: master sends motor_cmd_t while receiving telemetry_t.
 * GPIO: MISO=12, MOSI=13, SCLK=14, CS=15
 */
#ifndef SPI_SLAVE_H
#define SPI_SLAVE_H

#include <stdint.h>

/* Callback when a complete SPI frame has been received from RPi */
typedef void (*spi_frame_cb_t)(const uint8_t *rx_data, int len);

void spi_slave_init(void);
void spi_slave_load_tx_buffer(const uint8_t *data, int len);
void spi_slave_set_callback(spi_frame_cb_t cb);

#endif
