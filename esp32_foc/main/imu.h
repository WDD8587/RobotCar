/**
 * @file    imu.h
 * @brief   MPU6050 IMU via I2C — yaw angle + gyro Z reading
 *
 * I2C: GPIO 21(SDA), 22(SCL), 400 kHz
 * Uses complementary filter: yaw = 0.98*(yaw+gyro*dt) + 0.02*accel_yaw
 */
#ifndef IMU_H
#define IMU_H

#include <stdint.h>

void imu_init(void);
void imu_update(void);
void imu_get_yaw_gyro(int16_t *yaw_cdeg, int16_t *gyro_cdps);  /* centi-degrees */

#endif
