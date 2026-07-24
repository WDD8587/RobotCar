/**
 * @file    imu.c
 * @brief   MPU6050 IMU — ESP-IDF v6.0 I2C master API
 */
#include "imu.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

#define I2C_SCL      22
#define I2C_SDA      21
#define MPU6050_ADDR 0x68
#define I2C_FREQ     400000

static const char *TAG = "imu";
static i2c_master_bus_handle_t g_bus = NULL;
static i2c_master_dev_handle_t g_dev = NULL;
static float g_yaw = 0.0f, g_gyro_z = 0.0f;
static int64_t g_last_us = 0;

void imu_init(void) {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port     = I2C_NUM_0,
        .sda_io_num   = I2C_SDA,
        .scl_io_num   = I2C_SCL,
        .clk_source   = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &g_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MPU6050_ADDR,
        .scl_speed_hz    = I2C_FREQ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(g_bus, &dev_cfg, &g_dev));

    /* Wake up MPU6050 */
    uint8_t buf[2] = {0x6B, 0x00};
    i2c_master_transmit(g_dev, buf, 2, -1);

    g_last_us = esp_timer_get_time();
    ESP_LOGI(TAG, "MPU6050 initialized");
}

void imu_update(void) {
    int64_t now = esp_timer_get_time();
    float dt = (now - g_last_us) / 1000000.0f;
    g_last_us = now;
    if (dt <= 0 || dt > 0.1f) dt = 0.01f;

    uint8_t data[14];
    uint8_t reg = 0x3B;
    esp_err_t ret = i2c_master_transmit_receive(g_dev, &reg, 1, data, 14, -1);
    if (ret != ESP_OK) return;

    int16_t gz = (data[12] << 8) | data[13];
    int16_t ay = (data[2] << 8) | data[3];
    int16_t az = (data[4] << 8) | data[5];

    g_gyro_z = gz / 131.0f;
    float accel_angle = atan2f(ay, az) * 180.0f / M_PI;
    g_yaw = 0.98f * (g_yaw + g_gyro_z * dt) + 0.02f * accel_angle;
}

void imu_get_yaw_gyro(int16_t *yaw_cdeg, int16_t *gyro_cdps) {
    *yaw_cdeg  = (int16_t)(g_yaw * 100.0f);
    *gyro_cdps = (int16_t)(g_gyro_z * 100.0f);
}
