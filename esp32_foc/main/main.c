/**
 * @file    main.c
 * @brief   ESP32-S3 Motor Control Firmware — FreeRTOS, SPI Slave, PID, IMU
 *
 * Architecture:
 *   Core 0: vMotorTask (1 kHz PID speed loop + encoder reading)
 *   Core 1: vSpiTask (100 Hz SPI slave bridge)
 *           vImuTask (100 Hz MPU6050 I2C reading)
 *           vSensorTask (20 Hz UART polling STM32F103)
 *
 * Hardware:
 *   SPI2 (HSPI): Slave mode, GPIO 12(MISO), 13(MOSI), 14(SCLK), 15(CS)
 *   MCPWM0: Motor A — GPIO 4(PWMA_H), 5(PWMA_L)
 *   MCPWM0: Motor B — GPIO 6(PWMB_H), 7(PWMB_L)
 *   PCNT0: Encoder L — GPIO 16(A), 17(B)
 *   PCNT1: Encoder R — GPIO 18(A), 19(B)
 *   I2C0: MPU6050 — GPIO 21(SDA), 22(SCL)
 *   UART1: STM32 — GPIO 9(TX), 10(RX)
 *
 * Build: idf.py set-target esp32s3 && idf.py build
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include "spi_slave.h"
#include "motor_pid.h"
#include "encoder.h"
#include "imu.h"
#include "sensor_bridge.h"
#include "protocol.h"

static const char *TAG = "main";

static telemetry_t  g_telemetry;
static motor_cmd_t  g_cmd;
static SemaphoreHandle_t g_spi_mutex;
static SemaphoreHandle_t g_cmd_mutex;

/* ---- SPI callback: invoked from vSpiTask when frame received ---- */
void on_spi_frame_received(const uint8_t *rx_data, int len) {
    xSemaphoreTake(g_spi_mutex, portMAX_DELAY);

    /* Parse motor command */
    motor_cmd_t cmd;
    if (unpack_motor_cmd(rx_data, &cmd) == 0) {
        xSemaphoreTake(g_cmd_mutex, portMAX_DELAY);
        memcpy(&g_cmd, &cmd, sizeof(cmd));
        xSemaphoreGive(g_cmd_mutex);

        if (cmd.flags & 0x01) {  /* ESTOP */
            motor_estop();
            ESP_LOGW(TAG, "ESTOP!");
        }
    }

    /* Pack telemetry */
    sensor_data_t sensor;
    if (sensor_bridge_get_latest(&sensor)) {
        g_telemetry.us_front_mm = sensor.us1_mm;
        g_telemetry.us_rear_mm  = sensor.us2_mm;
        g_telemetry.bumper      = sensor.bumper;
        g_telemetry.battery_pct = (uint8_t)(sensor.bat_mv / 120);  /* rough SoC */
    } else {
        g_telemetry.us_front_mm = 0;
        g_telemetry.us_rear_mm  = 0;
        g_telemetry.bumper      = 0;
    }

    // 处理编码器数据
    int32_t enc_left, enc_right;
    encoder_get(&enc_left, &enc_right);
    g_telemetry.enc_left = enc_left;
    g_telemetry.enc_right = enc_right;

    // 处理IMU数据
    int16_t yaw, gyro;
    imu_get_yaw_gyro(&yaw, &gyro);
    g_telemetry.imu_yaw = yaw;
    g_telemetry.imu_gyro_z = gyro;

    xSemaphoreGive(g_spi_mutex);
}

/* ---- Motor control task (1 kHz, Core 0) ---- */
static void vMotorTask(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    ESP_LOGI(TAG, "Motor task started on Core %d", xPortGetCoreID());

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));  /* 1 ms = 1 kHz */

        xSemaphoreTake(g_cmd_mutex, portMAX_DELAY);
        float vl = g_cmd.v_left;
        float vr = g_cmd.v_right;
        xSemaphoreGive(g_cmd_mutex);

        /* PID speed control */
        int32_t enc_l, enc_r;
        encoder_get(&enc_l, &enc_r);
        motor_set_speed(vl, vr, enc_l, enc_r);
    }
}

/* ---- SPI bridge task (100 Hz, Core 1) ---- */
static void vSpiTask(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    uint8_t seq = 0;
    uint8_t tx_buf[SPI_FRAME_SIZE];
    ESP_LOGI(TAG, "SPI task started on Core %d", xPortGetCoreID());

    spi_slave_init();

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));  /* 100 Hz */

        xSemaphoreTake(g_spi_mutex, portMAX_DELAY);
        pack_telemetry(tx_buf, &g_telemetry, seq++);
        spi_slave_load_tx_buffer(tx_buf, SPI_FRAME_SIZE);
        xSemaphoreGive(g_spi_mutex);
    }
}

/* ---- IMU task (100 Hz, Core 1) ---- */
static void vImuTask(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    imu_init();
    ESP_LOGI(TAG, "IMU task started on Core %d", xPortGetCoreID());

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
        imu_update();
    }
}

/* ---- Sensor bridge task (20 Hz, Core 1) ---- */
static void vSensorTask(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    sensor_bridge_init();
    ESP_LOGI(TAG, "Sensor task started on Core %d", xPortGetCoreID());

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(50));
        sensor_bridge_poll();
    }
}

/* ---- Main ---- */
void app_main(void) {
    ESP_LOGI(TAG, "RobotCar ESP32-S3 Motor Controller");
    ESP_LOGI(TAG, "CPU: %d MHz, Free heap: %lu bytes",
             CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ, esp_get_free_heap_size());

    g_spi_mutex = xSemaphoreCreateMutex();
    g_cmd_mutex = xSemaphoreCreateMutex();
    memset(&g_cmd, 0, sizeof(g_cmd));
    memset(&g_telemetry, 0, sizeof(g_telemetry));

    motor_init();
    encoder_init();

    /* Motor control on Core 0 (dedicated for real-time) */
    xTaskCreatePinnedToCore(vMotorTask, "motor", 4096, NULL, 10, NULL, 0);

    /* IO tasks on Core 1 */
    xTaskCreatePinnedToCore(vSpiTask,   "spi",   4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(vImuTask,   "imu",   2048, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(vSensorTask,"sensor",2048, NULL, 3, NULL, 1);

    ESP_LOGI(TAG, "All tasks started");
}
