/**
 * @file    sensor_bridge.c
 * @brief   UART bridge between ESP32-S3 and STM32F103 sensor hub
 */
#include "sensor_bridge.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

#define UART_PORT   UART_NUM_1
#define UART_TX     9
#define UART_RX     10
#define UART_BUF    256

static const char *TAG = "sensor";
static sensor_data_t g_latest;
static bool g_valid = false;

void sensor_bridge_init(void) {
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_PORT, &cfg);
    uart_set_pin(UART_PORT, UART_TX, UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT, UART_BUF, 0, 0, NULL, 0);

    ESP_LOGI(TAG, "UART bridge to STM32 initialized: 115200 8N1");
}

void sensor_bridge_poll(void) {
    uint8_t buf[UART_FRAME_SIZE];
    int len = uart_read_bytes(UART_PORT, buf, UART_FRAME_SIZE, pdMS_TO_TICKS(10));
    if (len == UART_FRAME_SIZE) {
        sensor_data_t data;
        if (unpack_sensor_data(buf, &data)) {
            memcpy(&g_latest, &data, sizeof(data));
            g_valid = true;
        }
    }
    /* Flush stale bytes */
    if (len <= 0) {
        uart_flush_input(UART_PORT);
    }
}

bool sensor_bridge_get_latest(sensor_data_t *data) {
    if (!g_valid) return false;
    memcpy(data, &g_latest, sizeof(*data));
    return true;
}
