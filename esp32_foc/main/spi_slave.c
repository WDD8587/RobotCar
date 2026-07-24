/**
 * @file    spi_slave.c
 * @brief   SPI Slave implementation using ESP-IDF driver
 *
 * RPi drives clock at 8 MHz. ESP32 loads TX buffer, RPi clocks it out.
 * On transaction complete, RX callback fires.
 */
#include "spi_slave.h"
#include "driver/spi_slave.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include <string.h>

#define SPI_MOSI  13
#define SPI_MISO  12
#define SPI_SCLK  14
#define SPI_CS   15

static const char *TAG = "spi_slave";
static spi_frame_cb_t g_frame_cb = NULL;
static WORD_ALIGNED_ATTR uint8_t g_tx_buf[64];
static WORD_ALIGNED_ATTR uint8_t g_rx_buf[64];

void spi_slave_init(void) {
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SPI_MOSI,
        .miso_io_num = SPI_MISO,
        .sclk_io_num = SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_slave_interface_config_t slv_cfg = {
        .spics_io_num = SPI_CS,
        .flags = 0,
        .queue_size = 3,
        .mode = 3,                    /* CPOL=1, CPHA=1: SPI Mode 3 */
        .post_setup_cb = NULL,
        .post_trans_cb = NULL,
    };

    spi_slave_initialize(SPI2_HOST, &bus_cfg, &slv_cfg, SPI_DMA_DISABLED);
    memset(g_tx_buf, 0, sizeof(g_tx_buf));
    memset(g_rx_buf, 0, sizeof(g_rx_buf));

    ESP_LOGI(TAG, "SPI Slave initialized (Mode 3, pins MISO=%d MOSI=%d SCK=%d CS=%d)",
             SPI_MISO, SPI_MOSI, SPI_SCLK, SPI_CS);
}

void spi_slave_load_tx_buffer(const uint8_t *data, int len) {
    if (len > (int)sizeof(g_tx_buf)) len = sizeof(g_tx_buf);
    memcpy(g_tx_buf, data, len);
}

void spi_slave_set_callback(spi_frame_cb_t cb) {
    g_frame_cb = cb;
}

/**
 * @brief  Blocking SPI transaction: wait for master to clock data.
 * Call from task at 100 Hz. Master drives clock → data exchanged.
 */
void spi_slave_transact(int max_len) {
    spi_slave_transaction_t trans = {
        .length = max_len * 8,       /* in bits */
        .trans_len = 0,
        .tx_buffer = g_tx_buf,
        .rx_buffer = g_rx_buf,
    };

    esp_err_t ret = spi_slave_transmit(SPI2_HOST, &trans, portMAX_DELAY);
    if (ret == ESP_OK && g_frame_cb) {
        g_frame_cb(g_rx_buf, trans.trans_len / 8);
    }
}
