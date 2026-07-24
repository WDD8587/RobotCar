/**
 * @file    encoder.c
 * @brief   PCNT quadrature encoder reader — ESP-IDF v6.0 pulse_cnt API
 *
 * PCNT0: Left encoder  — GPIO 16(A), 17(B)
 * PCNT1: Right encoder — GPIO 18(A), 19(B)
 */
#include "encoder.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"

#define ENC_L_A  16
#define ENC_L_B  17
#define ENC_R_A  18
#define ENC_R_B  19
#define PCNT_HLIM 32767

static const char *TAG = "encoder";
static pcnt_unit_handle_t g_pcnt_l = NULL;
static pcnt_unit_handle_t g_pcnt_r = NULL;

void encoder_init(void) {
    /* Left encoder */
    pcnt_unit_config_t unit_cfg = {
        .low_limit  = -PCNT_HLIM,
        .high_limit = PCNT_HLIM,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &g_pcnt_l));

    pcnt_chan_config_t chan_cfg = {
        .edge_gpio_num = ENC_L_A,
        .level_gpio_num = ENC_L_B,
    };
    pcnt_channel_handle_t chan_l;
    ESP_ERROR_CHECK(pcnt_new_channel(g_pcnt_l, &chan_cfg, &chan_l));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_l, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_l, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_enable(g_pcnt_l));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(g_pcnt_l));
    ESP_ERROR_CHECK(pcnt_unit_start(g_pcnt_l));

    /* Right encoder */
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &g_pcnt_r));
    pcnt_chan_config_t chan_cfg_r = {
        .edge_gpio_num = ENC_R_A,
        .level_gpio_num = ENC_R_B,
    };
    pcnt_channel_handle_t chan_r;
    ESP_ERROR_CHECK(pcnt_new_channel(g_pcnt_r, &chan_cfg_r, &chan_r));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_r, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_r, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_enable(g_pcnt_r));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(g_pcnt_r));
    ESP_ERROR_CHECK(pcnt_unit_start(g_pcnt_r));

    ESP_LOGI(TAG, "Encoders initialized");
}

void encoder_get(int32_t *left, int32_t *right) {
    int count = 0;
    pcnt_unit_get_count(g_pcnt_l, &count);
    *left = (int32_t)count;
    pcnt_unit_get_count(g_pcnt_r, &count);
    *right = (int32_t)count;
}

void encoder_reset(void) {
    pcnt_unit_clear_count(g_pcnt_l);
    pcnt_unit_clear_count(g_pcnt_r);
}
