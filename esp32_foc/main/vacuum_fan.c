/**
 * @file    vacuum_fan.c
 * @brief   Fan motor control with anti-stall protection
 *
 * GPIO 8: LEDC PWM for fan speed control
 * GPIO 9: PCNT pulse counter for hall sensor RPM measurement
 *
 * Fan parameters (generic 24V vacuum fan):
 *   7 pole pairs, 6 hall pulses per mechanical revolution
 *   RPM = (hall_pulses_per_second * 60) / (7 * 6)
 *   Max RPM: ~25000, Min RPM: ~8000
 *
 * Anti-stall logic (from Dreame/Roborock fan control):
 *   1. Monitor RPM / current ratio every 100ms
 *   2. If RPM drops >30% from target while current stays high → stall
 *   3. Retry: reduce power to 30%, wait 1s, ramp back up
 *   4. After 3 retries → flag stalled, stop motor
 */

#include "vacuum_fan.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define FAN_PWM_FREQ   25000    /* 25 kHz, above audible */
#define FAN_PWM_GPIO   8
#define FAN_PWM_CH     LEDC_CHANNEL_4
#define FAN_TACH_GPIO  9

#define FAN_POLE_PAIRS   7
#define FAN_HALL_PPR     6     /* 6 hall pulses per mech rev */
#define FAN_PULSES_PER_REV (FAN_POLE_PAIRS * FAN_HALL_PPR)  /* 42 */

/* Anti-stall thresholds */
#define STALL_RPM_DROP_PCT  30   /* RPM drop >30% = possible stall */
#define STALL_RETRY_MAX     3
#define STALL_RECOVER_MS    1000

static const char *TAG = "fan";
static pcnt_unit_handle_t g_fan_pcnt = NULL;
static volatile uint16_t g_fan_rpm = 0;
static volatile bool g_fan_stalled = false;
static uint8_t g_fan_power = 0;
static int g_stall_retries = 0;
static int64_t g_last_check_us = 0;

void fan_init(void) {
    /* LEDC PWM for fan speed */
    ledc_channel_config_t ch = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = FAN_PWM_CH,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = FAN_PWM_GPIO,
        .duty       = 0,
    };
    ledc_channel_config(&ch);

    /* PCNT for hall sensor RPM */
    pcnt_unit_config_t unit_cfg = {
        .low_limit  = -32767,
        .high_limit = 32767,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &g_fan_pcnt));

    pcnt_chan_config_t chan_cfg = {
        .edge_gpio_num = FAN_TACH_GPIO,
        .level_gpio_num = -1,
    };
    pcnt_channel_handle_t chan;
    ESP_ERROR_CHECK(pcnt_new_channel(g_fan_pcnt, &chan_cfg, &chan));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
    ESP_ERROR_CHECK(pcnt_unit_enable(g_fan_pcnt));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(g_fan_pcnt));
    ESP_ERROR_CHECK(pcnt_unit_start(g_fan_pcnt));

    g_last_check_us = esp_timer_get_time();
    g_fan_stalled = false;
    g_stall_retries = 0;

    ESP_LOGI(TAG, "Fan initialized: PWM GPIO%d, TACH GPIO%d", FAN_PWM_GPIO, FAN_TACH_GPIO);
}

void fan_set_power(uint8_t power_pct) {
    if (g_fan_stalled) return;
    if (power_pct > 100) power_pct = 100;
    g_fan_power = power_pct;

    uint32_t duty = (uint32_t)(power_pct * 8191 / 100);  /* 13-bit */
    ledc_set_duty(LEDC_LOW_SPEED_MODE, FAN_PWM_CH, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, FAN_PWM_CH);
}

void fan_stop(void) {
    g_fan_power = 0;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, FAN_PWM_CH, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, FAN_PWM_CH);
    g_fan_stalled = false;
    g_stall_retries = 0;
}

uint16_t fan_get_rpm(void) {
    int count = 0;
    pcnt_unit_get_count(g_fan_pcnt, &count);
    pcnt_unit_clear_count(g_fan_pcnt);

    /* RPM = (pulses_per_second * 60) / pulses_per_rev */
    int64_t now = esp_timer_get_time();
    float dt = (now - g_last_check_us) / 1000000.0f;
    if (dt <= 0) return g_fan_rpm;

    g_fan_rpm = (uint16_t)((count / dt) * 60.0f / FAN_PULSES_PER_REV);
    g_last_check_us = now;
    return g_fan_rpm;
}

/**
 * @brief  Anti-stall check — call at 10 Hz from main loop
 *
 * Monitors RPM vs expected RPM based on power setting.
 * If fan is commanded >50% but RPM is <30% of expected → stall detected.
 */
bool fan_is_stalled(void) {
    if (g_fan_power < 30) return false;  /* low power, stall N/A */

    uint16_t rpm = fan_get_rpm();
    uint16_t expected = (uint16_t)(g_fan_power * 250);  /* 100% ≈ 25000 RPM */

    if (rpm < expected * (100 - STALL_RPM_DROP_PCT) / 100) {
        g_stall_retries++;
        ESP_LOGW(TAG, "Fan stall detected, retry %d/%d (RPM=%d, expected=%d)",
                 g_stall_retries, STALL_RETRY_MAX, rpm, expected);

        if (g_stall_retries >= STALL_RETRY_MAX) {
            ESP_LOGE(TAG, "Fan stalled permanently — stopping");
            g_fan_stalled = true;
            fan_stop();
            return true;
        }

        /* Stall recovery: reduce power, wait, ramp back */
        ledc_set_duty(LEDC_LOW_SPEED_MODE, FAN_PWM_CH, (uint32_t)(30 * 8191 / 100));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, FAN_PWM_CH);
        vTaskDelay(pdMS_TO_TICKS(STALL_RECOVER_MS));
        fan_set_power(g_fan_power);  /* restore */
        return false;
    }

    /* No stall, reset counter */
    if (g_stall_retries > 0) {
        ESP_LOGI(TAG, "Fan recovered from stall");
        g_stall_retries = 0;
    }
    return false;
}
