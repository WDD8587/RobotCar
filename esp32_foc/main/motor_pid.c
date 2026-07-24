/**
 * @file    motor_pid.c
 * @brief   PID motor control using ESP32-S3 LEDC (simple PWM)
 *
 * Uses LEDC for PWM output — more compatible across ESP-IDF versions.
 * MCPWM was removed from ESP-IDF v6.0; use MCPWM prelude if needed.
 *
 * PWM: LEDC 20 kHz, GPIO 4/5 (L), GPIO 6/7 (R)
 */
#include "motor_pid.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

#define PWM_FREQ_HZ    20000
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_CH_L_A     LEDC_CHANNEL_0
#define LEDC_CH_L_B     LEDC_CHANNEL_1
#define LEDC_CH_R_A     LEDC_CHANNEL_2
#define LEDC_CH_R_B     LEDC_CHANNEL_3
#define GPIO_L_A        4
#define GPIO_L_B        5
#define GPIO_R_A        6
#define GPIO_R_B        7

#define CPR            660.0f
#define WHEEL_R_MM     32.5f
#define MM_PER_COUNT   ((2.0f * 3.14159f * WHEEL_R_MM) / CPR)
#define KP  0.8f
#define KI  3.0f
#define KD  0.05f

static const char *TAG = "motor";

typedef struct {
    float kp, ki, kd, integral, prev_error, integral_limit;
} PID_t;

typedef struct {
    PID_t   pid;
    int32_t prev_enc;
    float   speed_measured, speed_target, output;
} MotorCtrl_t;

static MotorCtrl_t gMotorL, gMotorR;

static void pid_init(PID_t *pid, float kp, float ki, float kd, float ilim) {
    pid->kp = kp; pid->ki = ki; pid->kd = kd;
    pid->integral_limit = ilim;
    pid->integral = 0; pid->prev_error = 0;
}

static float pid_update(PID_t *pid, float error, float dt) {
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}

static void set_pwm(ledc_channel_t ch, float duty_pct) {
    if (duty_pct > 100.0f) duty_pct = 100.0f;
    if (duty_pct < 0.0f) duty_pct = 0.0f;
    uint32_t duty = (uint32_t)(duty_pct * 8191 / 100.0f);  /* 13-bit */
    ledc_set_duty(LEDC_MODE, ch, duty);
    ledc_update_duty(LEDC_MODE, ch);
}

void motor_init(void) {
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num       = LEDC_TIMER,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t ch = {
        .speed_mode = LEDC_MODE,
        .timer_sel  = LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
    };
    ch.gpio_num = GPIO_L_A; ch.channel = LEDC_CH_L_A; ledc_channel_config(&ch);
    ch.gpio_num = GPIO_L_B; ch.channel = LEDC_CH_L_B; ledc_channel_config(&ch);
    ch.gpio_num = GPIO_R_A; ch.channel = LEDC_CH_R_A; ledc_channel_config(&ch);
    ch.gpio_num = GPIO_R_B; ch.channel = LEDC_CH_R_B; ledc_channel_config(&ch);

    pid_init(&gMotorL.pid, KP, KI, KD, 50.0f);
    pid_init(&gMotorR.pid, KP, KI, KD, 50.0f);
    memset(&gMotorL, 0, sizeof(gMotorL));
    memset(&gMotorR, 0, sizeof(gMotorR));

    set_pwm(LEDC_CH_L_A, 0); set_pwm(LEDC_CH_L_B, 0);
    set_pwm(LEDC_CH_R_A, 0); set_pwm(LEDC_CH_R_B, 0);

    ESP_LOGI(TAG, "LEDC PWM initialized: %d Hz, GPIO %d/%d/%d/%d",
             PWM_FREQ_HZ, GPIO_L_A, GPIO_L_B, GPIO_R_A, GPIO_R_B);
}

void motor_set_speed(float left_mm_s, float right_mm_s,
                     int32_t enc_l, int32_t enc_r) {
    const float dt = 0.001f;

    int32_t dl = enc_l - gMotorL.prev_enc; gMotorL.prev_enc = enc_l;
    gMotorL.speed_measured = dl * MM_PER_COUNT / dt;
    float err_l = left_mm_s - gMotorL.speed_measured;
    float pid_l = pid_update(&gMotorL.pid, err_l, dt);
    float ff_l = left_mm_s / 5.0f;
    gMotorL.output = (left_mm_s != 0) ? (ff_l + pid_l) : 0;
    if (gMotorL.output > 0) {
        set_pwm(LEDC_CH_L_A, gMotorL.output);
        set_pwm(LEDC_CH_L_B, 0);
    } else {
        set_pwm(LEDC_CH_L_A, 0);
        set_pwm(LEDC_CH_L_B, -gMotorL.output);
    }

    int32_t dr = enc_r - gMotorR.prev_enc; gMotorR.prev_enc = enc_r;
    gMotorR.speed_measured = dr * MM_PER_COUNT / dt;
    float err_r = right_mm_s - gMotorR.speed_measured;
    float pid_r = pid_update(&gMotorR.pid, err_r, dt);
    float ff_r = right_mm_s / 5.0f;
    gMotorR.output = (right_mm_s != 0) ? (ff_r + pid_r) : 0;
    if (gMotorR.output > 0) {
        set_pwm(LEDC_CH_R_A, gMotorR.output);
        set_pwm(LEDC_CH_R_B, 0);
    } else {
        set_pwm(LEDC_CH_R_A, 0);
        set_pwm(LEDC_CH_R_B, -gMotorR.output);
    }
}

void motor_estop(void) {
    set_pwm(LEDC_CH_L_A, 0); set_pwm(LEDC_CH_L_B, 0);
    set_pwm(LEDC_CH_R_A, 0); set_pwm(LEDC_CH_R_B, 0);
    gMotorL.output = 0; gMotorR.output = 0;
    gMotorL.pid.integral = 0; gMotorR.pid.integral = 0;
    ESP_LOGW(TAG, "ESTOP");
}
