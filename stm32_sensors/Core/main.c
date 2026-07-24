/**
 * @file    main.c
 * @brief   RobotCar STM32F103 Sensor Hub — FWLib + FreeRTOS + Protocol
 *
 * Hardware config imported from a-meter-sweeping-robot project.
 * FWLib NOT modified — only referenced via Makefile include path.
 *
 * Tasks:
 *   vSensorTask (20Hz)      — ultrasound, bumper, battery ADC
 *   vEncoderTask (1kHz)     — wheel encoder input capture read
 *   vUartTxTask (20Hz)      — pack sensor_data_t + send via UART1 to ESP32
 *   vLedTask (5Hz)          — heartbeat LED
 */

#include "stm32f10x.h"
#include "robotcar_hw_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <string.h>

/* ---- Globals ---- */
static volatile uint16_t g_us_mm      = 0;
static volatile uint8_t  g_bumper     = 0;
static volatile uint16_t g_bat_mv     = 0;
static volatile int32_t  g_enc_left   = 0;
static volatile int32_t  g_enc_right  = 0;
static SemaphoreHandle_t g_sensor_mutex;

/* ============================================================================
 * Hardware Init
 * ============================================================================ */

static void Clock_Init(void) {
    /* HSE 8MHz → PLL×9 → 72MHz (from a-meter system_stm32f10x.c) */
    RCC_DeInit();
    RCC_HSEConfig(RCC_HSE_ON);
    while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET);

    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    FLASH_SetLatency(FLASH_Latency_2);

    RCC_HCLKConfig(RCC_SYSCLK_Div1);
    RCC_PCLK2Config(RCC_HCLK_Div1);
    RCC_PCLK1Config(RCC_HCLK_Div2);
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
    RCC_PLLCmd(ENABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Disable JTAG, keep SWD */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
}

static void UART_Init(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef g = {
        .GPIO_Pin   = UART_ESP32_TX_PIN,
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_Mode  = GPIO_Mode_AF_PP,
    };
    GPIO_Init(UART_ESP32_GPIO, &g);

    g.GPIO_Pin  = UART_ESP32_RX_PIN;
    g.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(UART_ESP32_GPIO, &g);

    USART_InitTypeDef u = {
        .USART_BaudRate            = UART_ESP32_BAUD,
        .USART_WordLength          = USART_WordLength_8b,
        .USART_StopBits            = USART_StopBits_1,
        .USART_Parity              = USART_Parity_No,
        .USART_HardwareFlowControl = USART_HardwareFlowControl_None,
        .USART_Mode                = USART_Mode_Tx | USART_Mode_Rx,
    };
    USART_Init(USART1, &u);
    USART_Cmd(USART1, ENABLE);
}

static void MotorPWM_Init(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOE, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);

    GPIO_InitTypeDef g = {
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_Mode  = GPIO_Mode_AF_PP,
    };
    g.GPIO_Pin = MOTOR_L_PWM_PIN; GPIO_Init(MOTOR_L_PWM_GPIO, &g);
    g.GPIO_Pin = MOTOR_R_PWM_PIN; GPIO_Init(MOTOR_R_PWM_GPIO, &g);

    /* Direction pins as GPIO output */
    g.GPIO_Mode = GPIO_Mode_Out_PP;
    g.GPIO_Pin = MOTOR_L_DIR_PIN; GPIO_Init(MOTOR_L_DIR_GPIO, &g);
    g.GPIO_Pin = MOTOR_R_DIR_PIN; GPIO_Init(MOTOR_R_DIR_GPIO, &g);

    TIM_TimeBaseInitTypeDef t = {
        .TIM_Prescaler         = MOTOR_PWM_PSC - 1,
        .TIM_Period            = MOTOR_PWM_ARR,
        .TIM_CounterMode       = TIM_CounterMode_Up,
        .TIM_ClockDivision     = TIM_CKD_DIV1,
    };
    TIM_TimeBaseInit(MOTOR_PWM_TIM, &t);

    TIM_OCInitTypeDef oc = {
        .TIM_OCMode      = TIM_OCMode_PWM1,
        .TIM_OutputState = TIM_OutputState_Enable,
        .TIM_Pulse       = 0,  /* 0 = full speed, ARR = stop (inverted logic) */
    };
    TIM_OC3Init(MOTOR_PWM_TIM, &oc);
    TIM_OC4Init(MOTOR_PWM_TIM, &oc);
    TIM_Cmd(MOTOR_PWM_TIM, ENABLE);

    /* Start at stop (ARR = full stop for this inverted H-bridge) */
    TIM_SetCompare3(MOTOR_PWM_TIM, MOTOR_PWM_ARR);
    TIM_SetCompare4(MOTOR_PWM_TIM, MOTOR_PWM_ARR);
}

static void Encoder_Init(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOE, ENABLE);
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);

    GPIO_InitTypeDef g = {
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_Mode  = GPIO_Mode_IN_FLOATING,
    };
    g.GPIO_Pin = ENCODER_L_PIN; GPIO_Init(ENCODER_L_GPIO, &g);
    g.GPIO_Pin = ENCODER_R_PIN; GPIO_Init(ENCODER_R_GPIO, &g);

    TIM_TimeBaseInitTypeDef t = {
        .TIM_Prescaler   = ENCODER_PSC - 1,
        .TIM_Period      = ENCODER_ARR,
        .TIM_CounterMode = TIM_CounterMode_Up,
    };
    TIM_TimeBaseInit(ENCODER_TIM, &t);

    /* Input capture: CH1 (left), CH2 (right), rising edge */
    TIM_ICInitTypeDef ic = {
        .TIM_Channel    = ENCODER_L_CH,
        .TIM_ICPolarity = TIM_ICPolarity_Rising,
        .TIM_ICSelection = TIM_ICSelection_DirectTI,
        .TIM_ICPrescaler = TIM_ICPSC_DIV1,
        .TIM_ICFilter    = 4,  /* 4-cycle filter for noise rejection */
    };
    TIM_ICInit(ENCODER_TIM, &ic);
    ic.TIM_Channel = ENCODER_R_CH;
    TIM_ICInit(ENCODER_TIM, &ic);

    TIM_ITConfig(ENCODER_TIM, TIM_IT_CC1 | TIM_IT_CC2, ENABLE);
    TIM_Cmd(ENCODER_TIM, ENABLE);
}

static void Ultrasonic_Init(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* TRIG: push-pull output */
    GPIO_InitTypeDef g = {
        .GPIO_Pin   = US_TRIG_PIN,
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_Mode  = GPIO_Mode_Out_PP,
    };
    GPIO_Init(US_TRIG_GPIO, &g);

    /* ECHO: input floating → EXTI */
    g.GPIO_Pin  = US_ECHO_PIN;
    g.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(US_ECHO_GPIO, &g);

    GPIO_EXTILineConfig(US_ECHO_EXTI_PORT, US_ECHO_EXTI_PIN);

    EXTI_InitTypeDef e = {
        .EXTI_Line    = US_ECHO_EXTI_LINE,
        .EXTI_Mode    = EXTI_Mode_Interrupt,
        .EXTI_Trigger = EXTI_Trigger_Rising_Falling,
        .EXTI_LineCmd = ENABLE,
    };
    EXTI_Init(&e);

    /* TIM2 for pulse width measurement */
    TIM_TimeBaseInitTypeDef t = {
        .TIM_Prescaler   = US_TIM_PSC - 1,
        .TIM_Period      = US_TIM_ARR,
        .TIM_CounterMode = TIM_CounterMode_Up,
    };
    TIM_TimeBaseInit(US_TIM, &t);
    TIM_Cmd(US_TIM, ENABLE);
}

static void Bumper_Init(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    GPIO_InitTypeDef g = {
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_Mode  = GPIO_Mode_IPD,  /* pull-down: LOW = no collision */
    };
    g.GPIO_Pin = BUMP_L_PIN; GPIO_Init(BUMP_L_GPIO, &g);
    g.GPIO_Pin = BUMP_R_PIN; GPIO_Init(BUMP_R_GPIO, &g);

    GPIO_EXTILineConfig(BUMP_L_EXTI_PORT, BUMP_L_EXTI_PIN);
    GPIO_EXTILineConfig(BUMP_R_EXTI_PORT, BUMP_R_EXTI_PIN);

    EXTI_InitTypeDef e = {
        .EXTI_Mode    = EXTI_Mode_Interrupt,
        .EXTI_Trigger = EXTI_Trigger_Rising,
        .EXTI_LineCmd = ENABLE,
    };
    e.EXTI_Line = BUMP_L_EXTI_LINE; EXTI_Init(&e);
    e.EXTI_Line = BUMP_R_EXTI_LINE; EXTI_Init(&e);
}

static void BatteryADC_Init(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef g = {
        .GPIO_Pin  = BAT_ADC_PIN,
        .GPIO_Mode = GPIO_Mode_AIN,
    };
    GPIO_Init(BAT_ADC_GPIO, &g);

    ADC_InitTypeDef a = {
        .ADC_Mode                   = ADC_Mode_Independent,
        .ADC_ScanConvMode           = DISABLE,
        .ADC_ContinuousConvMode     = DISABLE,
        .ADC_ExternalTrigConv       = ADC_ExternalTrigConv_None,
        .ADC_DataAlign              = ADC_DataAlign_Right,
        .ADC_NbrOfChannel           = 1,
    };
    ADC_Init(BAT_ADC, &a);
    ADC_RegularChannelConfig(BAT_ADC, BAT_ADC_CH, 1, ADC_SampleTime_239Cycles5);
    ADC_Cmd(BAT_ADC, ENABLE);
    ADC_ResetCalibration(BAT_ADC);
    while (ADC_GetResetCalibrationStatus(BAT_ADC));
    ADC_StartCalibration(BAT_ADC);
    while (ADC_GetCalibrationStatus(BAT_ADC));
}

/* ============================================================================
 * FreeRTOS Tasks
 * ============================================================================ */

/* 20Hz: read all sensors */
static void vSensorTask(void *arg) {
    TickType_t last = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(50));

        xSemaphoreTake(g_sensor_mutex, portMAX_DELAY);

        /* Ultrasonic: trigger 30us pulse */
        GPIO_SetBits(US_TRIG_GPIO, US_TRIG_PIN);
        for (volatile int i = 0; i < 72 * 30; i++) __NOP(); /* 30us @ 72MHz */
        GPIO_ResetBits(US_TRIG_GPIO, US_TRIG_PIN);

        /* Battery: read ADC */
        ADC_SoftwareStartConvCmd(BAT_ADC, ENABLE);
        while (!ADC_GetFlagStatus(BAT_ADC, ADC_FLAG_EOC));
        uint16_t raw = ADC_GetConversionValue(BAT_ADC);
        g_bat_mv = (uint16_t)(raw * 8.056f);  /* 10:1 divider, 3.3V ref */

        xSemaphoreGive(g_sensor_mutex);
    }
}

/* 20Hz: pack and send sensor data to ESP32-S3 via UART1 */
static void vUartTxTask(void *arg) {
    TickType_t last = xTaskGetTickCount();
    static uint8_t seq = 0;

    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(50));

        xSemaphoreTake(g_sensor_mutex, portMAX_DELAY);

        /* Pack using protocol from a-meter project */
        uint8_t buf[40];
        uint8_t data[16];
        data[0]  = (g_us_mm >> 8) & 0xFF;
        data[1]  = g_us_mm & 0xFF;
        data[2]  = (uint8_t)(g_enc_left >> 24);
        data[3]  = (uint8_t)(g_enc_left >> 16);
        data[4]  = (uint8_t)(g_enc_left >> 8);
        data[5]  = (uint8_t)(g_enc_left);
        data[6]  = (uint8_t)(g_enc_right >> 24);
        data[7]  = (uint8_t)(g_enc_right >> 16);
        data[8]  = (uint8_t)(g_enc_right >> 8);
        data[9]  = (uint8_t)(g_enc_right);
        data[10] = g_bumper;
        data[11] = (g_bat_mv >> 8) & 0xFF;
        data[12] = g_bat_mv & 0xFF;
        data[13] = 0x00; /* pad */
        data[14] = 0x00;
        data[15] = 0x00;

        /* Pack a-meter protocol frame: [0xAA][func][seq][len][data0..N][checksum] */
        uint8_t fn = 0xB2;  /* data frame */
        uint8_t dlen = 16;
        uint8_t sum = 0xAA + fn + seq + dlen;

        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, 0xAA);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, fn);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, seq++);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, dlen);
        for (int i = 0; i < 16; i++) {
            sum += data[i];
            while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
            USART_SendData(USART1, data[i]);
        }
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, sum);  /* 1-byte checksum */

        xSemaphoreGive(g_sensor_mutex);
    }
}

/* 5Hz: heartbeat LED */
static void vLedTask(void *arg) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef g = {
        .GPIO_Pin   = LED_STATUS_PIN | LED_ESTOP_PIN,
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_Mode  = GPIO_Mode_Out_PP,
    };
    GPIO_Init(GPIOB, &g);

    TickType_t last = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(200));
        LED_TOGGLE(LED_STATUS_GPIO, LED_STATUS_PIN);
    }
}

/* ============================================================================
 * ISR Handlers
 * ============================================================================ */

/* TIM1 CC: encoder input capture (from a-meter TimeCapture) */
static volatile uint16_t g_last_cap_l = 0, g_last_cap_r = 0;
static volatile uint32_t g_enc_l_raw = 0, g_enc_r_raw = 0;

void TIM1_CC_IRQHandler(void) {
    if (TIM_GetITStatus(TIM1, TIM_IT_CC1)) {
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
        uint16_t cap = TIM_GetCapture1(TIM1);
        g_enc_left += (int32_t)(cap - g_last_cap_l);
        g_last_cap_l = cap;
    }
    if (TIM_GetITStatus(TIM1, TIM_IT_CC2)) {
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);
        uint16_t cap = TIM_GetCapture2(TIM1);
        g_enc_right += (int32_t)(cap - g_last_cap_r);
        g_last_cap_r = cap;
    }
}

/* EXTI9_5: Ultrasonic echo + Bumper left */
static volatile uint16_t g_echo_start = 0;

void EXTI9_5_IRQHandler(void) {
    if (EXTI_GetITStatus(US_ECHO_EXTI_LINE)) {
        if (GPIO_ReadInputDataBit(US_ECHO_GPIO, US_ECHO_PIN)) {
            g_echo_start = TIM_GetCounter(US_TIM);  /* rising: start */
        } else {
            g_us_mm = (TIM_GetCounter(US_TIM) - g_echo_start) / 58;  /* falling: stop */
        }
        EXTI_ClearITPendingBit(US_ECHO_EXTI_LINE);
    }
    if (EXTI_GetITStatus(BUMP_L_EXTI_LINE)) {
        g_bumper |= 0x01;
        EXTI_ClearITPendingBit(BUMP_L_EXTI_LINE);
    }
}

/* EXTI15_10: Bumper right */
void EXTI15_10_IRQHandler(void) {
    if (EXTI_GetITStatus(BUMP_R_EXTI_LINE)) {
        g_bumper |= 0x02;
        EXTI_ClearITPendingBit(BUMP_R_EXTI_LINE);
    }
}

/* ============================================================================
 * Main
 * ============================================================================ */
int main(void) {
    Clock_Init();
    UART_Init();
    MotorPWM_Init();
    Encoder_Init();
    Ultrasonic_Init();
    Bumper_Init();
    BatteryADC_Init();

    /* NVIC priority group 4 (from a-meter config) */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_InitTypeDef nvic = { .NVIC_IRQChannelCmd = ENABLE };

    nvic.NVIC_IRQChannel = TIM1_CC_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel = US_ECHO_IRQ;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel = BUMP_R_IRQ;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_Init(&nvic);

    g_sensor_mutex = xSemaphoreCreateMutex();

    /* Create tasks (priorities from a-meter project) */
    xTaskCreate(vSensorTask, "sensor", 310, NULL, 1, NULL);
    xTaskCreate(vUartTxTask, "uart",   310, NULL, 1, NULL);
    xTaskCreate(vLedTask,   "led",    120, NULL, 1, NULL);

    vTaskStartScheduler();
    while (1);
}
