/**
 * @file    robotcar_hw_config.h
 * @brief   Unified hardware pin mapping for RobotCar STM32F103
 *
 * Sources: a-meter-sweeping-robot-stm32 FWLib project
 * FWLib path: ../../a-meter-sweeping-robot-stm32-master/firmware/STM32F10x_FWLib/
 *
 * This file maps ALL RobotCar hardware to STM32F103 pins.
 * Imported from the a-meter project and adapted for 2-wheel car.
 *
 * Chip: STM32F103CET6 (Medium-Density, 512KB Flash, 64KB RAM)
 * Clock: HSE 8MHz → PLL×9 → 72MHz SYSCLK
 * APB1: 36MHz, APB2: 72MHz
 */

#ifndef ROBOTCAR_HW_CONFIG_H
#define ROBOTCAR_HW_CONFIG_H

#include "stm32f10x.h"

/* ============================================================================
 * System Clock
 * ============================================================================ */
#define SYSCLK_FREQ_HZ  72000000
#define APB1_FREQ_HZ    36000000
#define APB2_FREQ_HZ    72000000

/* ============================================================================
 * UART1: Communication with ESP32-S3
 * ============================================================================ */
#define UART_ESP32           USART1
#define UART_ESP32_BAUD      115200
#define UART_ESP32_TX_PIN    GPIO_Pin_9   /* PA9  */
#define UART_ESP32_RX_PIN    GPIO_Pin_10  /* PA10 */
#define UART_ESP32_GPIO      GPIOA
#define UART_ESP32_IRQ       USART1_IRQn
#define UART_ESP32_TX_DMA    DMA1_Channel4
#define UART_ESP32_RX_DMA    DMA1_Channel5

/* ============================================================================
 * Motor PWM (2-wheel differential) — TIM3
 * ============================================================================ */
#define MOTOR_PWM_TIM         TIM3
#define MOTOR_PWM_FREQ        12000           /* 12 kHz, same as a-meter wheels */
#define MOTOR_PWM_ARR         500
#define MOTOR_PWM_PSC         12              /* 72MHz / 12 / 500 = 12kHz */

/* Left motor:  TIM3_CH3 (PB0) */
#define MOTOR_L_PWM_CH        TIM_Channel_3
#define MOTOR_L_PWM_PIN       GPIO_Pin_0
#define MOTOR_L_PWM_GPIO      GPIOB

/* Right motor: TIM3_CH4 (PB1) */
#define MOTOR_R_PWM_CH        TIM_Channel_4
#define MOTOR_R_PWM_PIN       GPIO_Pin_1
#define MOTOR_R_PWM_GPIO      GPIOB

/* ============================================================================
 * Motor Direction GPIO
 * ============================================================================ */
/* Left motor direction: PE7 (HIGH=reverse, LOW=forward) */
#define MOTOR_L_DIR_PIN       GPIO_Pin_7
#define MOTOR_L_DIR_GPIO      GPIOE
#define MOTOR_L_FORWARD()     GPIO_ResetBits(MOTOR_L_DIR_GPIO, MOTOR_L_DIR_PIN)
#define MOTOR_L_REVERSE()     GPIO_SetBits(MOTOR_L_DIR_GPIO, MOTOR_L_DIR_PIN)

/* Right motor direction: PE4 (HIGH=reverse, LOW=forward) */
#define MOTOR_R_DIR_PIN       GPIO_Pin_4
#define MOTOR_R_DIR_GPIO      GPIOE
#define MOTOR_R_FORWARD()     GPIO_ResetBits(MOTOR_R_DIR_GPIO, MOTOR_R_DIR_PIN)
#define MOTOR_R_REVERSE()     GPIO_SetBits(MOTOR_R_DIR_GPIO, MOTOR_R_DIR_PIN)

/* ============================================================================
 * Encoder Input Capture — TIM1 (Fully remapped)
 *
 * Left encoder:  TIM1_CH1 (PE9)
 * Right encoder: TIM1_CH2 (PE11)
 *
 * Method: Input capture, measure period between rising edges
 * Resolution: 100kHz (72MHz / 720 prescaler)
 * ============================================================================ */
#define ENCODER_TIM           TIM1
#define ENCODER_ARR           50000
#define ENCODER_PSC           720         /* 100kHz capture clock */

#define ENCODER_L_CH          TIM_Channel_1
#define ENCODER_L_PIN         GPIO_Pin_9
#define ENCODER_L_GPIO        GPIOE
#define ENCODER_L_IC          TIM_IC1

#define ENCODER_R_CH          TIM_Channel_2
#define ENCODER_R_PIN         GPIO_Pin_11
#define ENCODER_R_GPIO        GPIOE
#define ENCODER_R_IC          TIM_IC2

/* Wheel parameters */
#define WHEEL_PERIMETER_MM    220
#define ENC_COUNTS_PER_REV    263
#define SINGLE_ENC_DISTANCE   83650   /* scale factor for 100kHz clock */

/* ============================================================================
 * Ultrasonic HC-SR04 — TIM2 input capture
 *
 * TRIG: PC6 (GPIO output, 30us pulse)
 * ECHO: PC7 (EXTI7, input capture on TIM2_CH2)
 * ============================================================================ */
#define US_TRIG_PIN           GPIO_Pin_6
#define US_TRIG_GPIO          GPIOC
#define US_ECHO_PIN           GPIO_Pin_7
#define US_ECHO_GPIO          GPIOC
#define US_ECHO_EXTI_LINE     EXTI_Line7
#define US_ECHO_EXTI_PORT     GPIO_PortSourceGPIOC
#define US_ECHO_EXTI_PIN      GPIO_PinSource7
#define US_ECHO_IRQ           EXTI9_5_IRQn

/* Ultrasonic timer: TIM2_CH2 for echo pulse width measurement */
#define US_TIM                TIM2
#define US_TIM_CH             TIM_Channel_2
#define US_TIM_ARR            0xFFFF
#define US_TIM_PSC            71          /* 1MHz → 1us resolution */

/* ============================================================================
 * Bumper Sensors — GPIO Input with pull-down
 *
 * Bumper Left:  PD8 (HIGH = collision)
 * Bumper Right: PD10 (HIGH = collision)
 * ============================================================================ */
#define BUMP_L_PIN            GPIO_Pin_8
#define BUMP_L_GPIO           GPIOD
#define BUMP_L_EXTI_LINE      EXTI_Line8
#define BUMP_L_EXTI_PORT      GPIO_PortSourceGPIOD
#define BUMP_L_EXTI_PIN       GPIO_PinSource8
#define BUMP_L_IRQ            EXTI9_5_IRQn

#define BUMP_R_PIN            GPIO_Pin_10
#define BUMP_R_GPIO           GPIOD
#define BUMP_R_EXTI_LINE      EXTI_Line10
#define BUMP_R_EXTI_PORT      GPIO_PortSourceGPIOD
#define BUMP_R_EXTI_PIN       GPIO_PinSource10
#define BUMP_R_IRQ            EXTI15_10_IRQn

/* ============================================================================
 * Battery Voltage ADC — ADC1_CH6 (PA6)
 *
 * Voltage divider: 10:1, max 33V → 3.3V at ADC pin
 * Vref = 3.3V, 12-bit resolution
 * Vbat_mV = raw * 3300 / 4096 * 10 = raw * 8.056
 * ============================================================================ */
#define BAT_ADC               ADC1
#define BAT_ADC_CH            ADC_Channel_6
#define BAT_ADC_PIN           GPIO_Pin_6
#define BAT_ADC_GPIO          GPIOA
#define BAT_ADC_DMA           DMA1_Channel1

/* ============================================================================
 * LED Indicators — GPIO output (active HIGH)
 * ============================================================================ */
#define LED_STATUS_PIN        GPIO_Pin_3   /* PB3 (SWD disabled) */
#define LED_STATUS_GPIO       GPIOB
#define LED_ESTOP_PIN         GPIO_Pin_10  /* PB10 */
#define LED_ESTOP_GPIO        GPIOB

#define LED_ON(gpio, pin)     GPIO_SetBits(gpio, pin)
#define LED_OFF(gpio, pin)    GPIO_ResetBits(gpio, pin)
#define LED_TOGGLE(gpio, pin) GPIO_WriteBit(gpio, pin, \
    (BitAction)(1 - GPIO_ReadOutputDataBit(gpio, pin)))

/* ============================================================================
 * Timer for periodic interrupts — TIM6 (5ms tick)
 * ============================================================================ */
#define PERIODIC_TIM          TIM6
#define PERIODIC_ARR          4999
#define PERIODIC_PSC          71          /* 72MHz / 72 / 5000 = 200Hz = 5ms */
#define PERIODIC_IRQ          TIM6_IRQn

/* ============================================================================
 * Pin Remap Summary (from a-meter project)
 * ============================================================================ */
/* TIM1 fully remapped → PE9 (CH1), PE11 (CH2) */
/* TIM3 partially remapped → PB0 (CH3), PB1 (CH4) */
/* JTAG disabled, SWD only → PB3, PB4 released */

/* ============================================================================
 * FreeRTOS Config (imported from a-meter project)
 * ============================================================================ */
#include "FreeRTOSConfig.h"   /* Uses a-meter config: 1000Hz, 20KB heap */

#endif /* ROBOTCAR_HW_CONFIG_H */
