/**
 * @file    system_stm32f1xx.c
 * @brief   Minimal system init for STM32F103 — clock setup
 *
 * HSE: 8 MHz → PLL ×9 → SYSCLK = 72 MHz
 * AHB = 72 MHz, APB1 = 36 MHz, APB2 = 72 MHz
 *
 * Called by startup before main().
 */

#include <stdint.h>

/* RCC registers */
#define RCC_BASE        0x40021000UL
#define RCC_CR          (*(volatile uint32_t*)(RCC_BASE + 0x00))
#define RCC_CFGR        (*(volatile uint32_t*)(RCC_BASE + 0x04))
#define RCC_CR_HSEON    (1 << 16)
#define RCC_CR_HSERDY   (1 << 17)
#define RCC_CR_PLLON    (1 << 24)
#define RCC_CR_PLLRDY   (1 << 25)
#define RCC_CFGR_PLLMULL_9  (0x07 << 18)
#define RCC_CFGR_PLLSRC_HSE (1 << 16)
#define RCC_CFGR_SW_PLL     (0x02)
#define RCC_CFGR_SWS_PLL    (0x08)

/* Flash */
#define FLASH_ACR       (*(volatile uint32_t*)(0x40022000))
#define FLASH_ACR_LATENCY_2 (0x02)

uint32_t SystemCoreClock = 72000000;

void SystemInit(void) {
    /* Enable HSE */
    RCC_CR |= RCC_CR_HSEON;
    while (!(RCC_CR & RCC_CR_HSERDY));

    /* Flash: 2 wait states (required @ 72 MHz) */
    FLASH_ACR = FLASH_ACR_LATENCY_2;

    /* PLL: HSE × 9 = 72 MHz */
    RCC_CFGR |= RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL_9;
    RCC_CR |= RCC_CR_PLLON;
    while (!(RCC_CR & RCC_CR_PLLRDY));

    /* Switch system clock to PLL */
    RCC_CFGR = (RCC_CFGR & ~0x03) | RCC_CFGR_SW_PLL;
    while ((RCC_CFGR & 0x0C) != RCC_CFGR_SWS_PLL);
}

void SystemCoreClockUpdate(void) {
    SystemCoreClock = 72000000;
}
