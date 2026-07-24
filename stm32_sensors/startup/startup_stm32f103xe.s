/* Minimal startup for STM32F103 CET6 (Cortex-M3) */
.syntax unified
.arch armv7-m
.thumb

.section .isr_vector, "a", %progbits
.globl g_pfnVectors

.globl _estack
.globl Reset_Handler
.globl main

/* Default handler: infinite loop */
.weak Default_Handler
.type Default_Handler, %function
Default_Handler:
    b .

.macro def_irq name
    .weak \name
    .thumb_set \name, Default_Handler
.endm

g_pfnVectors:
    .word _estack
    .word Reset_Handler
    .word NMI_Handler
    .word HardFault_Handler
    .word MemManage_Handler
    .word BusFault_Handler
    .word UsageFault_Handler
    .word 0
    .word 0
    .word 0
    .word 0
    .word SVC_Handler
    .word DebugMon_Handler
    .word 0
    .word PendSV_Handler
    .word SysTick_Handler
    /* External IRQs 16-43 */
    .word WWDG_IRQHandler
    .word PVD_IRQHandler
    .word TAMPER_IRQHandler
    .word RTC_IRQHandler
    .word FLASH_IRQHandler
    .word RCC_IRQHandler
    .word EXTI0_IRQHandler
    .word EXTI1_IRQHandler
    .word EXTI2_IRQHandler
    .word EXTI3_IRQHandler
    .word EXTI4_IRQHandler
    .word DMA1_Channel1_IRQHandler
    .word DMA1_Channel2_IRQHandler
    .word DMA1_Channel3_IRQHandler
    .word DMA1_Channel4_IRQHandler
    .word DMA1_Channel5_IRQHandler
    .word DMA1_Channel6_IRQHandler
    .word DMA1_Channel7_IRQHandler
    .word ADC1_2_IRQHandler
    .word USB_HP_CAN1_TX_IRQHandler
    .word USB_LP_CAN1_RX0_IRQHandler
    .word CAN1_RX1_IRQHandler
    .word CAN1_SCE_IRQHandler
    .word EXTI9_5_IRQHandler
    .word TIM1_BRK_IRQHandler
    .word TIM1_UP_IRQHandler
    .word TIM1_TRG_COM_IRQHandler
    .word TIM1_CC_IRQHandler
    .word TIM2_IRQHandler
    .word TIM3_IRQHandler
    .word TIM4_IRQHandler
    .word I2C1_EV_IRQHandler
    .word I2C1_ER_IRQHandler
    .word I2C2_EV_IRQHandler
    .word I2C2_ER_IRQHandler
    .word SPI1_IRQHandler
    .word SPI2_IRQHandler
    .word USART1_IRQHandler
    .word USART2_IRQHandler
    .word USART3_IRQHandler
    .word EXTI15_10_IRQHandler
    .word RTC_Alarm_IRQHandler
    .word USBWakeUp_IRQHandler

/* Weak all ISRs */
def_irq NMI_Handler
def_irq HardFault_Handler
def_irq MemManage_Handler
def_irq BusFault_Handler
def_irq UsageFault_Handler
def_irq SVC_Handler
def_irq DebugMon_Handler
def_irq PendSV_Handler
def_irq SysTick_Handler
def_irq WWDG_IRQHandler
def_irq PVD_IRQHandler
def_irq TAMPER_IRQHandler
def_irq RTC_IRQHandler
def_irq FLASH_IRQHandler
def_irq RCC_IRQHandler
def_irq EXTI0_IRQHandler
def_irq EXTI1_IRQHandler
def_irq EXTI2_IRQHandler
def_irq EXTI3_IRQHandler
def_irq EXTI4_IRQHandler
def_irq DMA1_Channel1_IRQHandler
def_irq DMA1_Channel2_IRQHandler
def_irq DMA1_Channel3_IRQHandler
def_irq DMA1_Channel4_IRQHandler
def_irq DMA1_Channel5_IRQHandler
def_irq DMA1_Channel6_IRQHandler
def_irq DMA1_Channel7_IRQHandler
def_irq ADC1_2_IRQHandler
def_irq USB_HP_CAN1_TX_IRQHandler
def_irq USB_LP_CAN1_RX0_IRQHandler
def_irq CAN1_RX1_IRQHandler
def_irq CAN1_SCE_IRQHandler
def_irq EXTI9_5_IRQHandler
def_irq TIM1_BRK_IRQHandler
def_irq TIM1_UP_IRQHandler
def_irq TIM1_TRG_COM_IRQHandler
def_irq TIM1_CC_IRQHandler
def_irq TIM2_IRQHandler
def_irq TIM3_IRQHandler
def_irq TIM4_IRQHandler
def_irq I2C1_EV_IRQHandler
def_irq I2C1_ER_IRQHandler
def_irq I2C2_EV_IRQHandler
def_irq I2C2_ER_IRQHandler
def_irq SPI1_IRQHandler
def_irq SPI2_IRQHandler
def_irq USART1_IRQHandler
def_irq USART2_IRQHandler
def_irq USART3_IRQHandler
def_irq EXTI15_10_IRQHandler
def_irq RTC_Alarm_IRQHandler
def_irq USBWakeUp_IRQHandler

.section .text.Reset_Handler, "ax", %progbits
.type Reset_Handler, %function
Reset_Handler:
    ldr r0, =_estack
    msr msp, r0

    /* Copy .data from FLASH to RAM */
    ldr r0, =_sdata
    ldr r1, =_edata
    ldr r2, =_sidata
    b .L_data_check
.L_data_loop:
    ldr r3, [r2]
    str r3, [r0]
    adds r0, #4
    adds r2, #4
.L_data_check:
    cmp r0, r1
    bcc .L_data_loop

    /* Zero .bss */
    ldr r0, =_sbss
    ldr r1, =_ebss
    movs r2, #0
    b .L_bss_check
.L_bss_loop:
    str r2, [r0]
    adds r0, #4
.L_bss_check:
    cmp r0, r1
    bcc .L_bss_loop

    bl main
    b .

.end
