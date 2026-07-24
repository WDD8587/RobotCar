# RobotCar STM32F103 Sensor Hub — Makefile
# FWLib from a-meter-sweeping-robot project (NOT modified)
# FreeRTOS from this project (imported from a-meter)
# Build: make

TARGET    = sensor_hub

# Toolchain
CC        = arm-none-eabi-gcc
OBJCOPY   = arm-none-eabi-objcopy
SIZE      = arm-none-eabi-size

# FWLib path (original project, NO modifications)
FWLIB_DIR = C:/Users/EC/Desktop/a-meter-sweeping-robot-stm32-master/firmware
FWLIB_SRC = $(FWLIB_DIR)/STM32F10x_FWLib/src
FWLIB_INC = $(FWLIB_DIR)/STM32F10x_FWLib/inc
CORE_INC  = $(FWLIB_DIR)/CORE
USER_INC  = $(FWLIB_DIR)/USER

# CPU flags
CFLAGS   = -mcpu=cortex-m3 -mthumb -O2 -std=gnu11 -Wall -Wextra
CFLAGS  += -DSTM32F10X_HD -DUSE_STDPERIPH_DRIVER
CFLAGS  += -ffunction-sections -fdata-sections -fno-common

# Include paths
INC      = -I Core -I .
INC     += -I $(FWLIB_INC)
INC     += -I $(CORE_INC)
INC     += -I $(USER_INC)
INC     += -I $(FWLIB_DIR)/FreeRTOS/include
INC     += -I FreeRTOS
INC     += -I $(FWLIB_DIR)/HARDWARE
INC     += -I $(FWLIB_DIR)/SYSTEM/sys
INC     += -I $(FWLIB_DIR)/SYSTEM/usart
INC     += -I $(FWLIB_DIR)/SYSTEM/usart1
INC     += -I $(FWLIB_DIR)/SYSTEM/delay
INC     += -I ../shared
INC     += -I Protocol

# Linker
LDFLAGS  = -T stm32f103c8tx_flash.ld
LDFLAGS += -Wl,--gc-sections -nostartfiles -lc -lm --specs=nosys.specs

# Source files
SRCS  = Core/main.c
SRCS += Core/system_stm32f1xx.c
SRCS += startup/startup_stm32f103xe.s
SRCS += Protocol/protocol.c

# FWLib sources (minimal set — add as needed)
SRCS += $(FWLIB_SRC)/misc.c
SRCS += $(FWLIB_SRC)/stm32f10x_gpio.c
SRCS += $(FWLIB_SRC)/stm32f10x_rcc.c
SRCS += $(FWLIB_SRC)/stm32f10x_usart.c
SRCS += $(FWLIB_SRC)/stm32f10x_tim.c
SRCS += $(FWLIB_SRC)/stm32f10x_adc.c
SRCS += $(FWLIB_SRC)/stm32f10x_exti.c
SRCS += $(FWLIB_SRC)/stm32f10x_dma.c
SRCS += $(FWLIB_SRC)/stm32f10x_flash.c

# FreeRTOS sources
SRCS += $(FWLIB_DIR)/FreeRTOS/croutine.c
SRCS += $(FWLIB_DIR)/FreeRTOS/event_groups.c
SRCS += $(FWLIB_DIR)/FreeRTOS/list.c
SRCS += $(FWLIB_DIR)/FreeRTOS/queue.c
SRCS += $(FWLIB_DIR)/FreeRTOS/tasks.c
SRCS += $(FWLIB_DIR)/FreeRTOS/timers.c
SRCS += FreeRTOS/port.c
SRCS += $(FWLIB_DIR)/FreeRTOS/portable/MemMang/heap_2.c

OBJS = $(patsubst %.c,%.o,$(patsubst %.s,%.o,$(SRCS)))

.PHONY: all clean flash

all: $(TARGET).bin

$(TARGET).elf: $(OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^
	$(SIZE) $@

%.o: %.c
	$(CC) $(CFLAGS) $(INC) -c -o $@ $<

%.o: %.s
	$(CC) $(CFLAGS) -c -o $@ $<

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

clean:
	rm -f $(OBJS) $(TARGET).elf $(TARGET).bin

flash: $(TARGET).bin
	st-flash write $(TARGET).bin 0x08000000
