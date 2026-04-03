TARGET  := scenario-sensors
MCU     := STM32F446xx
CPU     := cortex-m4
FPU     := fpv4-sp-d16
FLOAT   := hard

CC      := arm-none-eabi-gcc
AS      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE    := arm-none-eabi-size

BUILD_DIR := build

CUBE_F4 ?= ./STM32CubeF4
HAL_DIR := $(CUBE_F4)/Drivers/STM32F4xx_HAL_Driver
CMSIS_DIR := $(CUBE_F4)/Drivers/CMSIS

INCLUDES := \
  -I. \
  -I$(HAL_DIR)/Inc \
  -I$(HAL_DIR)/Inc/Legacy \
  -I$(CMSIS_DIR)/Device/ST/STM32F4xx/Include \
  -I$(CMSIS_DIR)/Include

CFLAGS := \
	-mcpu=$(CPU) -mthumb -mfpu=$(FPU) -mfloat-abi=$(FLOAT) \
	-D$(MCU) -DUSE_HAL_DRIVER \
	-std=c11 -Wall -Wextra -Werror \
	-ffunction-sections -fdata-sections \
	-O2 -g3 \
	$(INCLUDES)

ASFLAGS := \
	-mcpu=$(CPU) -mthumb -x assembler-with-cpp \
	-D$(MCU) \
	$(INCLUDES)

LDFLAGS := \
	-mcpu=$(CPU) -mthumb -mfpu=$(FPU) -mfloat-abi=$(FLOAT) \
	-TSTM32F446RETX_FLASH.ld \
	-Wl,--gc-sections \
	-Wl,-Map,$(BUILD_DIR)/$(TARGET).map \
	-specs=nano.specs \
	-specs=nosys.specs

STARTUP := startup_stm32f446xx.s

SRCS := \
	main.c \
	syscalls.c \
	sysmem.c \
	system_stm32f4xx.c \
	$(STARTUP) \
	$(HAL_DIR)/Src/stm32f4xx_hal.c \
	$(HAL_DIR)/Src/stm32f4xx_hal_cortex.c \
	$(HAL_DIR)/Src/stm32f4xx_hal_dma.c \
	$(HAL_DIR)/Src/stm32f4xx_hal_gpio.c \
	$(HAL_DIR)/Src/stm32f4xx_hal_rcc.c \
	$(HAL_DIR)/Src/stm32f4xx_hal_rcc_ex.c \
	$(HAL_DIR)/Src/stm32f4xx_hal_pwr.c \
	$(HAL_DIR)/Src/stm32f4xx_hal_pwr_ex.c \
	$(HAL_DIR)/Src/stm32f4xx_hal_i2c.c \
	$(HAL_DIR)/Src/stm32f4xx_hal_i2c_ex.c \
	$(HAL_DIR)/Src/stm32f4xx_hal_uart.c

OBJS := \
	$(BUILD_DIR)/main.o \
	$(BUILD_DIR)/syscalls.o \
	$(BUILD_DIR)/sysmem.o \
	$(BUILD_DIR)/system_stm32f4xx.o \
	$(BUILD_DIR)/startup_stm32f446xx.o \
	$(BUILD_DIR)/stm32f4xx_hal.o \
	$(BUILD_DIR)/stm32f4xx_hal_cortex.o \
	$(BUILD_DIR)/stm32f4xx_hal_dma.o \
	$(BUILD_DIR)/stm32f4xx_hal_gpio.o \
	$(BUILD_DIR)/stm32f4xx_hal_rcc.o \
	$(BUILD_DIR)/stm32f4xx_hal_rcc_ex.o \
	$(BUILD_DIR)/stm32f4xx_hal_pwr.o \
	$(BUILD_DIR)/stm32f4xx_hal_pwr_ex.o \
	$(BUILD_DIR)/stm32f4xx_hal_i2c.o \
	$(BUILD_DIR)/stm32f4xx_hal_i2c_ex.o \
	$(BUILD_DIR)/stm32f4xx_hal_uart.o

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/$(TARGET).hex

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/main.o: main.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/syscalls.o: syscalls.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/sysmem.o: sysmem.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/system_stm32f4xx.o: system_stm32f4xx.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/startup_stm32f446xx.o: $(STARTUP) | $(BUILD_DIR)
	$(AS) $(ASFLAGS) -c $< -o $@

$(BUILD_DIR)/stm32f4xx_hal.o: $(HAL_DIR)/Src/stm32f4xx_hal.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/stm32f4xx_hal_cortex.o: $(HAL_DIR)/Src/stm32f4xx_hal_cortex.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/stm32f4xx_hal_dma.o: $(HAL_DIR)/Src/stm32f4xx_hal_dma.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/stm32f4xx_hal_gpio.o: $(HAL_DIR)/Src/stm32f4xx_hal_gpio.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/stm32f4xx_hal_rcc.o: $(HAL_DIR)/Src/stm32f4xx_hal_rcc.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/stm32f4xx_hal_rcc_ex.o: $(HAL_DIR)/Src/stm32f4xx_hal_rcc_ex.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/stm32f4xx_hal_pwr.o: $(HAL_DIR)/Src/stm32f4xx_hal_pwr.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/stm32f4xx_hal_pwr_ex.o: $(HAL_DIR)/Src/stm32f4xx_hal_pwr_ex.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/stm32f4xx_hal_i2c.o: $(HAL_DIR)/Src/stm32f4xx_hal_i2c.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/stm32f4xx_hal_i2c_ex.o: $(HAL_DIR)/Src/stm32f4xx_hal_i2c_ex.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/stm32f4xx_hal_uart.o: $(HAL_DIR)/Src/stm32f4xx_hal_uart.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

clean:
	rm -rf $(BUILD_DIR)

flash: $(BUILD_DIR)/$(TARGET).elf
	st-flash write $(BUILD_DIR)/$(TARGET).bin 0x08000000

.PHONY: all clean flash
