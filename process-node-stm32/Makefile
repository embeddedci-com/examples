TARGET  := process-node
MCU     := STM32F446xx
CPU     := cortex-m4
FPU     := fpv4-sp-d16
FLOAT   := hard

CC      := arm-none-eabi-gcc
AS      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE    := arm-none-eabi-size

BUILD_DIR := build

CUBE_F4 ?= /opt/STM32CubeF4
HAL_DIR := $(CUBE_F4)/Drivers/STM32F4xx_HAL_Driver
CMSIS_DIR := $(CUBE_F4)/Drivers/CMSIS

# Crystal on the MCP2515 module (8 MHz on most; the console "can osc" changes it at runtime).
MCP2515_OSC_HZ ?= 8000000

INCLUDES := \
  -I. \
  -I$(HAL_DIR)/Inc \
  -I$(HAL_DIR)/Inc/Legacy \
  -I$(CMSIS_DIR)/Device/ST/STM32F4xx/Include \
  -I$(CMSIS_DIR)/Include

CFLAGS := \
	-mcpu=$(CPU) -mthumb -mfpu=$(FPU) -mfloat-abi=$(FLOAT) \
	-D$(MCU) -DUSE_HAL_DRIVER -DMCP2515_OSC_HZ=$(MCP2515_OSC_HZ)U \
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

APP_SRCS := main.c mcp2515.c syscalls.c sysmem.c system_stm32f4xx.c

HAL_SRCS := \
	stm32f4xx_hal.c \
	stm32f4xx_hal_cortex.c \
	stm32f4xx_hal_dma.c \
	stm32f4xx_hal_gpio.c \
	stm32f4xx_hal_rcc.c \
	stm32f4xx_hal_rcc_ex.c \
	stm32f4xx_hal_pwr.c \
	stm32f4xx_hal_pwr_ex.c

OBJS := \
	$(APP_SRCS:%.c=$(BUILD_DIR)/%.o) \
	$(HAL_SRCS:%.c=$(BUILD_DIR)/%.o) \
	$(BUILD_DIR)/startup_stm32f446xx.o

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/$(TARGET).hex

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: %.c mcp2515.h mcp2515_timing.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: $(HAL_DIR)/Src/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/startup_stm32f446xx.o: startup_stm32f446xx.s | $(BUILD_DIR)
	$(AS) $(ASFLAGS) -c $< -o $@

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

# Host unit test for the MCP2515 bit-timing solver (no MCU/HAL).
HOSTCC ?= cc
test-host: | $(BUILD_DIR)
	$(HOSTCC) -std=c11 -Wall -Wextra -Werror -I. tests/test_mcp2515_timing.c -o $(BUILD_DIR)/test_mcp2515_timing
	$(BUILD_DIR)/test_mcp2515_timing

.PHONY: all clean flash test-host
