# usb-node-stm32h7: a USB device under test on a NUCLEO-H753ZI (README.md).
#   make deps    once: ST's CMSIS headers and TinyUSB, pinned (not vendored)
#   make         build/usb-node.elf, .bin, .hex
#   make flash   over the Nucleo's own ST-LINK (not the pod)
TARGET  := usb-node
CC      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE    := arm-none-eabi-size
BUILD   := build

DEV  := lib/cmsis_device_h7
CORE := lib/cmsis_core/CMSIS/Core/Include
TUSB := lib/tinyusb/src

VERSION ?= $(shell git describe --always --dirty 2>/dev/null || echo dev)

ARCH := -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
CFLAGS_COMMON := $(ARCH) -O2 -g3 -std=gnu11 -ffunction-sections -fdata-sections -DSTM32H753xx \
  -I. -I$(DEV)/Include -I$(CORE) -I$(TUSB)
CFLAGS     := $(CFLAGS_COMMON) -Wall -Wextra -Werror -DUSBNODE_VERSION='"$(VERSION)"'
CFLAGS_LIB := $(CFLAGS_COMMON) -w
LDFLAGS := $(ARCH) -T link.ld -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).map --specs=nano.specs --specs=nosys.specs -Wl,--no-warn-rwx-segments

APP_SRCS := main.c board.c disk.c usb_descriptors.c syscalls.c
LIB_SRCS := $(DEV)/Source/Templates/system_stm32h7xx.c \
  $(TUSB)/tusb.c $(TUSB)/common/tusb_fifo.c \
  $(TUSB)/device/usbd.c \
  $(TUSB)/class/cdc/cdc_device.c $(TUSB)/class/msc/msc_device.c $(TUSB)/class/hid/hid_device.c \
  $(TUSB)/portable/synopsys/dwc2/dcd_dwc2.c $(TUSB)/portable/synopsys/dwc2/dwc2_common.c
STARTUP := $(DEV)/Source/Templates/gcc/startup_stm32h753xx.s

APP_OBJS := $(APP_SRCS:%.c=$(BUILD)/%.o)
LIB_OBJS := $(patsubst %.c,$(BUILD)/lib/%.o,$(notdir $(LIB_SRCS)))
OBJS := $(APP_OBJS) $(LIB_OBJS) $(BUILD)/startup.o
vpath %.c $(sort $(dir $(LIB_SRCS)))

all: $(BUILD)/$(TARGET).bin $(BUILD)/$(TARGET).hex

$(BUILD)/%.o: %.c *.h | $(BUILD)/lib
	$(CC) $(CFLAGS) -c $< -o $@
$(BUILD)/lib/%.o: %.c tusb_config.h | $(BUILD)/lib
	$(CC) $(CFLAGS_LIB) -c $< -o $@
$(BUILD)/startup.o: $(STARTUP) | $(BUILD)/lib
	$(CC) $(ARCH) -x assembler-with-cpp -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJS) link.ld
	$(CC) $(OBJS) $(LDFLAGS) -o $@
	$(SIZE) $@
$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@
$(BUILD)/$(TARGET).hex: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD)/lib:
	mkdir -p $@

deps:
	test -d $(DEV) || git clone -q https://github.com/STMicroelectronics/cmsis_device_h7.git $(DEV)
	git -C $(DEV) checkout -q 81db1ec63cdc191fae1565b772da3ea5aa29a683
	test -d lib/cmsis_core || git clone -q https://github.com/STMicroelectronics/cmsis_core.git lib/cmsis_core
	git -C lib/cmsis_core checkout -q 01d18147527e30ed69ace1cb03b818de78bc7062
	test -d lib/tinyusb || git clone -q https://github.com/hathach/tinyusb.git lib/tinyusb
	git -C lib/tinyusb checkout -q 91cdc7bbb4d85a3ecb4c8c640297d9c401ff4390

# st-flash may fail with error 255 on macOS: `make flash-openocd` then.
flash: $(BUILD)/$(TARGET).bin
	st-flash --reset write $< 0x08000000
flash-openocd: $(BUILD)/$(TARGET).elf
	openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c "program $< verify reset exit"

clean:
	rm -rf $(BUILD)

.PHONY: all deps flash flash-openocd clean

# The pod's MSC flasher against disk.c, on this computer (no hardware). BENCHPOD_FW = benchpod-firmware/stm32h563/src.
BENCHPOD_FW ?= ../../benchpod-firmware/stm32h563/src
host-check:
	mkdir -p $(BUILD)/host
	sed 's/__attribute__((section(".disk"), aligned(32)))//' disk.c > $(BUILD)/host/disk.c
	cc -O1 -Wall -Itests/host -I. -I$(BENCHPOD_FW) tests/host/check_msc.c $(BUILD)/host/disk.c \
	  $(BENCHPOD_FW)/xip_mod_msc_flash.c $(BENCHPOD_FW)/msc_flash_names.c -o $(BUILD)/host/check_msc
	$(BUILD)/host/check_msc

.PHONY: host-check
