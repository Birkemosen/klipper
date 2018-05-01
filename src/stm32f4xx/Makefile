# Additional STM32F4 build rules

# Setup the toolchain
CROSS_PREFIX=arm-none-eabi-

dirs-y += src/stm32f4xx src/generic
dirs-y += lib/cmsis-stm32f4xx/source
dirs-y += lib/hal-stm32f4xx/source

CFLAGS += -mthumb -mcpu=cortex-m4
CFLAGS += -Ilib/cmsis-stm32f4xx/include -Ilib/cmsis-stm32f4xx/cmsis-include 
CFLAGS += -Ilib/hal-stm32f4xx/include
CFLAGS += -Dstm32f446xx -DSTM32F4XX

CFLAGS_klipper.elf += -Llib/cmsis-stm32f4xx/source
CFLAGS_klipper.elf += -Tlib/cmsis-stm32f4xx/source/stm32f4.ld
CFLAGS_klipper.elf += --specs=nano.specs --specs=nosys.specs

# Extra build rules
$(OUT)%.o: %.s $(OUT)autoconf.h $(OUT)board-link
	@echo "  Assembling $@"
	$(Q)$(AS) $< -o $@

# Add source files
src-y += stm32f4xx/main.c stm32f4xx/timer.c stm32f4xx/gpio.c stm32f4xx/watchdog.c
src-y += $(addprefix ../, $(wildcard lib/hal-stm32f4xx/source/stm32f4xx_ll_*.c))
src-y += generic/crc16_ccitt.c generic/armcm_irq.c generic/timer_irq.c
src-y += stm32f4xx/system_stm32f4xx.c
src-ys = ../lib/cmsis-stm32f4xx/source/startup_stm32f446xx.s
src-$(CONFIG_SERIAL) += stm32f4xx/serial.c generic/serial_irq.c

# Build the additional hex output file
target-y += $(OUT)klipper.bin

# Add assembler objects to prerequisites list
$(OUT)klipper.elf: $(patsubst %.s, $(OUT)src/%.o,$(src-ys))

$(OUT)klipper.bin: $(OUT)klipper.elf
	@echo "  Creating hex file $@"
	$(Q)$(OBJCOPY) -O binary $< $@

flash: $(OUT)klipper.bin
	@echo "  Flashing $^ to $(FLASH_DEVICE) via stm32flash"
	$(Q)stm32flash -w $(OUT)klipper.bin -v -g 0 $(FLASH_DEVICE)