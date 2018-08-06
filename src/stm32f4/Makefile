# Additional STM32F4 build rules

# Setup the toolchain
CROSS_PREFIX=arm-none-eabi-

dirs-y += src/stm32f4 src/generic
dirs-y += lib/cmsis-stm32f4/source
dirs-y += lib/hal-stm32f4/source

CFLAGS += -mthumb -mcpu=cortex-m4
CFLAGS += -Ilib/cmsis-stm32f4/include -Ilib/cmsis-stm32f4/cmsis-include 
CFLAGS += -Ilib/hal-stm32f4/include
CFLAGS += -DSTM32F446xx -DSTM32F4XX

CFLAGS_klipper.elf += -Llib/cmsis-stm32f4/source
CFLAGS_klipper.elf += -Tlib/cmsis-stm32f4/source/stm32f4xx.ld
CFLAGS_klipper.elf += --specs=nano.specs --specs=nosys.specs

# Extra build rules
$(OUT)%.o: %.s $(OUT)autoconf.h $(OUT)board-link
	@echo "  Assembling $@"
	$(Q)$(AS) $< -o $@

# Add source files
src-y += stm32f4/main.c stm32f4/timer.c stm32f4/gpio.c stm32f4/watchdog.c stm32f4/spi.c
src-y += $(addprefix ../, $(wildcard lib/hal-stm32f4/source/stm32f4xx_ll_*.c))
src-y += generic/crc16_ccitt.c generic/armcm_irq.c generic/timer_irq.c
src-y += ../lib/cmsis-stm32f4/source/system_stm32f4xx.c
src-ys = ../lib/cmsis-stm32f4/source/startup_stm32f446xx.s
src-$(CONFIG_SERIAL) += stm32f4/serial.c generic/serial_irq.c


# Build the additional hex output file
target-y += $(OUT)klipper.bin

# Add assembler objects to prerequisites list
$(OUT)klipper.elf: $(patsubst %.s, $(OUT)src/%.o,$(src-ys))

$(OUT)klipper.bin: $(OUT)klipper.elf
	@echo "  Creating hex file $@"
	$(Q)$(OBJCOPY) -O binary $< $@

flash: $(OUT)klipper.bin
	@echo "  Flashing via st-flash"
	$(Q)st-flash write $(OUT)klipper.bin 0x8000000