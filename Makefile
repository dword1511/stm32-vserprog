# Makefile for stm32-vserprog
# * Simply make: will make the firmware for default board.
# * Override toochain: make CROSS=/path/to/arm-none-eabi
# * Override UART for downloading: make SERIAL=/dev/ttyS1
# * Override hardware config: make BOARD=some_board_in_boards_folder

###############################################################################

PROGRAM     = stm32-vserprog
CROSS      ?= arm-none-eabi
SERIAL     ?= /dev/ttyUSB0
BOARD      ?= stm32-vserprog-v3
OBJS        = vserprog.o \
              usbcdc.o \
              spi.o \


DOCS        = README.html \



###############################################################################

CC          = $(CROSS)-gcc
LD          = $(CROSS)-ld
OBJCOPY     = $(CROSS)-objcopy
OBJDUMP     = $(CROSS)-objdump
SIZE        = $(CROSS)-size

ELF         = $(PROGRAM).elf
BIN         = $(PROGRAM).bin
HEX         = $(PROGRAM).hex
MAP         = $(PROGRAM).map
DMP         = $(PROGRAM).out

include boards/$(BOARD).mk

CFLAGS     += -O3 -Wall -g
#CFLAGS     += -Os -Wall -g
#CFLAGS     += -Wextra -fprofile-generate -fprofile-use
CFLAGS     += -fno-common -ffunction-sections -fdata-sections
CFLAGS     += $(ARCH_FLAGS) -Ilibopencm3/include/

LIBM        = $(shell $(CC) $(CFLAGS) --print-file-name=libm.a)
LIBC        = $(shell $(CC) $(CFLAGS) --print-file-name=libc.a)
LIBNOSYS    = $(shell $(CC) $(CFLAGS) --print-file-name=libnosys.a)
LIBGCC      = $(shell $(CC) $(CFLAGS) --print-libgcc-file-name)

# LDPATH is required for libopencm3 ld scripts to work.
LDPATH      = libopencm3/lib/
LDFLAGS    += -L$(LDPATH) -T$(LDSCRIPT) -Map $(MAP) --gc-sections
LDLIBS     += $(LIBOPENCM3) $(LIBC) $(LIBNOSYS) $(LIBGCC)

all: $(LIBOPENCM3) $(BIN) $(HEX) $(DMP) size

$(ELF): $(LDSCRIPT) $(OBJS)
	$(LD) -o $@ $(LDFLAGS) $(OBJS) $(LDLIBS)

$(DMP): $(ELF)
	$(OBJDUMP) -d $< > $@

%.hex: %.elf
	$(OBJCOPY) -S -O ihex   $< $@
	cp $@ $(PROGRAM)-$(BOARD).hex

%.bin: %.elf
	$(OBJCOPY) -S -O binary $< $@

%.o: %.c board.h
	$(CC) $(CFLAGS) -c $< -o $@

%.html: %.md
	markdown $< > $@

$(LIBOPENCM3):
	git submodule init
	git submodule update --remote
	CFLAGS="$(CFLAGS)" make -C libopencm3 $(OPENCM3_MK) V=1


.PHONY: board.h clean distclean flash size

board.h:
	ln -sfT "boards/$(BOARD).h" $@

clean:
	rm -f $(OBJS) $(DOCS) $(ELF) $(HEX) $(BIN) $(MAP) $(DMP) board.h

distclean: clean
	make -C libopencm3 clean
	rm -f *~ *.swp *.hex

flash: $(HEX)
	stm32flash -w $< -v $(SERIAL)

reboot:
	stm32flash -g 0x0 $(SERIAL)

size: $(PROGRAM).elf
	@echo ""
	@$(SIZE) $(PROGRAM).elf
	@echo ""

docs: $(DOCS)
