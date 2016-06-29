# Makefile for stm32-vserprog
# * Simply make: will make the firmware for default board.
# * Override toochain: make CROSS=/path/to/arm-none-eabi
# * Override UART for downloading: make SERIAL=/dev/ttyS1
# * Override hardware config: make BOARD=some_board_in_boards_folder
# * Build for GD32 variants with 12MHz crystal: make EXTRA_CFLAGS=-DGD32F103

###############################################################################

PROGRAM  = stm32-vserprog
CROSS   ?= arm-none-eabi
SERIAL  ?= /dev/ttyUSB0
OBJS     = vserprog.o \
           usbcdc.o \
           spi.o

DOCS     = README.html

###############################################################################

CC       = $(CROSS)-gcc
LD       = $(CROSS)-ld
OBJCOPY  = $(CROSS)-objcopy
OBJDUMP  = $(CROSS)-objdump
SIZE     = $(CROSS)-size

ELF      = $(PROGRAM).elf
BIN      = $(PROGRAM).bin
HEX      = $(PROGRAM).hex
MAP      = $(PROGRAM).map
DMP      = $(PROGRAM).out

ifneq ($(wildcard last_board.mk),)
  include last_board.mk
endif
ifdef BOARD
  include boards/$(BOARD).mk
endif

all:
ifndef BOARD
	@echo "Please specify a board by using 'make BOARD=board'."
	@echo "Available boards:"
	@echo "===================="
	@ls boards/*.h | cut -d '/' -f 2 | cut -d '.' -f 1
else
  ifndef LAST_BOARD
	@echo "First run, cleaning..."
	@make clean
  else
    ifneq ($(LAST_BOARD), $(BOARD))
	@echo "Board changed, cleaning..."
	@make clean
    endif
  endif
	@echo "LAST_BOARD = $(BOARD)" > last_board.mk
	@ln -sfT "boards/$(BOARD).h" board.h
	@make firmware
endif

CFLAGS  += -O3 -Wall -g
#CFLAGS  += -Os -Wall -g
#CFLAGS  += -Wextra -fprofile-generate -fprofile-use
CFLAGS  += -fno-common -ffunction-sections -fdata-sections
CFLAGS  += $(ARCH_FLAGS) -Ilibopencm3/include/ $(EXTRA_CFLAGS)

LIBM     = $(shell $(CC) $(CFLAGS) --print-file-name=libm.a)
LIBC     = $(shell $(CC) $(CFLAGS) --print-file-name=libc.a)
LIBNOSYS = $(shell $(CC) $(CFLAGS) --print-file-name=libnosys.a)
LIBGCC   = $(shell $(CC) $(CFLAGS) --print-libgcc-file-name)

# LDPATH is required for libopencm3 ld scripts to work.
LDPATH   = libopencm3/lib/
LDFLAGS += -L$(LDPATH) -T$(LDSCRIPT) -Map $(MAP) --gc-sections
LDLIBS  += $(LIBOPENCM3) $(LIBC) $(LIBNOSYS) $(LIBGCC)

firmware: $(LIBOPENCM3) $(BIN) $(HEX) $(DMP) size
docs: $(DOCS)

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


.PHONY: clean distclean flash flash-dfu reboot size

clean:
	rm -f $(OBJS) $(DOCS) $(ELF) $(HEX) $(BIN) $(MAP) $(DMP) board.h last_board.mk

distclean: clean
	make -C libopencm3 clean
	rm -f *~ *.swp *.hex

flash: $(HEX)
	stm32flash -w $< -v $(SERIAL)

flash-dfu: $(BIN)
	dfu-util -a 0 -d 0483:df11 -s 0x08000000:leave -D $<

reboot:
	stm32flash -g 0x0 $(SERIAL)

size: $(PROGRAM).elf
	@echo ""
	@$(SIZE) $(PROGRAM).elf
	@echo ""
