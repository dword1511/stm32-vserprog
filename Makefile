# Makefile for stm32-vserprog
# * Simply make: will make the firmware for default board.
# * Override toochain: make CROSS=/path/to/arm-none-eabi-
# * Override UART for downloading: make SERIAL=/dev/ttyS1 flash
# * Override hardware config: make BOARD=some_board_in_boards_folder
# * Test and benchmark: make PSERIAL=/dev/ttyACM0 SPISPD=50000000 test (Upon failure please check both programmer and flash)
# * Build for GD32 variants with 12MHz crystal: make EXTRA_CFLAGS=-DGD32F103

###############################################################################

PROGRAM  = stm32-vserprog
CROSS   ?= arm-none-eabi-
SERIAL  ?= /dev/ttyUSB0
PSERIAL ?= /dev/ttyACM0
SPISPD  ?= 1000000000
OBJS     = vserprog.o \
           usbcdc.o \
           spi.o

DOCS     = README.html

###############################################################################

CC       = $(CROSS)gcc
LD       = $(CROSS)ld
OBJCOPY  = $(CROSS)objcopy
OBJDUMP  = $(CROSS)objdump
SIZE     = $(CROSS)size
NM       = $(CROSS)nm

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
	@echo "Please specify a board by using '$(MAKE) BOARD=board'."
	@echo "Available boards:"
	@echo "===================="
	@ls boards/*.h | cut -d '/' -f 2 | cut -d '.' -f 1
else
  ifndef LAST_BOARD
	@echo "First run, cleaning..."
	@$(MAKE) clean
  else
    ifneq ($(LAST_BOARD), $(BOARD))
	@echo "Board changed, cleaning..."
	@$(MAKE) clean
    endif
  endif
	@echo "LAST_BOARD = $(BOARD)" > last_board.mk
	@ln -sfT "boards/$(BOARD).h" board.h
	@$(MAKE) firmware
endif

CFLAGS  += -O3 -Wall -g -std=gnu99
#CFLAGS  += -Os -Wall -g
#CFLAGS  += -Wextra -fprofile-generate -fprofile-use
CFLAGS  += -fno-common -ffunction-sections -fdata-sections -funit-at-a-time
CFLAGS  += -fgcse-sm -fgcse-las -fgcse-after-reload -funswitch-loops
#CFLAGS  += -funroll-loops -funsafe-loop-optimizations -fipa-pta -flto
CFLAGS  += $(ARCH_FLAGS) -Ilibopencm3/include/ $(EXTRA_CFLAGS)

LIBC     = $(shell $(CC) $(CFLAGS) --print-file-name=libc.a)
LIBGCC   = $(shell $(CC) $(CFLAGS) --print-libgcc-file-name)

# LDPATH is required for libopencm3 ld scripts to work.
LDPATH   = libopencm3/lib/
LDFLAGS += -L$(LDPATH) -T$(LDSCRIPT) -Map $(MAP) --gc-sections
LDLIBS  += $(LIBOPENCM3) $(LIBC) $(LIBGCC)


.PHONY: firmware docs clean distclean flash flash-dfu reboot size symbols test

firmware: $(BIN) $(HEX) $(DMP) size
docs: $(DOCS)

$(ELF): $(LDSCRIPT) $(OBJS) $(LIBOPENCM3)
	$(LD) -o $@ $(LDFLAGS) $(OBJS) $(LDLIBS)

$(DMP): $(ELF)
	$(OBJDUMP) -d $< > $@

%.hex: %.elf
	$(OBJCOPY) -S -O ihex   $< $@

%.bin: %.elf
	$(OBJCOPY) -S -O binary $< $@

%.o: %.c board.h $(LIBOPENCM3)
	$(CC) $(CFLAGS) -c $< -o $@

%.html: %.md
	markdown $< > $@

$(LIBOPENCM3):
	git submodule init
	git submodule update --init
	CFLAGS="$(CFLAGS)" $(MAKE) -C libopencm3 $(OPENCM3_MK) V=1

flashrom/flashrom:
	git submodule init
	git submodule update --init
	$(MAKE) -C flashrom

clean:
	rm -f $(OBJS) $(DOCS) $(ELF) $(HEX) $(BIN) $(MAP) $(DMP) board.h last_board.mk

distclean: clean
	$(MAKE) -C libopencm3 clean
	$(MAKE) -C flashrom distclean
	rm -f *~ *.swp *.hex *.bin

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

symbols: $(ELF)
	@$(NM) --demangle --size-sort -S $< | grep -v ' [bB] '


# Erasing must come first, otherwise some sectors might be skipped.
# After testing you may clear flash contents manually.
test: flashrom/flashrom
	@echo ""; \
	FFLAGS="-p serprog:dev=$(PSERIAL),spispeed=$(SPISPD)"; \
	echo "Detecting flash..."; \
	FOUT=`command time -f 'XXTIMEXX %e' $< $${FFLAGS} 2>&1`; \
	FPART=`echo "$${FOUT}" | grep "Found" | grep -oP '\".*\"' | sed -e 's/"//g'`; \
	FSIZE=`echo "$${FOUT}" | grep -oP '[0-9]+ kB' | sed -e 's/ kB//g'`; \
	PTIME=`echo "$${FOUT}" | grep 'XXTIMEXX' | sed -e 's/XXTIMEXX //g'`; \
	echo "Generating test file..."; \
	dd if=/dev/urandom iflag=fullblock of=rand.bin bs=1024 count=$${FSIZE} 2>/dev/null; \
	echo "Erasing..."; \
	ETIME=`command time -f 'XXTIMEXX %e' $< $${FFLAGS} -E 2>&1 >/dev/null | grep 'XXTIMEXX' | sed -e 's/XXTIMEXX //g'`; \
	echo -n "Verifying... "; \
	$< $${FFLAGS} -r compare.bin 1>/dev/null 2>/dev/null; \
	STR_VERIFY=`sed 's/\xff//g' compare.bin | hd`; \
	if test -z "$${STR_VERIFY}"; \
	then echo "PASS"; \
	else echo "FAIL"; \
	exit 1; \
	fi; \
	echo "Writing..."; \
	WTIME=`command time -f 'XXTIMEXX %e' $< $${FFLAGS} -w rand.bin 2>&1 >/dev/null | grep 'XXTIMEXX' | sed -e 's/XXTIMEXX //g'`; \
	echo "Reading..."; \
	RTIME=`command time -f 'XXTIMEXX %e' $< $${FFLAGS} -r compare.bin 2>&1 >/dev/null | grep 'XXTIMEXX' | sed -e 's/XXTIMEXX //g'`; \
	echo -n "Comparing..."; \
	CKSUMS=`md5sum rand.bin compare.bin | cut -d ' ' -f 1 | uniq | wc -l`; \
	if test "$${CKSUMS}" = "1"; \
	then echo "PASS"; \
	else echo "FAIL"; \
	fi; \
	echo ""; \
	echo "Flash type: $${FPART}"; \
	echo "Flash size: $${FSIZE} KiB"; \
	echo "SPI speed : $(SPISPD) Hz (requested)"; \
	printf 'Probe     : %6.2fs\n' "$${PTIME}"; \
	printf 'Erase     : %6.2fs\n' "$${ETIME}"; \
	printf 'Write     : %6.2fs\n' "$${WTIME}"; \
	printf 'Read      : %6.2fs\n' "$${RTIME}"; \
	echo ""; \
	rm rand.bin compare.bin;
