PROGRAM     = stm32-vserprog
CROSS       = arm-none-eabi
LDSCRIPT    = libopencm3/lib/stm32/f1/stm32f103x8.ld
SERIAL      = /dev/ttyUSB0
OBJS        = vserprog.o \
              usbcdc.o \
              spi.o \


DOCS        = README.html


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

DEFS        = -DSTM32F1
INCS        = -Ilibopencm3/include/
FP_FLAGS    = -msoft-float
ARCH_FLAGS  = -mthumb -mcpu=cortex-m3 $(FP_FLAGS) -mfix-cortex-m3-ldrd

CFLAGS     += -O3 -Wall -g
#CFLAGS     += -Os -Wall -g
#CFLAGS     += -Wextra -fprofile-generate -fprofile-use
CFLAGS     += -fno-common -ffunction-sections -fdata-sections
CFLAGS     += $(ARCH_FLAGS) $(DEFS) $(INCS)

LDPATH      = libopencm3/lib/

LIBM        = $(shell $(CC) $(CFLAGS) --print-file-name=libm.a)
LIBC        = $(shell $(CC) $(CFLAGS) --print-file-name=libc.a)
LIBNOSYS    = $(shell $(CC) $(CFLAGS) --print-file-name=libnosys.a)
LIBGCC      = $(shell $(CC) $(CFLAGS) --print-libgcc-file-name)
LIBOPENCM3  = $(LDPATH)/libopencm3_stm32f1.a

LDFLAGS    += -L$(LDPATH) -T$(LDSCRIPT) -Map $(MAP) --gc-sections
LDLIBS     += $(LIBOPENCM3) $(LIBC) $(LIBNOSYS) $(LIBGCC)

all: $(LDPATH)$(LIBOPENCM3) $(BIN) $(HEX) $(DMP) size

$(ELF): $(LDSCRIPT) $(OBJS)
	$(LD) -o $@ $(LDFLAGS) $(OBJS) $(LDLIBS)

$(DMP): $(ELF)
	$(OBJDUMP) -d $< > $@

%.hex: %.elf
	$(OBJCOPY) -S -O ihex   $< $@

%.bin: %.elf
	$(OBJCOPY) -S -O binary $< $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.html: %.md
	markdown $< > $@

$(LDPATH)$(LIBOPENCM3):
	git submodule init
	git submodule update --remote
	CFLAGS="$(CFLAGS)" make -C libopencm3 lib/stm32/f1 V=1

.PHONY: clean distclean flash size

clean:
	rm -f $(OBJS) $(DOCS) $(ELF) $(HEX) $(BIN) $(MAP) $(DMP)

distclean: clean
	make -C libopencm3 clean
	rm -f *~ *.swp

flash: $(HEX)
	stm32flash -w $< -v $(SERIAL)

reboot:
	stm32flash -g 0x0 $(SERIAL)

size: $(PROGRAM).elf
	@echo ""
	@$(SIZE) $(PROGRAM).elf
	@echo ""

docs: $(DOCS)
