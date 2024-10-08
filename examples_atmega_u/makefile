
### SOURCE DEPENDENCIES: libavr/avr
### BINARY DEPENDENCIES: libavr/avr

# --- Toolchain Config ---
CC := avr-gcc
testcc := gcc
OBJ2HEX := avr-objcopy

# Settings for Arduino Leonardo:
# 16 MHz MCU clock, 64 clock cycles per scheduler tick
f_cpu := 16000000
f_cpu_prescale := 6
mcu := atmega32u4

avrdude := avrdude

# Settings for Arduino Leonardo:
avrdude_port := /dev/ttyACM0
avrdude_device := m32u4
avrdude_prog := avr109
avrdude_baud := 115200
avrdude_args := -D

# --- Directory Variables ---
out_dir := .
lib_out_dir := $(out_dir)/lib
libavr_dir := ../avr
custom_libavr := yes

ifdef debug
rel_odir := obj/debug
rel_bdir := bin/debug
else
rel_odir := obj/dist
rel_bdir := bin/dist
endif

odir := $(out_dir)/$(rel_odir)
bdir := $(out_dir)/$(rel_bdir)
builddirs := $(odir) $(bdir)

ifeq ($(custom_libavr),yes)
ldirs_paths := $(lib_out_dir)/$(rel_bdir)_atmega_u
else
ldirs_paths := $(libavr_dir)/$(rel_bdir)_atmega_u
endif

# --- Filename Variables ---
targets  := sched_blinker mcp4x_test nrf24x_test lcd_driver
programs := $(addprefix prog-, $(targets))

headers := $(libavr_dir)/*.h

target_o   := $(addprefix $(odir)/, $(addsuffix .o, $(targets)))
target_obj := $(addprefix $(odir)/, $(addsuffix .obj, $(targets)))
target_hex := $(addprefix $(bdir)/, $(addsuffix .hex, $(targets)))

# --- Command Option Variables ---
warnings := -Wall -Wmissing-prototypes -Wstrict-prototypes
macro_defs := -DF_CPU=$(f_cpu)L -DLIBAVR_ATMEGA_U -DSCHED_CLOCK_PRESCALE_LOG=$(f_cpu_prescale)
macro_defs += -DSPI_NO_ASYNC_API

idirs := -I$(libavr_dir)
ldirs := $(addprefix -L, $(ldirs_paths))
lnames := -lavrtask -lm

CFLAGS := -std=c11 $(macro_defs) -g $(warnings) -mcall-prologues -mmcu=$(mcu) -Os -flto
LDFLAGS := -fuse-linker-plugin -Wl,--gc-sections -Wl,--print-gc-sections -Wl,--relax

# --- Phony Targets ---
.PHONY : all $(programs) $(targets) clean clean-libavr libavr

all : $(targets)

$(programs) : prog-% : %
$(programs) : prog-% : $(bdir)/%.hex
	$(avrdude) -v -p $(avrdude_device) -c $(avrdude_prog) -P $(avrdude_port) -b $(avrdude_baud) \
	           $(avrdude_args) -U flash:w:$<

# TODO: Add a command that programs the EEPROM (with a hex file that only
#       contains the .eeprom section).

ifeq ($(custom_libavr),yes)
$(targets) : libavr
clean : clean-libavr
endif

$(targets) : % : $(bdir)/%.hex

clean :
	$(RM) $(odir)/*.o $(odir)/*.obj $(bdir)/*.hex

clean-libavr : | $(lib_out_dir)
	$(MAKE) -C $(libavr_dir) build=atmega_u out_dir=$(realpath $(lib_out_dir)) clean

libavr : | $(lib_out_dir)
	$(MAKE) -C $(libavr_dir) build=atmega_u out_dir=$(realpath $(lib_out_dir)) CFLAGS="$(CFLAGS)"

# --- File Targets ---
$(builddirs) $(lib_out_dir) :
	mkdir -p $@

$(bdir)/%.hex : $(odir)/%.obj | $(bdir)
	$(OBJ2HEX) -R .eeprom -O ihex $< $@

.PRECIOUS : $(odir)/%.obj

$(odir)/%.obj : $(odir)/%.o $(ldirs_paths)
	$(CC) -o $@ $(CFLAGS) $(LDFLAGS) $(idirs) $(ldirs) $< $(lnames)

.PRECIOUS : $(odir)/%.o

$(odir)/%.o : %.c $(headers) | $(odir)
	$(CC) -o $@ -c $(CFLAGS) $(idirs) $<
