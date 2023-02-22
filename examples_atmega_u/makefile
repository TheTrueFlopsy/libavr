
### SOURCE DEPENDENCIES: libavr-dev/avr
### BINARY DEPENDENCIES: libavr-dev/avr

# --- Toolchain Config ---
CC := avr-gcc
testcc := gcc
OBJ2HEX := avr-objcopy

natural := naturaldocs

# Good settings for Arduino Leonardo
# 16 MHz MCU clock, 64 clock cycles per scheduler tick
f_cpu := 16000000
f_cpu_prescale := 6
mcu := atmega32u4

avrdude := avrdude

avrdude_port ?= /dev/ttyACM0
#avrdude_port ?= /dev/ttyUSB0
avrdude_device := m32u4
avrdude_prog := avr109
avrdude_baud := 115200

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
targets  := sched_blinker lcd_driver
programs := $(addprefix prog-, $(targets))

headers := $(libavr_dir)/*.h

target_o   := $(addprefix $(odir)/, $(addsuffix .o, $(targets)))
target_obj := $(addprefix $(odir)/, $(addsuffix .obj, $(targets)))
target_hex := $(addprefix $(bdir)/, $(addsuffix .hex, $(targets)))

# --- Command Option Variables ---
warnings := -Wall -Wmissing-prototypes -Wstrict-prototypes
macro_defs := -DF_CPU=$(f_cpu)L -DLIBAVR_ATMEGA_U -DSCHED_CLOCK_PRESCALE_LOG=$(f_cpu_prescale)

idirs := -I$(libavr_dir)
ldirs := $(addprefix -L, $(ldirs_paths))
lnames := -lavrtask -lm

CFLAGS := -std=c11 $(macro_defs) -g $(warnings) -mcall-prologues -mmcu=$(mcu) -Os -flto
LDFLAGS := -fuse-linker-plugin -Wl,-gc-sections -Wl,-relax

# --- Phony Targets ---
.PHONY : all $(programs) $(targets) clean clean-libavr libavr

all : $(targets)

$(programs) : prog-% : %
$(programs) : prog-% : $(bdir)/%.hex
	$(avrdude) -v -p $(avrdude_device) -c $(avrdude_prog) -P $(avrdude_port) -b $(avrdude_baud) -D -U flash:w:$<

# Bus Pirate settings:
#$(avrdude) -v -p $(avrdude_device) -c buspirate -x spifreq=1 -P $(avrdude_port) -U flash:w:$<
# Pololu programmer settings:
#$(avrdude) -v -p $(avrdude_device) -c avrisp2 -B 100kHz -P $(avrdude_port) -U flash:w:$<
# CAUTION: The -D option disables automatic chip erase before programming.
#          Not including that option might have unpleasant consequences.
# Arduino settings:
#$(avrdude) -v -p $(avrdude_device) -c arduino -P $(avrdude_port) -b $(avrdude_baud) -D -U flash:w:$<

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
