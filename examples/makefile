
### SOURCE DEPENDENCIES: libavr/avr
### BINARY DEPENDENCIES: libavr/avr

# --- Toolchain Config ---
CC := avr-gcc
testcc := gcc
OBJ2HEX := avr-objcopy

# Good settings for Arduino Uno
# 16 MHz MCU clock, 32 clock cycles per scheduler tick
f_cpu := 16000000
f_cpu_prescale := 5
# Good settings for ATmega328 out-of-the-box
# 1 MHz MCU clock (factory default: 8 MHz osc. / MCU prescale 8), 8 clock cycles per scheduler tick
#f_cpu := 1000000
#f_cpu_prescale := 3
mcu := atmega328p

avrdude := avrdude

avrdude_port ?= /dev/ttyUSB0
#avrdude_port ?= /dev/ttyACM0
avrdude_device := m328p
avrdude_prog := arduino
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

idirs := -I$(libavr_dir)
lnames := -lavrtask -lm

ifeq ($(custom_libavr),yes)
ldirs := -L$(lib_out_dir)/$(rel_bdir)
else
ldirs := -L$(libavr_dir)/$(rel_bdir)
endif

# --- Filename Variables ---
targets := sched_blinker inm_node_demo
programs := $(addprefix prog-, $(targets))

headers := *.h

target_o   := $(addprefix $(odir)/, $(addsuffix .o, $(targets)))
target_obj := $(addprefix $(odir)/, $(addsuffix .obj, $(targets)))
target_hex := $(addprefix $(bdir)/, $(addsuffix .hex, $(targets)))

# --- Command Option Variables ---
warnings := -Wall -Wmissing-prototypes -Wstrict-prototypes
macro_defs := -DF_CPU=$(f_cpu)L -DSCHED_CLOCK_PRESCALE_LOG=$(f_cpu_prescale)

CFLAGS := -std=c11 $(macro_defs) -g $(warnings) -mcall-prologues -mmcu=$(mcu) -Os -flto
LDFLAGS := -fuse-linker-plugin -Wl,--gc-sections -Wl,--relax

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
	$(MAKE) -C $(libavr_dir) build=atmega out_dir=$(realpath $(lib_out_dir)) clean

libavr : | $(lib_out_dir)
	$(MAKE) -C $(libavr_dir) build=atmega out_dir=$(realpath $(lib_out_dir)) CFLAGS="$(CFLAGS)"

# --- File Targets ---
$(builddirs) $(lib_out_dir) :
	mkdir -p $@

$(bdir)/%.hex : $(odir)/%.obj | $(bdir)
	$(OBJ2HEX) -R .eeprom -O ihex $< $@

.PRECIOUS : $(odir)/%.obj

$(odir)/%.obj : $(odir)/%.o
	$(CC) -o $@ $(CFLAGS) $(LDFLAGS) $(idirs) $(ldirs) $< $(lnames)

.PRECIOUS : $(odir)/%.o

$(odir)/%.o : %.c $(headers) | $(odir)
	$(CC) -o $@ -c $(CFLAGS) $(idirs) $<
