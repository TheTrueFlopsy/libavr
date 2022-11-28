
### SOURCE DEPENDENCIES: libavr/avr
### BINARY DEPENDENCIES: libavr/avr

# Toolchain config.
CC := avr-gcc
testcc := gcc
OBJ2HEX := avr-objcopy

device := atmega328p
mcu := atmega328p
avrdude_device := m328p

port ?= /dev/ttyUSB0  # Good for Arduino and Bus Pirate.
#port ?= /dev/ttyACM0  # Good for Pololu programmer.

avrdude := avrdude

# Directory variables.
ifdef debug
odir := obj/debug
bdir := bin/debug
else
odir := obj/dist
bdir := bin/dist
endif

builddirs := $(odir) $(bdir)

idirs := -I../avr
ldirs := -L../avr/$(bdir)
lnames := -lavrtask -lm

# Filename variables.
targets := sched_blinker inm_node_demo

programs := $(addprefix prog-, $(targets))

headers    := *.h
target_o   := $(addprefix $(odir)/, $(addsuffix .o, $(targets)))
target_obj := $(addprefix $(odir)/, $(addsuffix .obj, $(targets)))
target_hex := $(addprefix $(bdir)/, $(addsuffix .hex, $(targets)))

# Command option variables.
warnings := -Wall -Wmissing-prototypes -Wstrict-prototypes

# 16 MHz MCU clock, 32 clock cycles per scheduler tick.
macro_defs := -DF_CPU=16000000L -DSCHED_CLOCK_PRESCALE_LOG=5  # Good for Arduino Uno.
# 1 MHz MCU clock (factory default: 8 MHz osc / prescale 8), 8 clock cycles per scheduler tick.
#macro_defs := -DF_CPU=1000000L -DSCHED_CLOCK_PRESCALE_LOG=3  # Good for ATmega out-of-the-box.

# Re: -mcall-prologues
# "Functions prologues/epilogues are expanded as calls to appropriate subroutines.
#  Code size is smaller."
# https://gcc.gnu.org/onlinedocs/gcc/AVR-Options.html
CFLAGS := $(macro_defs) -g $(warnings) -mcall-prologues -mmcu=$(mcu) -Os -flto $(idirs)
# Re: -Wl,-relax
# "On some platforms the --relax option performs target specific,
#  global optimizations that become possible when the linker resolves
#  addressing in the program"
# man ld
LDFLAGS := -fuse-linker-plugin -Wl,--gc-sections -Wl,--relax

# Phony targets.
.PHONY : all clean $(programs) $(targets)

all : $(targets)

clean :
	rm -f $(odir)/*.o $(odir)/*.obj $(bdir)/*.hex

# CAUTION: The -D option disables automatic chip erase before programming.
#          Not including that option might have unpleasant consequences.
$(programs) : prog-% : %
$(programs) : prog-% : $(bdir)/%.hex
	$(avrdude) -v -p $(avrdude_device) -c arduino -P $(port) -b 115200 -D -U flash:w:$<

# Bus Pirate settings:
#$(avrdude) -v -p $(avrdude_device) -c buspirate -x spifreq=1 -P $(port) -U flash:w:$<
# Pololu programmer settings:
#$(avrdude) -v -p $(avrdude_device) -c avrisp2 -B 100kHz -P $(port) -U flash:w:$<
# Arduino settings:
#$(avrdude) -v -p $(avrdude_device) -c arduino -P $(port) -b 115200 -D -U flash:w:$<

# TODO: Add a command that programs the EEPROM (with a hex file that only
#       contains the .eeprom section).

$(targets) : % : $(builddirs) $(bdir)/%.hex

# File targets.
$(builddirs) :
	mkdir -p $@

$(bdir)/%.hex : $(odir)/%.obj
	$(OBJ2HEX) -R .eeprom -O ihex $< $@

.PRECIOUS : $(odir)/%.obj  # Don't touch my precious object files make you beast.

$(odir)/%.obj : $(odir)/%.o
	$(CC) -o $@ $(CFLAGS) $(LDFLAGS) $(ldirs) $< $(lnames)

.PRECIOUS : $(odir)/%.o

$(odir)/%.o : %.c  $(headers)
	$(CC) -o $@ -c $(CFLAGS) $<
