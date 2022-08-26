
libname := avrtask

# Toolchain config.
natural := naturaldocs
doc_dir := docs/natural
doc_conf_dir := docs/natural-config

ifdef libavr_attiny
device ?= attiny84
avrdude_device ?= t84
else
device ?= atmega328p
avrdude_device ?= m328p
endif

mcu ?= $(device)

warnings := -Wall -Wmissing-prototypes -Wstrict-prototypes

# IDEA: Let each project build its own custom copy of the library, with all the right
#       macro defs, etc.

ifdef libavr_attiny
# 1 MHz MCU clock (factory default: 8 MHz osc / prescale 8), 8 clock cycles per scheduler tick.
macro_defs := -DF_CPU=1000000L -DLIBAVR_ATTINY -DSCHED_CLOCK_PRESCALE_LOG=3
else
# 16 MHz MCU clock, 32 clock cycles per scheduler tick.
# Good for Arduino Uno.
macro_defs := -DF_CPU=16000000L -DSCHED_CLOCK_PRESCALE_LOG=5
# 1 MHz MCU clock (factory default: 8 MHz osc / prescale 8), 8 clock cycles per scheduler tick.
# Good for ATmega out-of-the-box.
#macro_defs := -DF_CPU=1000000L -DSCHED_CLOCK_PRESCALE_LOG=3
endif

# Re: -mcall-prologues
# "Functions prologues/epilogues are expanded as calls to appropriate subroutines.
#  Code size is smaller."
# https://gcc.gnu.org/onlinedocs/gcc/AVR-Options.html
ASFLAGS := $(macro_defs) -mmcu=$(mcu)
CFLAGS := $(macro_defs) -g $(warnings) -mcall-prologues -mmcu=$(mcu) -Os
testflags := $(macro_defs) -g $(warnings)
CC := avr-gcc
testcc := gcc

# Directory variables.
ifdef debug
odir := obj/debug
bdir := bin/debug
else
odir := obj/dist
bdir := bin/dist
endif

ifdef libavr_attiny
odir := $(odir)_attiny
bdir := $(bdir)_attiny
endif

todir := $(odir)/tests
tbdir := $(bdir)/tests
builddirs := $(odir) $(bdir) $(todir) $(tbdir)

# Filename variables.
ifdef libavr_attiny
common := task_sched tbouncer
else
common := task_sched tbouncer task_tlv std_tlv memmon spihelper i2chelper mcp4x mcp23018
endif
test_targets := task_sched_test

test_runs := $(addprefix run-, $(test_targets))

headers := *.h

common_o        := $(addprefix $(odir)/, $(addsuffix .o, $(common)))
libname_a       := $(addprefix $(bdir)/lib, $(addsuffix .a, $(libname)))
test_target_o   := $(addprefix $(todir)/, $(addsuffix .o, $(test_targets)))
test_target_exe := $(addprefix $(tbdir)/, $(test_targets))

# Phony targets.
.PHONY : all $(test_runs) $(libname) $(test_targets) clean doc tests run-tests

all : $(libname)

$(test_runs) : run-% : %
$(test_runs) : run-% : $(tbdir)/%
	$<

$(libname) : $(builddirs) $(libname_a)

$(test_targets) : % : $(builddirs) $(tbdir)/%

clean :
	rm -f $(odir)/*.o $(odir)/*.obj $(bdir)/*.hex $(todir)/*.o  $(tbdir)/*_test

# NOTE: Output dirs are automatically excluded from doc generation.
doc : $(headers)
	mkdir -p $(doc_dir) $(doc_conf_dir)
	$(natural) -i . -o HTML $(doc_dir) -p $(doc_conf_dir)

tests : $(test_targets)

run-tests : $(test_runs)

# File targets.
$(builddirs) :
	mkdir -p $@

$(libname_a) : $(common_o)
	$(AR) -rcs $@ $^

.PRECIOUS : $(odir)/%.o

$(odir)/%.o : %.c $(headers)
	$(CC) -c $(CFLAGS) -o $@ $<

$(odir)/%.o : %.S
	$(CC) -c $(ASFLAGS) -o $@ $<

.PRECIOUS : $(todir)/%.o

$(tbdir)/% : $(todir)/%.o
	$(testcc) $(testflags) -o $@ $< $(lnames)

$(todir)/%.o : %.c $(headers)
	$(testcc) -c $(testflags) -o $@ $<
