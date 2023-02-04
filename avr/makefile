
libname := avrtask

# --- Toolchain Config ---
CC := avr-gcc
testcc := gcc
# NOTE: Apparently 'gcc-ar' is better than plain 'ar' for creating LTO-enabled
#       static libraries:
# https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html
AR := avr-gcc-ar

natural := naturaldocs

ifdef build
ifeq ($(build),attiny)
libavr_attiny := yes
else ifeq ($(build),atmega_u)
libavr_atmega_u := yes
else ifneq ($(build),atmega)
$(error Unrecognized build type: $(build))
endif
endif

ifdef libavr_attiny  # Good settings for ATtiny84 out-of-the-box
# 1 MHz MCU clock (factory default: 8 MHz osc. / MCU prescale 8), 8 clock cycles per scheduler tick
f_cpu := 1000000
f_cpu_prescale := 3
mcu := attiny84
else ifdef libavr_atmega_u  # Good settings for Arduino Leonardo
# 16 MHz MCU clock, 64 clock cycles per scheduler tick
f_cpu := 16000000
f_cpu_prescale := 6
mcu := atmega32u4
else  # Good settings for Arduino Uno / ATmega328 out-of-the-box
# 16 MHz MCU clock, 32 clock cycles per scheduler tick
f_cpu := 16000000
f_cpu_prescale := 5
# 1 MHz MCU clock (factory default: 8 MHz osc. / MCU prescale 8), 8 clock cycles per scheduler tick
#f_cpu := 1000000
#f_cpu_prescale := 3
mcu := atmega328p
endif

# --- Directory Variables ---
out_dir := .

ifdef debug
odir := $(out_dir)/obj/debug
bdir := $(out_dir)/bin/debug
else
odir := $(out_dir)/obj/dist
bdir := $(out_dir)/bin/dist
endif

ifdef libavr_attiny
odir := $(odir)_attiny
bdir := $(bdir)_attiny
else ifdef libavr_atmega_u
odir := $(odir)_atmega_u
bdir := $(bdir)_atmega_u
endif

todir := $(odir)/tests
tbdir := $(bdir)/tests
builddirs := $(odir) $(bdir) $(todir) $(tbdir)

doc_dir := docs/natural
doc_conf_dir := docs/natural-config
docdirs := $(doc_dir) $(doc_conf_dir)

# --- Filename Variables ---
ifdef libavr_attiny
common := task_sched tbouncer
else
common := task_sched tbouncer task_tlv std_tlv memmon spihelper i2chelper mcp4x mcp23018
endif

test_common    := tbouncer
test_targets   := task_sched_test tbouncer_test
cytest_targets := tbouncer_test

test_runs   := $(addprefix run-, $(test_targets))
cytest_runs := $(addprefix cyrun-, $(cytest_targets))

headers := *.h

common_o        := $(addprefix $(odir)/, $(addsuffix .o, $(common)))
libname_a       := $(addprefix $(bdir)/lib, $(addsuffix .a, $(libname)))
test_common_o   := $(addprefix $(todir)/, $(addsuffix .o, $(test_common)))
test_target_o   := $(addprefix $(todir)/, $(addsuffix .o, $(test_targets)))
test_target_exe := $(addprefix $(tbdir)/, $(test_targets))

# --- Command Option Variables ---
warnings := -Wall -Wmissing-prototypes -Wstrict-prototypes
macro_defs := -DF_CPU=$(f_cpu) -DSCHED_CLOCK_PRESCALE_LOG=$(f_cpu_prescale)

ifdef libavr_attiny
macro_defs += -DLIBAVR_ATTINY
else ifdef libavr_atmega_u
macro_defs += -DLIBAVR_ATMEGA_U
endif

# NOTE: about -mcall-prologues
# "Functions prologues/epilogues are expanded as calls to appropriate subroutines.
#  Code size is smaller."
# https://gcc.gnu.org/onlinedocs/gcc/AVR-Options.html
# NOTE: -flto enables Link-Time Optimization, which among other things lets the compiler
#       exclude unused library functions from the linked binary. Note that this leads to
#       specialized LTO data being added to object files.
CFLAGS := -std=c11 $(macro_defs) -g $(warnings) -mcall-prologues -mmcu=$(mcu) -Os -flto
override CFLAGS += -ffat-lto-objects
#ASFLAGS := $(macro_defs) -mmcu=$(mcu)

testflags := $(macro_defs) -DLIBAVR_TEST_BUILD -g $(warnings)

# --- Phony Targets ---
.PHONY : all $(test_runs) $(cytest_runs) $(libname) $(test_targets) \
         clean doc tests cytests run-tests cyrun-tests

all : $(libname)

$(test_runs) : run-% : %
$(test_runs) : run-% : $(tbdir)/%
	$<

$(cytest_runs) : cytests
$(cytest_runs) : cyrun-% : cython/%.py
	cd cython && ./$(notdir $<)

$(libname) : $(libname_a)

$(test_targets) : % : $(tbdir)/% | $(builddirs)

clean :
	rm -f $(odir)/*.o $(odir)/*.obj $(bdir)/*.hex $(todir)/*.o  $(tbdir)/*_test

# NOTE: Output dirs are automatically excluded from doc generation.
doc : $(headers) | $(docdirs)
	$(natural) -i . -o HTML $(doc_dir) -p $(doc_conf_dir)

tests : $(test_targets) cytests

cytests :
	cd cython && ./setup.py build_ext --inplace

run-tests : $(test_runs) cyrun-tests

cyrun-tests : $(cytest_runs)

# --- File Targets ---
$(builddirs) $(docdirs) :
	mkdir -p $@

$(libname_a) : $(common_o) | $(bdir)
	$(AR) -rcs $@ $^

.PRECIOUS : $(odir)/%.o  # Don't touch my precious object files make you beast.

$(odir)/%.o : %.c $(headers) | $(odir)
	$(CC) -c $(CFLAGS) -o $@ $<

.PRECIOUS : $(todir)/%.o

$(tbdir)/% : $(todir)/%.o $(test_common_o) | $(tbdir)
	$(testcc) $(testflags) -o $@ $^ $(lnames)

$(todir)/%.o : %.c $(headers) | $(todir)
	$(testcc) -c $(testflags) -o $@ $<
