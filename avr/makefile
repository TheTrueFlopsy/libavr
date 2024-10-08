
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

# NOTE: If this makefile is being processed by a recursive invocation of make,
#       to produce a project-specific build of libavr, then the build configuration
#       variables below are still defined, but typically not used. The project makefile
#       that invokes make recursively should specify a project-specific value for the
#       CFLAGS variable, overriding the definition where these variables are used.
ifdef libavr_attiny  # Settings for ATtiny84 out-of-the-box
mcu := attiny84
# 1 MHz MCU clock (factory default: 8 MHz osc. / MCU prescale 8), 8 clock cycles per scheduler tick
f_cpu := 1000000
f_cpu_prescale := 3
else ifdef libavr_atmega_u  # Settings for Arduino Leonardo
mcu := atmega32u4
# 16 MHz MCU clock, 64 clock cycles per scheduler tick
f_cpu := 16000000
f_cpu_prescale := 6
else  # Settings for Arduino Uno and for ATmega328 out-of-the-box
mcu := atmega328p
# 16 MHz MCU clock, 32 clock cycles per scheduler tick
f_cpu := 16000000
f_cpu_prescale := 5
# 1 MHz MCU clock (factory default: 8 MHz osc. / MCU prescale 8), 8 clock cycles per scheduler tick
#f_cpu := 1000000
#f_cpu_prescale := 3
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
common := watchdog task_sched tbouncer
else
common := watchdog task_sched tbouncer task_tlv std_tlv memmon spihelper i2chelper mcp4x nrf24x mcp23018
endif

test_common    := task_sched tbouncer
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
# NOTE: -flto enables link-time optimization (LTO), which among other things lets
#       the compiler exclude unused library functions from the linked binary. Note
#       that this leads to LTO-specific data being added to object files.
CFLAGS := -std=c11 $(macro_defs) -g $(warnings) -mcall-prologues -mmcu=$(mcu) -Os -flto
override CFLAGS += -ffat-lto-objects
#ASFLAGS := $(macro_defs) -mmcu=$(mcu)

testflags := $(macro_defs) -DLIBAVR_TEST_BUILD -g $(warnings)

natural_flags := --highlight all --style Default MyStyle

# --- Phony Targets ---
.PHONY : all $(test_runs) $(cytest_runs) $(libname) $(test_targets) \
         clean doc doc-rebuild tests cytests run-tests cyrun-tests

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
	$(RM) $(odir)/*.o $(odir)/*.obj $(bdir)/*.hex $(todir)/*.o  $(tbdir)/*_test

doc-rebuild : natural_flags += --rebuild
doc-rebuild : doc

# NOTE: Output dirs are automatically excluded from doc generation.
doc : $(headers) | $(docdirs)
	$(natural) -i . -o HTML $(doc_dir) -p $(doc_conf_dir) $(natural_flags)

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
