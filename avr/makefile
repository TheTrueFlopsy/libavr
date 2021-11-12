
libname := avrtask

# Toolchain config.
natural := naturaldocs
doc_dir := docs/natural
doc_conf_dir := docs/natural-config

device := atmega328p
mcu := atmega328p
avrdude_device := m328p

warnings := -Wall -Wmissing-prototypes -Wstrict-prototypes
# Re: -mcall-prologues
# "Functions prologues/epilogues are expanded as calls to appropriate subroutines.
#  Code size is smaller."
# https://gcc.gnu.org/onlinedocs/gcc/AVR-Options.html
macro_defs := -DF_CPU=16000000L
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

todir := $(odir)/tests
tbdir := $(bdir)/tests
builddirs := $(odir) $(bdir) $(todir) $(tbdir)

# Filename variables.
common       := task_sched tbouncer task_tlv std_tlv memmon spihelper i2chelper mcp4x mcp23018
test_targets := task_sched_test

test_runs := $(addprefix run-, $(test_targets))

headers := *.h

common_o        := $(addprefix $(odir)/, $(addsuffix .o, $(common)))
libname_a       := $(addprefix $(bdir)/lib, $(addsuffix .a, $(libname)))
test_target_o   := $(addprefix $(todir)/, $(addsuffix .o, $(test_targets)))
test_target_exe := $(addprefix $(tbdir)/, $(test_targets))

# Phony targets.
.PHONY : all $(programs) $(test_runs) $(libname) $(test_targets) clean doc tests run-tests

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
