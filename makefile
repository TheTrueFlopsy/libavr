
.PHONY : all clean doc doc-rebuild tests run-tests

all :
	$(MAKE) -C avr all
	$(MAKE) -C examples_atmega all
	$(MAKE) -C examples_atmega_u all
	$(MAKE) -C examples_attiny all

clean :
	$(MAKE) -C avr clean
	$(MAKE) -C examples_atmega clean
	$(MAKE) -C examples_atmega_u clean
	$(MAKE) -C examples_attiny clean
	$(MAKE) -C python clean

doc :
	$(MAKE) -C avr doc
	$(MAKE) -C python doc

doc-rebuild :
	$(MAKE) -C avr doc-rebuild
	$(MAKE) -C python doc-rebuild

tests :
	$(MAKE) -C avr tests

run-tests :
	$(MAKE) -C avr run-tests
