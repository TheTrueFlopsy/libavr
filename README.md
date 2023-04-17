# libavr

This is a firmware framework and utility library for AVR microcontrollers,
like the ATmega328 ones found in Arduino products. I wrote this code
for my own projects and my primary reason for publishing it is to provide
an example of my recent work to employers. If someone else has use for it
then that's great. It was designed to be used for firmware development on
a Linux (Debian) system and I have not tried it on any other platform.

The library contains both general facilities and a couple of modules that handle
specific ICs that I happened to be working with. There is some example firmware
and a few Python scripts that facilitate communication between the AVR firmware
and other computing devices.

Out-of-the-box, the library should be compatible with at least the following AVR
devices:
  * ATmega(4|8|16|32)8: Tested with an ATmega328P.
  * ATmega(16|32)U4: The library currently doesn't deal with the USB controller
  provided by these devices. Tested with an ATmega32U4.
  * ATtiny(2|4|8)4: Only a few of the library's modules are compatible with the
  ATtiny. Tested with an ATtiny84.

Please note that the code hasn't been systematically tested. Some use cases
haven't been tested at all. Approach the library with reasonable skepticism and
caution. If something doesn't work, it might not be your or your MCU's fault.
Reports on development platform and AVR device compatibility, as well as problems
encountered, are welcome.


## Building the Library
The library depends on [AVR Libc](https://www.nongnu.org/avr-libc/). Personally,
I compile firmware with `avr-gcc`, convert it to an
 [Intel hex](https://en.wikipedia.org/wiki/Intel_HEX)
file with `avr-objcopy` and upload it to the MCU with `avrdude` (
 [documented here](https://avrdudes.github.io/avrdude/current/avrdude_toc.html)
). These are all available in Debian packages (`avr-libc`, `gcc-avr`, `binutils-avr`
and `avrdude`, respectively).

Building the library on a Linux system should be a matter of installing the required
packages, cloning the libavr repository and running `make` in the library source
directory (i.e. the `avr/` subdirectory of the repository). You will probably need
at least AVR Libc, GCC and binutils with AVR support, GNU Make and some tool that can
upload built firmware to the MCU.

The library makefile supports building project-specific versions of libavr in project
directories. The example firmware makefiles demonstrate how this can be done. Basically,
one does a recursive make on the library source directory, specifying the target device
type, output directory and compiler options. Since the AVR device type and various other
parameters of the library and the system where the firmware will be running (e.g. the
CPU clock frequency) are compile-time constants, project-specific builds will often be
the best choice.


## Using the Library
Writing, building and uploading firmware is not hard once you know how to do it.
When developing AVR firmware, I rely heavily on MCU datasheets (available on the
 [web site](https://www.microchip.com/en-us/document-listing?docCategory=datasheets)
of Microchip Corporation) and the AVR Libc documentation. The source code and makefiles
for the example firmware should provide some useful hints. It's basically a matter of
writing C (and sometimes assembly) code that correctly uses library and hardware
interfaces to accomplish your goals, compiling and uploading the code (which a script
can do for you in seconds once you have a setup that works, getting to that point is
unfortunately beyond the scope of this readme) and then verifying that your system works
as intended. I've found
 [_Make: AVR Programming_](https://www.makershed.com/products/make-avr-programming-1ed-pdf)
by Elliot Williams to be a useful introduction. Apparently there are tools (debugWIRE,
JTAG and all that) that allow you to debug firmware as it runs in the MCU, but I must
admit that I have zero experience with those. I'm more of an "upload, observe behavior,
watch the serial output, think, try again" guy. Although my code usually works perfectly
the first time, of course.

Each library module has a corresponding C header file that defines the API of the module.
Keep in mind that some library modules need to be initialized, typically by calling a
function named `<module prefix>_init`, before use.

All module header files have [Natural Docs](https://www.naturaldocs.org/) doc comments
that describe the module's API. These were written for the legacy version (1.5x) of Natural
Docs (available in the Debian package `naturaldocs` as of 2023-04-15). With the legacy
version installed, HTML documentation can be generated by running the command `make doc`
in the library source directory.

### The Task Scheduler
When you develop firmware that uses libavr, I recommend that you think about the
functional requirements in terms of tasks running in the library's task scheduler.
Each task implements an activity that the firmware needs to perform, parallel to
and often independent of other tasks. Tasks may execute after a set delay (which might
be periodic) or be triggered by external or internal events. See the module documentation
below and in `task_sched.h` for more information about how the scheduler works.

**NOTE:** Some libavr modules rely on the task scheduler and will not work properly
if the scheduler isn't running.

### INM Communication
The library modules use the INM (Inter-Node Messaging) protocol to communicate
with other computing devices. This protocol is my own invention, basically link-agnostic
 [TLV](https://en.wikipedia.org/wiki/Type%E2%80%93length%E2%80%93value)
messages with simple datagram addressing and sequencing. The current version of libavr
only supports an USART data link.

Once you have uploaded your firmware, you can use the Python modules in the repository
to communicate with the firmware via INM (a USB-to-serial adapter or other means of
connection to the MCU's USART is required). These modules also have Natural Docs
comments, corresponding HTML documentation can be generated by running `make doc`
in the Python source directory (i.e. `python/`).


## Library Modules

### avr/watchdog.h - Watchdog Timer Control
This module provides facilities that control the MCU's watchdog timer. There is
a function that performs a software reset of the MCU via the watchdog, as well as
macros that disable the watchdog immediately after reset (the watchdog timer remains
enabled after a watchdog reset).

**NOTE:** This module is compatible with ATtiny devices.

**NOTE:** This module does not depend on the task scheduler.

### avr/task_sched.h - Task Scheduler
Task scheduler with run-time task management and delayed execution. Some of the other
library modules depend on the task scheduler.

The scheduler has a main loop that repeatedly performs _scheduler iterations_. During
each iteration, the scheduler invokes a handler procedure for each scheduled task.
A scheduled task is represented by a `sched_task` struct that contains a pointer to
the handler procedure, a delay field and a _task control and status byte_ (TCSB). The TCSB
consists of a sleep bit, a task category number and a task instance number. Task
categories are significant because they are used to asynchronously notify sleeping tasks.
Task instance numbers provide a way to distinguish tasks that belong to the same category.

For many applications, it will be sufficient to configure and schedule a number of tasks
during firmware initialization and then let those tasks run until the MCU is reset.
However, the scheduler also fully supports adding and removing tasks while the scheduler
is running. A task can even remove itself.

**CAUTION:** Performing scheduler operations (like adding, removing or synchronously
invoking tasks) in an ISR is NOT supported.

Sleeping tasks are not executed by the scheduler. A task is put to sleep by setting
the sleep bit in the TCSB. A sleeping task can be awakened by manually clearing the
sleep bit, via a scheduler operation or by notifying its task category.

Task execution can be delayed by setting the delay field in the `sched_task` struct.
The library's representation of time allows delays up to at least 16 seconds, in
steps of a few microseconds (the error in the scheduler's approximation of the
specified execution delay will usually be far larger than that, though). The
scheduler module always zeroes the delay field before invoking the task handler
procedure, so to achieve periodic execution, the handler procedure should reset
the delay to the desired period each time it is invoked.

The fact that the execution delay is always zeroed before task handler invocation
is problematic for tasks that need to respond to asynchronous events and simultaneously
keep track of an elapsing delay/timeout. The recommended way to deal with such cases is
to store the value of the time counter `sched_ticks` when the delay is set, then check
whether the difference between the current value of `sched_ticks` and the stored value
is no less than the intended delay (the library functions `sched_time_sub` and
`sched_time_gte` can be used to do this) each time the task handler is invoked.

**NOTE:** This module is compatible with ATtiny devices.

### avr/task_tlv.h - Asynchronous TLV/INM Communication

**NOTE:** This module depends on the task scheduler and will not work if the
scheduler isn't running.

### avr/std_tlv.h - Standard Types and Helpers for TLV/INM

**NOTE:** This module depends on the task scheduler (via `task_tlv.h`) and will
not work if the scheduler isn't running.

### avr/memmon.h - Memory Monitors
Memory monitoring via TLV messages.

**NOTE:** This module depends on the task scheduler and will not work if the
scheduler isn't running.

### avr/tbouncer.h - Input Debouncing
GPIO input debouncing with task notification.

**NOTE:** This module is compatible with ATtiny devices.

**NOTE:** This module depends on the task scheduler and will not work if the
scheduler isn't running.

### avr/i2chelper.h - Asynchronous I2C/TWI Communication
Asynchronous, interrupt-driven I2C (aka TWI) helper module.

**NOTE:** This module can notify tasks when an I2C operation finishes, but does
not need the task scheduler to function.

### avr/spihelper.h - SPI Helpers
Helper functions and macros for SPI communication.

**NOTE:** This module does not depend on the task scheduler.

### avr/mcp23018.h - MCP23018 I/O Expander Interface
Helper module for controlling an MCP23018 I/O expander via I2C.

**NOTE:** This module can notify tasks when an I2C operation finishes, but does
not need the task scheduler to function.

### avr/mcp4x.h - MCP4x Potentiometer Interface
Helper module for controlling an MCP4x potentiometer via SPI.

**NOTE:** This module does not depend on the task scheduler.


## Example Firmware

### examples_atmega/sched_blinker.c - Simple LED Blinker
A simple LED blinker.

### examples_atmega/watchdog_test.c - Watchdog Timer Demo
Demonstration of the watchdog timer control module.

### examples_atmega/inm_node_demo.c - Sensors and Comms
Sensor sampling and notifications. WIP for a home security system.

### examples_attiny/sched_test.c - Tiny Scheduler Test
Minimal test of the task scheduler.

### examples_attiny/stepper_test.c - Stepper Motor Demo
* `stepper_test`: Demonstration of stepper motor control via PWM generator.

### examples_atmega_u/sched_blinker.c - Simple LED Blinker
Simple LED blinker for the ATmegaU.

### examples_atmega_u/lcd_driver.c - 7-seg LCD Demo
Driving a seven-segment LCD via PWM generator and a MCP23018 I/O expander.


## Python Modules and Scripts

These Python modules use [pySerial](https://pyserial.readthedocs.io/en/latest/pyserial.html)
for serial port communication. No other third-party modules are used. Python 3.6 or later is
required.

### python/inm/inm.py - General API for INM
INM communication API. Supports serial port and UDP/IP links.

### python/inm/helper.py - Convenience API for INM
Convenience wrapper for an INM communication channel.

### python/inm_router.py - INM Router Script
INM message router script. May be installed as a Linux service.

### python/inm_memmon.py - Memory Monitor Visualization
Text-based visualization script for libavr memory monitors.
