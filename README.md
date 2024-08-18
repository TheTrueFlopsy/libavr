# libavr

This is a firmware framework and utility library for AVR microcontrollers,
like the ATmega328 ones found in Arduino products. I wrote this code
for my own projects and my primary reason for publishing it is to provide
an example of my recent work to employers. If someone else has use for it
then that's great. It was designed to be used for firmware development on
a Linux (Debian) system and I have not tried it on any other platform.

The library contains both general facilities and modules that handle
specific ICs that I happened to be working with. There is some example firmware
and Python scripts that facilitate communication between the AVR firmware
and other computing devices.

The philosophy of the library is to not attempt to replace or add layers
on top of what already works well with just AVR Libc and the hardware.
For example, I don't perceive doing GPIO directly on registers as inconvenient
(it would be even better if AVR Libc would keep register, bit and ISR names
fully consistent in the I/O headers).

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
The library depends on [AVR Libc](https://github.com/avrdudes/avr-libc). Personally,
I compile firmware with `avr-gcc`, convert it to an
 [Intel hex](https://en.wikipedia.org/wiki/Intel_HEX)
file with `avr-objcopy` and upload it to the MCU with `avrdude`, which is may be found
 [here](https://github.com/avrdudes/avrdude).
These are all available in Debian packages (`avr-libc`, `gcc-avr`, `binutils-avr`
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

The library makes use of GCC's support for
 [link-time optimization](https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html#index-flto)
(LTO) to enable optimization of the built firmware as a whole (including library code)
and to exclude unused library functions from the uploadable firmware. The use of LTO
requires the presence of additional data (GIMPLE bytecode) in the linked object files.
See the example firmware makefiles for details on how to set up LTO builds in a project
(it amounts to adding a couple of compiler options).

There are a few test programs included in the repository. The purpose of these is to
enable testing of library code directly on the build system, instead of in an MCU.
The test programs can be built with `make tests` and then executed with `make run-tests`.
In the current version, the test programs only exercise the debouncer module (`tbouncer.h`)
and some helper macros and functions for the task scheduler (`task_sched.h`). Some tests
require [Cython](https://cython.org/) and [setuptools](https://setuptools.pypa.io/)
(available in Debian packages `cython3` and `python3-setuptools`).


## Using the Library
Writing, building and uploading firmware is not hard once you know how to do it.
When developing AVR firmware, I rely heavily on MCU datasheets (available on the
 [web site](https://www.microchip.com/en-us/document-listing)
of Microchip Inc.) and the AVR Libc documentation. The source code and makefiles
for the example firmware should provide some useful hints. It's basically a matter of
writing C (and sometimes assembly) code that correctly uses library and hardware
interfaces to accomplish your goals, compiling and uploading the code (which a script
can do for you in seconds once you have a setup that works, getting to that point is
unfortunately beyond the scope of this readme) and then verifying that your system works
as intended. I've found
 [_AVR Programming_](https://www.makershed.com/products/make-avr-programming-1ed-pdf)
by Elliot Williams to be a useful introduction. Apparently there are tools (debugWIRE,
JTAG and all that) that allow you to debug firmware as it runs in the MCU, but I must
admit that I have zero experience with those. I'm more of an "upload, observe behavior,
watch the serial output, think, try again" guy. Although my code usually works perfectly
the first time, of course.

Each library module has a corresponding C header file that defines the API of the module.
Keep in mind that some library modules need to be initialized, typically by calling a
function named `{module_prefix}_init`, before use.

All module header files have [Natural Docs](https://www.naturaldocs.org/) doc comments
that describe the module's API. These were written for the legacy version (1.5x) of Natural
Docs (available in the Debian package `naturaldocs` as of 2023-05-20). With the legacy
version installed, HTML documentation can be generated by running the command `make doc`
in the library source directory. This documentation is also available online:
  * [C API documentation](https://thetrueflopsy.github.io/libavr/avr/index/Files.html)
  * [Python API documentation](https://thetrueflopsy.github.io/libavr/python/index/Files.html)

### Task-Oriented Design
When you develop firmware that uses libavr, I recommend that you think about the
functional requirements in terms of tasks running in the library's task scheduler.
Each task should implement an activity that the firmware needs to perform, parallel to
and often independent of other tasks. Tasks may execute after a set delay (which might
be periodic) or be triggered by external or internal events. Asynchronous notification
of tasks by ISRs and other tasks is supported, as well as synchronous invocation
of tasks by other tasks. See the module documentation below and in `task_sched.h`
for more information about how the scheduler works.

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

### avr/bitops.h – Bit Manipulation Macros
This header-only module defines a number of bit manipulation macros. In particular,
there are macros for bit masks and extraction/insertion of bit fields in integers.

**NOTE:** This module is compatible with ATtiny devices.

**NOTE:** This module does not depend on the task scheduler.

### avr/watchdog.h – Watchdog Timer Control
This module provides facilities that control the MCU's watchdog timer. There is
a function that performs a software reset of the MCU via the watchdog, as well as
macros that disable the watchdog immediately after reset (the watchdog timer remains
enabled after a watchdog reset).

**NOTE:** This module is compatible with ATtiny devices.

**NOTE:** This module does not depend on the task scheduler.

### avr/task_sched.h – Task Scheduler
Task scheduler with run-time task management and delayed execution. Some of the other
library modules depend on the task scheduler.

The scheduler has a main loop that repeatedly performs _scheduler iterations_. During
each iteration, the scheduler invokes a handler procedure for each scheduled task
(that isn't sleeping or delayed). A scheduled task is represented by a `sched_task`
struct that contains a pointer to the handler procedure, a delay field and
a _task control and status byte_ (TCSB). The TCSB consists of a sleep bit, a task
category number and a task instance number. Task categories are significant because they
are used to asynchronously notify sleeping tasks. Task instance numbers provide a way
to distinguish tasks that belong to the same category.

For many applications, it will be sufficient to configure and schedule a number of tasks
during firmware initialization and then let those tasks run until the MCU is reset.
However, the scheduler also fully supports adding and removing tasks while the scheduler
is running. A task can even remove itself.

**CAUTION:** Performing scheduler operations (like adding, removing or synchronously
invoking tasks) in an ISR is NOT supported.

**NOTE:** The multitasking provided by the scheduler is cooperative, relying on each
invoked task handler returning without excessive delay. The hard limit is that the
duration of a scheduler iteration must be representable by the `sched_time` data type
(i.e. generally not longer than 16 seconds), but for timely execution of tasks
in response to notifications and timeouts, task handlers will typically have to return
a lot quicker than that. If a handler needs to run for more than a few milliseconds at
a time, it might be a good idea to reconsider your solution.

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
to store the value of the global time counter `sched_ticks` when the delay is set, then
check whether the difference between the current value of `sched_ticks` and the stored
value is no less than the intended delay (the library functions `sched_time_sub` and
`sched_time_gte` can be used to do this) each time the task handler is invoked.

**NOTE:** This module is compatible with ATtiny devices.

### avr/task_tlv.h – Asynchronous TLV/INM Communication
The TLV/INM module provides a way for the firmware to exchange messages via the USART.
Communication via this module is asynchronous, meaning that firmware execution is not
held up waiting for incoming messages or transmitting outgoing messages. Incoming
messages received by the module and awaiting processing by application code can be
detected by polling or task notification. Outgoing messages can be submitted to the
module, which will then transmit them without further intervention by the code that
submitted them.

Messages exchanged via this module have a type-length-value (TLV) format. An 8-bit
message type identifier is followed by an 8-bit payload length field and zero or more
arbitrary payload bytes.

The module can operate in different modes. In INM mode, a header containing source and
destination node addresses, plus a sequential message identifier, is prepended to each
message. In plain TLV (i.e. non-INM) mode, messages are exchanged over a point-to-point
link, without any addressing or sequencing. In stream mode, the module will report
reception of and perform transmission of partial messages (instead of waiting for
a complete message to be received or submitted for transmission).

**NOTE:** This module depends on the task scheduler and will not work if the
scheduler isn't running.

### avr/std_tlv.h – Standard Types and Helpers for TLV/INM
This module defines standard message types for TLV/INM communication, as well as
convenience macros and functions to simplify implementation of firmware support
for these standard message types. A key concept is to design the remote interfaces
of firmware in terms of logical registers on which a standard set of generic
operations (read, update, toggle, etc.) may be performed. Some logical registers
are themselves standardized and intended to form the basis of a general,
firmware-agnostic remote interface. For example, it should be possible to query
the firmware ID and version of any system that supports INM in a generic way.

**CAUTION:** The current logical register specification has bothersome and
unnecessary limitations (e.g. all registers being one byte large) and is therefore
being considered for compatibility-breaking replacement in a future release of libavr.

**NOTE:** This module depends on the task scheduler (via `task_tlv.h`) and will
not work if the scheduler isn't running.

### avr/memmon.h – Memory Monitors
Memory monitors let the firmware set up periodic TLV/INM transmission of the
content of specified locations in the MCU's data address space. They are primarily
intended as a debugging facility.

**NOTE:** This module depends on the task scheduler and will not work if the
scheduler isn't running.

### avr/tbouncer.h – Input Debouncing
This module provides software debouncing of GPIO inputs. The debouncing algorithm
can filter out both switch bounce (i.e. a legitimate logic state transition being
accompanied by additional rapid, spurious transitions) and spikes (i.e. isolated,
brief, spurious logic state changes) in the input signals.

The interface of the debouncing module is entirely event-based. At module
initialization, the firmware selects GPIO pins to perform debouncing on and tasks
to notify or invoke in response to changes in the debounced logic states. When
the selected input handling tasks execute, they can poll global variables exposed
by the module to determine which pin states have changed and the direction of the
changes.

**NOTE:** This module is compatible with ATtiny devices.

**NOTE:** This module depends on the task scheduler and will not work if the
scheduler isn't running.

### avr/i2chelper.h – Asynchronous I<sup>2</sup>C/TWI Communication
Asynchronous, interrupt-driven I<sup>2</sup>C (aka TWI) helper module. Currently
only supports master mode. Provides an API that lets the firmware initiate a
request-response operation and then be notified when the operation finishes,
or poll the transaction status.

**NOTE:** This module can notify tasks when an I<sup>2</sup>C operation finishes,
but does not need a running task scheduler to function.

### avr/spihelper.h – SPI Communication
Helper module for SPI communication. Provides initialization routines for the SPI
hardware peripheral, a basic synchronous byte exchange function and an optional
asynchronous API.

**NOTE:** This module can notify tasks when an asynchronous SPI operation finishes,
but does not need a running task scheduler to function.

### avr/mcp23018.h – MCP23018 I/O Expander Interface
Helper module for controlling an MCP23018 I/O expander via I<sup>2</sup>C. Uses
the `i2chelper.h` module to communicate with the expander. Defines the register
addresses and control register pin numbers of an MCP23018 as enum constants.
Provides synchronous and asynchronous register read and write operations.

**NOTE:** This module can notify tasks when an I<sup>2</sup>C operation finishes,
but does not need a running task scheduler to function.

### avr/mcp4x.h – MCP4x Potentiometer Interface
Helper module for controlling an MCP4x potentiometer IC via SPI. Provides a basic
synchronous API for updating the wiper setting of a potentiometer, or disabling it.

**NOTE:** This module does not depend on the task scheduler.

### avr/nrf24x.h – nRF24x Transceiver Interface
Helper module for controlling an nRF24x wireless transceiver IC via SPI. Supports
both synchronous and asynchronous SPI communication.

**NOTE:** This module does not depend on the task scheduler.


## Example Firmware

### examples_atmega/sched_blinker.c – Simple LED Blinker
This is a simple example program that blinks two LEDs. The blinking can be turned
on and off via GPIO inputs and INM messages. An INM notification message is sent every
time the blink state of a LED changes. The program provides basic usage examples for
the task scheduler (`task_sched.h`), TLV/INM modules (`task_tlv.h` and `std_tlv.h`),
input debouncer (`tbouncer.h`) and automatic disabling of the watchdog timer
(`auto_watchdog.h`).

### examples_atmega/watchdog_test.c – Watchdog Timer Test
Test program for the watchdog timer control module (`watchdog.h`). Derived from
`sched_blinker.c`, designed to simplify testing whether software-triggered MCU reset
via the watchdog timer works as intended.

### examples_atmega/inm_node_demo.c – Sensors and Messages
Example of sensor sampling and INM messaging. Two analog inputs, a light sensor
and a microphone, are sampled by the same task, with the microphone being sampled
at a higher rate than the light sensor. INM notification messages are sent to report
the sensor values, for the microphone these are aggregated amplitude values instead
of the raw input data. The program provides examples of using the task scheduler
(`task_sched.h`), TLV/INM modules (`task_tlv.h` and `std_tlv.h`), the input
debouncer (`tbouncer.h`) and memory monitors (`memmon.h`).

### examples_atmega/nrf24x_test.c – nRF24x Wireless Demo
Demonstration of the nRF24x helper module (`nrf24x.h`). Lets two MCUs, each connected
to an nRF24L01+ transceiver, exchange information about the state of LEDs to keep them
synchronized. Also demonstrates updating the task handler pointer to implement a state
machine. Controlled via a TLV/INM interface (`task_tlv.h` and `std_tlv.h`).

*NOTE:* The `python/` subdirectory contains some scripts that simplify and test
communication with this firmware.

### examples_attiny/sched_test.c – Tiny Scheduler Test
Minimal test of the task scheduler (`task_sched.h`). Starts a task that blinks a LED
until you pull the plug.

### examples_attiny/stepper_test.c – Stepper Motor Demo
Demonstration of two-phase unipolar stepper motor (a 28BYJ-48, specifically) control
via a PWM generator in the MCU (generating two square waves that are 90° out of phase)
and a couple of external inverters (providing waves that are 180° and 270° out of phase
with the first wave, completing a full-step motor driving cycle). Provides examples
of using the task scheduler (`task_sched.h`) and input debouncer (`tbouncer.h`).

### examples_atmega_u/sched_blinker.c – Simple LED Blinker
Simple LED blinker for ATmegaU devices. Basically the same program as `sched_blinker.c`
for regular ATmegas, but adapted to the ATmegaU pinout.

### examples_atmega_u/lcd_driver.c – 7-seg LCD Demo
This program is designed to drive a seven-segment LCD via a PWM generator, an MCP23018
I/O expander and four 4066 quad transmission gates. The generated PWM signals can be
fed through an RC filter to obtain a makeshift AC supply that's good enough to drive
some LCD segments with. Provides usage examples for the task scheduler (`task_sched.h`),
input debouncer (`tbouncer.h`), I2C helper module (`i2chelper.h`) and MCP23018 interface
(`mcp23018.h`).

### examples_atmega_u/nrf24x_test.c – nRF24x Wireless Demo
Demonstration of the nRF24x helper module (`nrf24x.h`). Basically the same program as
`nrf24x_test.c` for regular ATmegas, but adapted to the ATmegaU pinout.


## Python Modules and Scripts

These Python modules use [pySerial](https://pyserial.readthedocs.io/en/latest/pyserial.html)
for serial port communication. No other third-party modules are used. Python 3.6 or later is
required.

### python/inm/inm.py – General API for INM
This is a general-purpose Python API for INM communication. The primary abstraction used is
the _message channel_, representing a communication interface by which messages can be
exchanged with other INM nodes. The current version provides serial port and UDP/IP channels.
There is also a special message channel class that provides basic routing functionality, using
a static routing table to forward incoming messages via one of a set of encapsulated message
channels. Message channels currently only support synchronous I/O, with timeouts.

_Message factory_ objects are used to perform conversions between application-specific message
payloads and the generic TLV message objects that message channels send and receive. Message
payloads generally consist of a sequence of fields, where each field may be converted to or
from a sequence of bytes by a message factory. Message factories have configuration parameters
and conversion tables that associate message types with payload formats. Together, these allow
extensive customization, reducing the amount of message packing and unpacking that needs to
be done in application code.

### python/inm/helper.py – Convenience API for INM
This module is intended to simplify the implementation of INM in client nodes. It provides
a convenience wrapper class for an INM message channel, with an API for constructing,
sending and/or receiving messages that is more simple than using the `inm.py` API directly.

### python/inm_router.py – INM Router Script
INM message router script. Typically configured via CLI arguments read from a file. May be
installed as a Linux service via the makefile in the `python/` directory. Based on the routing
message channel in the `inm.py` module.

Run the script with the `-h` option to get a list of available options.
The file `router_args.example` in the `python/` directory shows how the script may be configured.

### python/inm_memmon.py – Memory Monitor Visualization
Text-based visualization script for libavr memory monitors (`memmon.h`). Listens for INM
messages of a specified type and outputs their payloads (assumed to have the format
of a standard `MEMMON_DATA` message) as an ASCII graph on standard output. Typically
configured via CLI arguments read from a file.

Run the script with the `-h` option to get a list of available options.
The file `memmon_args.example` in the `python/` directory shows how the script may be configured.

### python/nrf24x.py – nRF24x Helper Functions
This module contains helper functions for TLV/INM communication with the `nrf24x_test.c` example
firmware. These functions can read and write registers in the nRF24x via a single call, with
error checking.

The test scripts `nrf24x_send.py` and `nrf24x_recv.py` use this module to verify that
the AVR firmware and the nRF24x IC are communicating and generally behaving as expected.
