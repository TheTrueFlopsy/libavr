
#include <stddef.h>
#include <avr/interrupt.h>
#include <avr/io.h>

// NOTE: When "auto_watchdog.h" is included, the watchdog timer is automatically
//       disabled following an MCU reset.
#include "auto_watchdog.h"
#include "task_sched.h"


// ---<<< Constant Definitions >>>---
#define TEST_TASK_CAT 1

#define LED_DDR  DDRA
#define LED_PORT PORTA
#define LED_PIN  PINA

#define LED_DDR_PIN  BV(DDA2)
#define LED_PORT_PIN BV(PA2)
#define LED_PIN_PIN BV(PINA2)


// ---<<< Task Handlers >>>---
static void test_handler(sched_task *task) {
	LED_PIN = LED_PIN_PIN;  // Write 1 to toggle.
	
	task->delay = SCHED_TIME_MS(500);
}


// ---<<< Entry Point and Initializers >>>---
static void init_tasks(void) {
	sched_task task;
	
	task = (sched_task) {
		.st = TASK_ST_MAKE(0, TEST_TASK_CAT, 0),
		.delay = SCHED_TIME_ZERO,
		.handler = test_handler
	};
	sched_add(&task);
}

// NOTE: This is the entry point, tell compiler not to save/restore registers.
int main(void) __attribute__ ((OS_main));

int main(void) {
	// Other I/O initialization.
	LED_DDR |= LED_DDR_PIN;  // Make LED pin an output.
	
	sched_init();
	
	init_tasks();
	
	sched_run();
	
	return 0;
}
