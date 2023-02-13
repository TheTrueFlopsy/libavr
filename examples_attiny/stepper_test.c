
#include <stddef.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#include "task_sched.h"
#include "tbouncer.h"


// ---<<< I/O Constants >>>---
#define TBOUNCER_TASK_CAT 14
#define STEPPER_TASK_CAT 1

#define TBOUNCER_DELAY_MS 20
#define OUTPUT_SHUTDOWN_DELAY_MS 100

#define ENABLE_DDR  DDRA
#define ENABLE_PORT PORTA

#define ENABLE_DDR_PIN  BV(DDA0)
#define ENABLE_PORT_PIN BV(PA0)
#define ENABLE_PIN_PIN  BV(PINA0)

#define DIRECTION_DDR  DDRA
#define DIRECTION_PORT PORTA
#define DIRECTION_PIN PINA

#define DIRECTION_DDR_PIN  BV(DDA1)
#define DIRECTION_PORT_PIN BV(PA1)
#define DIRECTION_PIN_PIN  BV(PINA1)

#define LED_DDR  DDRA
#define LED_PORT PORTA

#define LED_DDR_PIN  BV(DDA2)
#define LED_PORT_PIN BV(PA2)

#define ENABLE_INVERTERS_DDR  DDRA
#define ENABLE_INVERTERS_PORT PORTA

#define ENABLE_INVERTERS_DDR_PIN  BV(DDA3)
#define ENABLE_INVERTERS_PORT_PIN BV(PA3)

#define FREQ_SELECT_DDR  DDRA
#define FREQ_SELECT_PORT PORTA

#define FREQ_SELECT_DDR_PIN  BV(DDA4)
#define FREQ_SELECT_PORT_PIN BV(PA4)

/*#define PWM_OUTPUT_DDR_A  DDRA
#define PWM_OUTPUT_PORT_A PORTA
#define PWM_OUTPUT_DDR_B  DDRA
#define PWM_OUTPUT_PORT_B PORTA*/

#define PWM_OUTPUT_DDR  DDRA
#define PWM_OUTPUT_PORT PORTA

#define PWM_OUTPUT_DDR_PIN_A  BV(DDA6)
#define PWM_OUTPUT_DDR_PIN_B  BV(DDA5)
#define PWM_OUTPUT_PORT_PIN_A BV(PA6)
#define PWM_OUTPUT_PORT_PIN_B BV(PA5)


// ---<<< Type and Constant Definitions >>>---
#define E_POW_X_SHIFT 3
#define POLY_VAR_SHIFT 11
#define FRACTION_SHIFT 4

#define PWM_PRESCALER_LOG2 3
#define PWM_PRESCALER (1 << PWM_PRESCALER_LOG2)
#define PWM_FREQ_MIN (F_CPU / (2 * PWM_PRESCALER * 65536) + 1)
#define PWM_FREQ_MAX (F_CPU / (2 * PWM_PRESCALER * 2))

// NOTE: The datasheets are pretty vague, but going by what someone on the internet
//       wrote, the stated 5.625-degree (1/64 turn) stride angle of the motor itself
//       (pre-gearing) applies to half-stepping. This would mean that it takes
//       (64/2)*64 = 2048 full steps to rotate the output shaft one turn.
// 25 Hz square wave outputs, 50 timer cycles per second, 100 full steps per second,
// ~20 seconds per turn
#define PWM_FREQ_INIT 25

#define INVALID_ADC_INPUT 0xffff

typedef uint16_t output_freq;

enum {
	STATE_NONE     = 0,
	STATE_STARTUP  = 1,  // Waiting for ADC.
	STATE_CANCELED = 2,  // Stopped during startup.
	STATE_STARTED  = 3,
	STATE_SHUTDOWN = 4,  // Waiting to shut down motor control outputs.
	STATE_STOPPED  = 5
};

enum {
	OUTPUT_STATE_NONE   = 0,
	OUTPUT_STATE_ON     = 1,  // Output being produced.
	OUTPUT_STATE_PAUSED = 2,  // Output timer stopped, output pins still enabled.
	OUTPUT_STATE_OFF    = 3   // Output pins disabled.
};


// ---<<< Data Definitions >>>---
static volatile uint16_t adc_input = INVALID_ADC_INPUT;

static output_freq pulse_freq      = 16 * PWM_FREQ_INIT;  // square wave frequency in Hz/16
// If this is true, the motor control sequence is [(-A,-B), (-A,+B), (+A,+B), (+A,-B)] (i.e. 0132).
// If this is false, the sequence is [(-A,+B), (-A,-B), (+A,-B), (+A,+B)] (i.e. 1023).
static uint8_t     pulse_direction = 1;
static uint8_t     pulse_state     = STATE_STOPPED;  // motor control output not requested
static uint8_t     pulse_output    = OUTPUT_STATE_OFF;  // motor control output disabled


// ---<<< ISRs >>>---
ISR(ADC_vect) {
	adc_input = ADC;
	sched_isr_tcww |= SCHED_CATFLAG(STEPPER_TASK_CAT);
}


// ---<<< Helper Functions >>>---
static uint16_t int_exp_poly(uint16_t x) {
	uint32_t x32;
	uint16_t res = 1U << POLY_VAR_SHIFT;
	
	x32 = x;
	res += x;
	
	x32 *= x;
	res += (uint16_t)((x32 / 2U) >> POLY_VAR_SHIFT);
	
	x32 *= x;
	res += (uint16_t)((x32 / 6U) >> 2*POLY_VAR_SHIFT);
	
	return res;
}

static uint16_t int_exp(uint16_t x) {
	uint32_t y = int_exp_poly(x);
	uint8_t i;
	
	for (i = 0; i < E_POW_X_SHIFT; i++) {
		y *= y;
		y >>= POLY_VAR_SHIFT;
	}
	
	y >>= (POLY_VAR_SHIFT - FRACTION_SHIFT);
	
	return (uint16_t)y;
}

static uint16_t hz_to_top(output_freq hz16) {  // Argument is in 16ths of a hertz.
	// F_COUNT = F_CPU / PWM_PRESCALER
	//         = F_CPU >> PWM_PRESCALER_LOG2.
	// top = (F_COUNT / (2*hz)) - 1
	//     = (F_COUNT >> 1) / hz - 1
	//     = ((F_CPU >> PWM_PRESCALER_LOG2) >> 1) / hz - 1
	//     = (F_CPU >> (PWM_PRESCALER_LOG2 + 1)) / hz - 1.
	//uint16_t top = (uint16_t)((F_CPU >> (PWM_PRESCALER_LOG2 + 5)) / hz16 - 1);
	uint16_t top = (uint16_t)(F_CPU / hz16 - 1);
	
	// NOTE: To get output signals in quadrature, use only odd values for top.
	//   Top (i.e. (2*N - 1)) is odd, (top // 2) is ((top+1)/2 - 1) (i.e. N-1), correct:
	//     (2*N - 1) // 2 = (2*(N-1) + 1) // 2 = N - 1.  # Cycle evenly divided: [0, N-1], [N, 2*N-1]
	//   Top (i.e. (2*N)) is even, (top // 2) is (top/2) (i.e. N), incorrect:
	//     (2*N) // 2 = N.  # Cycle NOT evenly divided: [0, N], [N+1, 2*N]
	if ((0x01 & top) == 0)
		top++;
	
	return top;
}

static void pulse_output_on(void) {
	if (pulse_output != OUTPUT_STATE_OFF) // Not fully disabled, do not attempt to enable.
		return;
	
	uint16_t top = hz_to_top(pulse_freq);
	
	// Enable motor control output inverters.
	ENABLE_INVERTERS_PORT |= ENABLE_INVERTERS_PORT_PIN;
	
	// Reset Timer1.
	TCNT1 = 0;
	OCR1A = top;       // Toggle OC1A at the end of each cycle.
	OCR1B = top >> 1;  // Toggle OC1B halfway through each cycle.
	
	// Re-initialize the compare match outputs.
	TCCR1A |= BV(COM1A1) | BV(COM1A0) | BV(COM1B1) | BV(COM1B0);  // Go to set-on-match mode (COM1x1=1,COM1x0=1).
	
	if (pulse_direction)  // Start sequence at (OC1A=0, OC1B=0) (i.e. (-A,-B)).
		TCCR1A &= ~(BV(COM1A0) | BV(COM1B0));  // Go to clear-on-match mode (COM1x1=1,COM1x0=0).
	else  // Start sequence at (OC1A=0, OC1B=1) (i.e. (-A,+B)).
		TCCR1A &= ~BV(COM1A0);  // Go to clear-on-match mode only for compare unit A.
	
	TCCR1C |= BV(FOC1A) | BV(FOC1B);  // Force output compare latches to desired state.
	
	// Restore toggle-on-match mode (COM1x1=0,COM1x0=1).
	TCCR1A |= BV(COM1A0) | BV(COM1B0);
	TCCR1A &= ~(BV(COM1A1) | BV(COM1B1));
	
	// Make Timer1's OCA and OCB pins outputs.
	PWM_OUTPUT_DDR |= (PWM_OUTPUT_DDR_PIN_A | PWM_OUTPUT_DDR_PIN_B);
	
	// ISSUE: Insert a short delay here?
	
	// Timer clock on.
	TCCR1B |= BV(CS11);  // Prescaled clock setting 1/8.
	//TCCR1B |= BV(CS12) | BV(CS10);  // Prescaled clock setting 1/1024.
	
	pulse_output = OUTPUT_STATE_ON;
}

static void pause_pulse_output(void) {
	if (pulse_output == OUTPUT_STATE_OFF) // Already disabled, do not attempt to pause.
		return;
	
	// Timer clock off.
	TCCR1B &= ~(BV(CS12) | BV(CS11) | BV(CS10)); // Stop the clock.
	
	pulse_output = OUTPUT_STATE_PAUSED;
}

static void pulse_output_off(void) {
	// Timer clock off, in case that hasn't been done.
	TCCR1B &= ~(BV(CS12) | BV(CS11) | BV(CS10)); // Stop the clock.
	
	// Make Timer1's OCA and OCB pins inputs.
	PWM_OUTPUT_DDR &= ~(PWM_OUTPUT_DDR_PIN_A | PWM_OUTPUT_DDR_PIN_B);
	
	// ISSUE: Insert a short delay here?
	
	// Disable motor control output inverters.
	ENABLE_INVERTERS_PORT &= ~ENABLE_INVERTERS_PORT_PIN;
	
	pulse_output = OUTPUT_STATE_OFF;
}


// ---<<< Task Handlers >>>---
static void stepper_handler(sched_task *task) {
	// Detect on/off input (falling edge starts motor control output,
	// rising edge stops it).
	uint8_t starting =
		(pulse_state == STATE_STOPPED && (ENABLE_PIN_PIN & TBOUNCER_A_FALLING));
	uint8_t stopping =
		((pulse_state == STATE_STARTUP || pulse_state == STATE_STARTED)
		 && (ENABLE_PIN_PIN & TBOUNCER_A_RISING));
	uint8_t shutdown_ready =
		(pulse_state == STATE_SHUTDOWN && sched_time_is_zero(task->delay));
	
	if (starting) {
		// Sample the potentiometer input to get desired frequency
		// of the motor control output.
		adc_input = INVALID_ADC_INPUT;
		ADCSRA |= BV(ADSC);
		
		pulse_state = STATE_STARTUP;
		task->st |= TASK_SLEEP_BIT;  // Put the task to sleep.
	}
	else if (stopping) {
		if (pulse_state == STATE_STARTED) {
			// Stop the timer that generates the motor control output.
			pause_pulse_output();
			
			pulse_state = STATE_SHUTDOWN;
			task->delay = SCHED_TIME_MS(OUTPUT_SHUTDOWN_DELAY_MS);
		}
		else if (pulse_state == STATE_STARTUP) {
			pulse_state = STATE_CANCELED;
			task->st |= TASK_SLEEP_BIT;  // Put the task to sleep.
		}
	}
	else if (pulse_state == STATE_STARTUP) {
		uint16_t adc_input_val = adc_input;
		
		if (adc_input_val != INVALID_ADC_INPUT) {  // ADC conversion finished.
			// Use an exponential relationship between ADC value and PWM frequency.
			pulse_freq = int_exp(adc_input_val) + 16;
			
			// Get desired direction of rotation.
			pulse_direction = !!(DIRECTION_PIN_PIN & DIRECTION_PIN);
			
			// Starting, turn on the indicator LED.
			LED_PORT |= LED_PORT_PIN;
			
			// Start motor control output.
			pulse_output_on();
			
			pulse_state = STATE_STARTED;
		}
		
		task->st |= TASK_SLEEP_BIT;  // Put the task to sleep.
	}
	else if (pulse_state == STATE_CANCELED) {
		uint16_t adc_input_val = adc_input;
		
		if (adc_input_val != INVALID_ADC_INPUT)  // ADC conversion finished.
			pulse_state = STATE_STOPPED;
		
		task->st |= TASK_SLEEP_BIT;  // Put the task to sleep.
	}
	else if (shutdown_ready) {
		// Stopped, turn off the indicator LED.
		LED_PORT &= ~LED_PORT_PIN;
		
		// Disable motor control output.
		// NOTE: Probably a bad idea to keep burning power in the coils,
		//       so all the motor control outputs are turned off after
		//       a little while.
		pulse_output_off();
		
		pulse_state = STATE_STOPPED;
		task->st |= TASK_SLEEP_BIT;  // Put the task to sleep.
	}
	else {  // Nothing of interest happening.
		task->st |= TASK_SLEEP_BIT;  // Put the task to sleep.
	}
}


// ---<<< Entry Point and Initializers >>>---
static void init_adc(void) {
	ADMUX = BV(MUX2);  // Input from PA4.
	// Enable interrupt, prescaler 8 (Fadc=125 kHz @ Fcpu=1 MHz).
	ADCSRA = BV(ADIE) | BV(ADPS1) | BV(ADPS0);
	ADCSRB = 0;
	DIDR0 = BV(ADC4D);  // Disable digital input on PA4.
	
	ADCSRA |= BV(ADEN); // Enable ADC.
	
	// Do a warm-up conversion.
	ADCSRA |= BV(ADSC); // Start conversion.
	
	while (BV(ADSC) & ADCSRA)  // Wait for conversion to finish.
		;
	
	ADCSRA |= BV(ADIF); // Clear interrupt flag.
}

static void init_pwm_generator(void) {
	//uint16_t top = hz_to_top(PWM_FREQ_INIT);
	
	// Clear Timer1.
	//TCNT1 = 0;
	//OCR1A = top;       // Toggle OC1A at the end of each cycle.
	//OCR1B = top >> 1;  // Toggle OC1B halfway through each cycle.
	
	// Configure Timer1.
	TCCR1A = BV(COM1A0) | BV(COM1B0);  // Toggle output pins on compare match.
	TCCR1B = BV(WGM12);  // Enable CTC mode. Timer clock stopped for now.
	
	// Make the output inverter supply pin an output.
	ENABLE_INVERTERS_DDR |= ENABLE_INVERTERS_DDR_PIN;
}

static void init_tasks(void) {
	sched_task task;
	
	task = (sched_task) {
		.st = TASK_ST_MAKE(0, STEPPER_TASK_CAT, 1),
		.delay = SCHED_TIME_ZERO,
		.handler = stepper_handler
	};
	sched_add(&task);
}

// NOTE: This is the entry point, tell compiler not to save/restore registers.
int main(void) __attribute__ ((OS_main));

int main(void) {
	// Initialize the ADC and PWM generator.
	init_adc();
	init_pwm_generator();
	
	// Other I/O initialization.
	LED_DDR |= LED_DDR_PIN;  // Make LED pin an output.
	
	ENABLE_PORT |= ENABLE_PORT_PIN;  // Enable pull-up on output-enable input pin.
	DIRECTION_PORT |= DIRECTION_PORT_PIN;  // Enable pull-up on output direction input pin.
	
	sched_init();
	TBOUNCER_INIT(
		TASK_ST_MAKE(0, TBOUNCER_TASK_CAT, 0), SCHED_TIME_MS(TBOUNCER_DELAY_MS),
		ENABLE_PIN_PIN, 0,
		0, TASK_ST_CAT_MASK, TASK_ST_CAT(STEPPER_TASK_CAT));
	
	init_tasks();
	
	sched_run();
	
	return 0;
}
