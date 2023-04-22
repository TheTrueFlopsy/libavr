
#include <stddef.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

// NOTE: When "auto_watchdog.h" is included, the watchdog timer is automatically
//       disabled following an MCU reset.
#include "auto_watchdog.h"
#include "i2chelper.h"
#include "mcp23018.h"
#include "task_sched.h"
#include "tbouncer.h"


// ---<<< I/O Constants >>>---
// Input: LDC drive enable
#define EN_DRIVE_DDR  DDRD
#define EN_DRIVE_PORT PORTD

#define EN_DRIVE_DDR_PIN  BV(DDD4)
#define EN_DRIVE_PORT_PIN BV(PORTD4)
#define EN_DRIVE_PIN_PIN  BV(PIND4)  // D4 on the Leonardo
#define EN_DRIVE_PIN_FALLING TBOUNCER_D_FALLING
#define EN_DRIVE_PIN_RISING TBOUNCER_D_RISING

// Input: LDC backlight enable
#define EN_BLITE_DDR  DDRD
#define EN_BLITE_PORT PORTD

#define EN_BLITE_DDR_PIN  BV(DDD7)
#define EN_BLITE_PORT_PIN BV(PORTD7)
#define EN_BLITE_PIN_PIN  BV(PIND7)  // D6 on the Leonardo
#define EN_BLITE_PIN_FALLING TBOUNCER_D_FALLING
#define EN_BLITE_PIN_RISING TBOUNCER_D_RISING

// Output: MCP23018 reset
#define IOEXP_RESET_DDR  DDRB
#define IOEXP_RESET_PORT PORTB

#define IOEXP_RESET_DDR_PIN  BV(DDB4)
#define IOEXP_RESET_PORT_PIN BV(PORTB4)  // D8 on the Leonardo

// Output: LDC backlight control
#define BLITE_DDR  DDRC
#define BLITE_PORT PORTC

#define BLITE_DDR_PIN  BV(DDC7)
#define BLITE_PORT_PIN BV(PORTC7)  // D13 on the Leonardo

// Output: initialization success indicator
#define LED_OK_DDR  DDRD
#define LED_OK_PORT PORTD
#define LED_OK_PIN  PIND

#define LED_OK_DDR_PIN  BV(DDD5)
#define LED_OK_PORT_PIN BV(PORTD5)  // TX_LED on the Leonardo, active low
#define LED_OK_PIN_PIN  BV(PIND5)

// Output: error indicator
#define LED_ERR_DDR  DDRB
#define LED_ERR_PORT PORTB

#define LED_ERR_DDR_PIN  BV(DDB0)
#define LED_ERR_PORT_PIN BV(PORTB0)  // RX_LED on the Leonardo, active low

// Output: LDC drive PWM signal
#define PWM_OUTPUT_DDR  DDRB
#define PWM_OUTPUT_PORT PORTB

#define PWM_OUTPUT_DDR_PIN_A  BV(DDB5)
#define PWM_OUTPUT_DDR_PIN_B  BV(DDB6)
#define PWM_OUTPUT_PORT_PIN_A BV(PORTB5)  // D9 on the Leonardo
#define PWM_OUTPUT_PORT_PIN_B BV(PORTB6)  // D10 on the Leonardo

#define PWM_PRESCALER_LOG2 6
#define PWM_PRESCALER (1 << PWM_PRESCALER_LOG2)
#define PWM_FREQ_MIN (F_CPU / (2 * PWM_PRESCALER * 65536) + 1)
#define PWM_FREQ_MAX (F_CPU / (2 * PWM_PRESCALER * 2))

#define PWM_FREQ_INIT 300

// Output: I2C interface to I/O expander
// NOTE: At CPU clock rate 16 MHz and TWI prescaler setting 1 (the default),
//       TWBR=72 corresponds to the I2C clock rate (16 MHz)/(16 + 2*72*1) == 100 kHz.
// CAUTION: ATmega(16|32)U4 datasheet (p. 231) says that TWBR settings below 10 are FUBAR.
#define TWBR_INIT 72

// NOTE: This assumes that the three settable address bits
//       on the MCP23018 are equal to 2 (i.e. 0b010).
#define MCP23018_ADDR 0b00100010


// ---<<< Type and Constant Definitions >>>---
#define TBOUNCER_DELAY_MS 4
#define OUTPUT_SHUTDOWN_DELAY_MS 100

enum {
	LCD_DRIVE_TASK_CAT  =  1,
	ADC_INPUT_TASK_CAT  =  2,
	IOEXP_CTRL_TASK_CAT =  3,
	TBOUNCER_TASK_CAT   = 14
};

enum {
	OUTPUT_STATE_NONE   = 0,
	OUTPUT_STATE_ON     = 1,  // Output being produced.
	OUTPUT_STATE_OFF    = 2   // Output timer stopped, output pins disabled.
};

enum {
	LCD_CH_SP = 0b00000000,  // all segments off
	LCD_CH_DA = 0b10000000,  // G
	LCD_CH_US = 0b00010000,  // D
	LCD_CH_0  = 0b01111110,  // ABCDEF
	LCD_CH_1  = 0b00001100,  // BC
	LCD_CH_2  = 0b10110110,  // ABDEG
	LCD_CH_3  = 0b10011110,  // ABCDG
	LCD_CH_4  = 0b11001100,  // BCFG
	LCD_CH_5  = 0b11011010,  // ACDFG
	LCD_CH_6  = 0b11111010,  // ACDEFG
	LCD_CH_7  = 0b00001110,  // ABC
	LCD_CH_8  = 0b11111110,  // ABCDEFG
	LCD_CH_9  = 0b11011110,  // ABCDFG
	LCD_CH_A  = 0b11101110,  // ABCEFG
	LCD_CH_B  = 0b11111000,  // CDEFG
	LCD_CH_C  = 0b01110010,  // ADEF
	LCD_CH_D  = 0b10111010,  // BCDEG
	LCD_CH_E  = 0b11110010,  // ADEFG
	LCD_CH_F  = 0b11100010   // AEFG
};

typedef uint16_t output_freq;

const uint8_t lcd_hex_digits[16] PROGMEM = {
	LCD_CH_0, LCD_CH_1, LCD_CH_2, LCD_CH_3,
	LCD_CH_4, LCD_CH_5, LCD_CH_6, LCD_CH_7,
	LCD_CH_8, LCD_CH_9, LCD_CH_A, LCD_CH_B,
	LCD_CH_C, LCD_CH_D, LCD_CH_E, LCD_CH_F
};


// ---<<< Program State >>>---
static output_freq pulse_freq   = PWM_FREQ_INIT;     // square wave frequency in Hz
static uint8_t     pulse_output = OUTPUT_STATE_OFF;  // LCD drive output disabled

static uint8_t adc_started = 0;  // ADC sampling status flag.

static uint8_t ioexp_a_requested = 0x00;
static uint8_t ioexp_a_attempted = 0x00;
static uint8_t ioexp_a_committed = 0x00;

static uint8_t ioexp_b_requested = 0x00;
static uint8_t ioexp_b_attempted = 0x00;
static uint8_t ioexp_b_committed = 0x00;


// ---<<< ISRs >>>---
ISR(ADC_vect) {  // ADC conversion complete
	sched_isr_tcww |= SCHED_CATFLAG(ADC_INPUT_TASK_CAT);  // Awaken the ADC input task.
}


// ---<<< Helper Functions >>>---
static uint8_t lcd_display_decimal(int16_t val) {
	uint16_t uval = (uint16_t)abs(val);
	uint8_t ioexp_a_val, ioexp_b_val;
	
	if (uval > 990)
		return 0;  // Value out of range.
	
	if (uval < 100) {  // Display tenths.
		uint8_t d_tenths = (uint8_t)(uval % 10);
		uint8_t d_units = (uint8_t)(uval / 10);
		
		// Set segment bits for decimal point and digits.
		ioexp_a_val = pgm_read_byte_near(lcd_hex_digits + d_tenths) | 0x01;
		ioexp_b_val = pgm_read_byte_near(lcd_hex_digits + d_units);
	}
	else {  // Display tens.
		uint8_t d_units = (uint8_t)((uval % 100) / 10);
		uint8_t d_tens = (uint8_t)(uval / 100);
		
		// Set segment bits for digits.
		ioexp_a_val = pgm_read_byte_near(lcd_hex_digits + d_units);
		ioexp_b_val = pgm_read_byte_near(lcd_hex_digits + d_tens);
	}
	
	if (val < 0)
		ioexp_b_val |= 0x01;  // Set segment bit for minus sign.
	
	if (ioexp_a_val != ioexp_a_requested || ioexp_b_val != ioexp_b_requested) {
		ioexp_a_requested = ioexp_a_val;
		ioexp_b_requested = ioexp_b_val;
		sched_task_tcww |= SCHED_CATFLAG(IOEXP_CTRL_TASK_CAT);  // Awaken expander controller.
	}
	
	return 1;  // Success!
}

static uint16_t hz_to_top(output_freq hz) {
	// F_COUNT = F_CPU / PWM_PRESCALER
	//         = F_CPU >> PWM_PRESCALER_LOG2.
	// top = (F_COUNT / (2*hz)) - 1
	//     = (F_COUNT >> 1) / hz - 1
	//     = ((F_CPU >> PWM_PRESCALER_LOG2) >> 1) / hz - 1
	//     = (F_CPU >> (PWM_PRESCALER_LOG2 + 1)) / hz - 1.
	uint16_t top = (uint16_t)((F_CPU >> (PWM_PRESCALER_LOG2 + 1)) / hz - 1);
	
	// NOTE: To get symmetric output signals, use only odd values for top.
	//   Top (i.e. (2*N - 1)) is odd, (top // 2) is ((top+1)/2 - 1) (i.e. N-1), correct:
	//     (2*N - 1) // 2 = (2*(N-1) + 1) // 2 = N - 1.  # Cycle evenly divided: [0, N-1], [N, 2*N-1]
	//   Top (i.e. (2*N)) is even, (top // 2) is (top/2) (i.e. N), incorrect:
	//     (2*N) // 2 = N.  # Cycle NOT evenly divided: [0, N], [N+1, 2*N]
	if ((0x01 & top) == 0)
		top++;
	
	return top;
}

static void pulse_output_on(void) {
	if (pulse_output != OUTPUT_STATE_OFF)  // Not disabled, do not attempt to enable.
		return;
	
	uint16_t top = hz_to_top(pulse_freq);
	
	// Reset Timer1.
	TCNT1 = 0;
	OCR1A = top;  // Toggle OC1A at the end of each cycle.
	OCR1B = top;  // Toggle OC1B at the end of each cycle, too.
	
	// Re-initialize the compare match outputs.
	// Start sequence at (OC1A=0, OC1B=1) (i.e. (-A,+B)).
	TCCR1A |= BV(COM1A1) | BV(COM1A0) | BV(COM1B1) | BV(COM1B0);  // Go to set-on-match mode (COM1x1=1,COM1x0=1).
	TCCR1A &= ~BV(COM1A0);  // Go to clear-on-match mode only for compare unit A.
	
	TCCR1C |= BV(FOC1A) | BV(FOC1B);  // Force output compare latches to desired state.
	
	// Restore toggle-on-match mode (COM1x1=0,COM1x0=1).
	TCCR1A |= BV(COM1A0) | BV(COM1B0);
	TCCR1A &= ~(BV(COM1A1) | BV(COM1B1));
	
	// Make Timer1's OCA and OCB pins outputs.
	PWM_OUTPUT_DDR |= (PWM_OUTPUT_DDR_PIN_A | PWM_OUTPUT_DDR_PIN_B);
	
	// ISSUE: Insert a short delay here?
	
	// Timer clock on.
	TCCR1B |= BV(CS11) | BV(CS10);  // Prescaled clock setting 1/64.
	
	pulse_output = OUTPUT_STATE_ON;
}

static void pulse_output_off(void) {
	// Timer clock off.
	TCCR1B &= ~(BV(CS12) | BV(CS11) | BV(CS10));  // Stop the clock.
	
	// Make Timer1's OCA and OCB pins inputs.
	PWM_OUTPUT_DDR &= ~(PWM_OUTPUT_DDR_PIN_A | PWM_OUTPUT_DDR_PIN_B);
	
	// ISSUE: Insert a short delay here?
	
	pulse_output = OUTPUT_STATE_OFF;
}


// ---<<< Task Handlers >>>---
static void lcd_drive_handler(sched_task *task) {
	// Detect backlight on/off input (falling edge turns on backlight, rising edge turns it off).
	uint8_t blite_on = EN_BLITE_PIN_PIN & EN_BLITE_PIN_FALLING;
	uint8_t blite_off = EN_BLITE_PIN_PIN & EN_BLITE_PIN_RISING;
	
	// Detect driver on/off input (falling edge starts LCD drive output, rising edge stops it).
	uint8_t starting =
		(pulse_output == OUTPUT_STATE_OFF && (EN_DRIVE_PIN_PIN & EN_DRIVE_PIN_FALLING));
	uint8_t stopping =
		(pulse_output == OUTPUT_STATE_ON && (EN_DRIVE_PIN_PIN & EN_DRIVE_PIN_RISING));
	
	task->st |= TASK_SLEEP_BIT;  // Put the task back to sleep. (It should run only when notified.)
	
	if (blite_on)
		BLITE_PORT |= BLITE_PORT_PIN;
	else if (blite_off)
		BLITE_PORT &= ~BLITE_PORT_PIN;
	
	if (starting)
		pulse_output_on();  // Start LCD drive output.
	else if (stopping)
		pulse_output_off();  // Disable LCD drive output.
	// else: Nothing of interest happening.
}

static void adc_input_handler(sched_task *task) {
	if (adc_started) {
		uint16_t adc_res = ADCW;
		int16_t disp_val = (int16_t)adc_res - 512;
		
		if (disp_val >= 100)
			disp_val = 100 + (int16_t)((890 * (int32_t)(disp_val - 100)) / 411);
		else if (disp_val <= -100)
			disp_val = -100 + (int16_t)((890 * (int32_t)(disp_val + 100)) / 412);
		
		lcd_display_decimal(disp_val);
		
		adc_started = 0;
		task->delay = SCHED_TIME_MS(20);  // Wait 20 ms between conversions (sample rate ~50 Hz).
	}
	else {
		ADCSRA |= BV(ADSC);  // Start ADC conversion.
		adc_started = 1;
		task->st |= TASK_SLEEP_BIT;  // Wait for conversion to finish.
	}
}

static void ioexp_ctrl_handler(sched_task *task) {
	task->st |= TASK_SLEEP_BIT;  // Put the task back to sleep. (It should run only when notified.)
	
	if (I2C_IS_ACTIVE)  // I2C module busy, unable to proceed.
		return;
	else if (!I2C_IS_READY) {  // I2C error, unable to proceed.
		LED_ERR_PORT &= ~LED_ERR_PORT_PIN;  // Signal the error.
		// IDEA: Try to recover by restarting the I2C module, or the whole MCU?
		return;
	}
	else
		LED_ERR_PORT |= LED_ERR_PORT_PIN;  // Clear error signal.
	
	// Any pending update attempt (there can be at most one) must have succeeded,
	// so set committed=attempted.
	ioexp_a_committed = ioexp_a_attempted;
	ioexp_b_committed = ioexp_b_attempted;
	
	if (ioexp_a_requested != ioexp_a_committed) {  // Bank A out-of-date.
		i2c_state res = mcp23018_begin_write(MCP23018_ADDR, MCP23018_GPIOA, ioexp_a_requested);
		
		if (res != I2C_ACTIVE) {  // Failed to initiate write operation.
			LED_ERR_PORT &= ~LED_ERR_PORT_PIN;  // Signal the error.
			return;  // Hope that the error was temporary, try again next time.
		}
		
		ioexp_a_attempted = ioexp_a_requested;  // Record update attempt.
	}
	else if (ioexp_b_requested != ioexp_b_committed) {  // Bank B out-of-date.
		i2c_state res = mcp23018_begin_write(MCP23018_ADDR, MCP23018_GPIOB, ioexp_b_requested);
		
		if (res != I2C_ACTIVE) {  // Failed to initiate write operation.
			LED_ERR_PORT &= ~LED_ERR_PORT_PIN;  // Signal the error.
			return;  // Hope that the error was temporary, try again next time.
		}
		
		ioexp_b_attempted = ioexp_b_requested;  // Record update attempt.
	}
	// else: Requested output state already achieved, nothing to do.
}


// ---<<< Initialization Routines and Main Function >>>---
static void disable_usb(void) {
	// TODO: Investigate precisely which USB features the Leonardo bootloader
	//       leaves enabled.
	USBCON &= ~BV(VBUSTE);
	USBINT &= ~BV(VBUSTI);
	UDIEN &= ~(BV(UPRSME) | BV(EORSME) | BV(WAKEUPE) | BV(EORSTE) | BV(SOFE) | BV(SUSPE));
	UDINT &= ~(BV(UPRSMI) | BV(EORSMI) | BV(WAKEUPI) | BV(EORSTI) | BV(SOFI) | BV(SUSPI));
	USBCON &= ~BV(USBE);
	//USBCON |= BV(FRZCLK);
}

static void init_pwm_generator(void) {
	// Configure Timer1.
	TCCR1A = BV(COM1A0) | BV(COM1B0);  // Toggle output pins on compare match.
	TCCR1B = BV(WGM12);  // Enable CTC mode. Timer clock stopped for now.
}

static void init_adc(void) {
	// Use external AREF, select ADC channel 7 (pin PF7, A0 on the Leonardo).
	//ADMUX = BV(MUX2) | BV(MUX1) | BV(MUX0);
	// Use 2.56 V internal AREF, select ADC channel 7 (pin PF7, A0 on the Leonardo).
	//ADMUX = BV(REFS1) | BV(REFS0) | BV(MUX2) | BV(MUX1) | BV(MUX0);
	// Use AVcc as internal AREF, select ADC channel 7 (pin PF7, A0 on the Leonardo).
	ADMUX = BV(REFS0) | BV(MUX2) | BV(MUX1) | BV(MUX0);
	// Clock prescaler setting 128 (ADC clock 125 kHz @ 16 MHz CPU clock).
	ADCSRA = BV(ADPS2) | BV(ADPS1) | BV(ADPS0);
	ADCSRB = 0;  // Default settings.
	DIDR0 |= BV(ADC7D);  // Disable digital input on the ADC7 pin.
	
	ADCSRA |= BV(ADEN);  // Enable the ADC.
	
	// Do a warmup conversion.
	ADCSRA |= BV(ADSC);
	
	while (ADCSRA & BV(ADSC))  // Conversion ongoing.
		_delay_us(10.0);  // Pass the time.
	
	ADCSRA |= BV(ADIE);  // Enable the ADC interrupt.
}

static uint8_t init_ioexp(void) {
	// Enable pull-ups on SCL and SDA pins.
	I2C_SCL_PORT |= I2C_SCL_PORT_PIN;
	I2C_SDA_PORT |= I2C_SDA_PORT_PIN;
	
	// Reset the MCP23018.
	IOEXP_RESET_DDR |= IOEXP_RESET_DDR_PIN;  // Enable (low) reset output.
	_delay_ms(0.1);  // Wait a little while.
	IOEXP_RESET_DDR &= ~IOEXP_RESET_DDR_PIN;  // Back to high-Z.
	_delay_ms(0.1);  // Wait a little longer.
	
	sei(); // Ensure that interrupts are enabled. (Required by I2C module.)
	
	i2chelper_mstr_init(TWBR_INIT, SCHED_CATFLAG(IOEXP_CTRL_TASK_CAT));  // Initialize I2C module.
	
	// Configure the MCP23018.
	if (mcp23018_write(MCP23018_ADDR, MCP23018_GPPUA, 0xff) != I2C_READY)  // All pull-ups enabled.
		return 0;  // Failure
	
	if (mcp23018_write(MCP23018_ADDR, MCP23018_IODIRA, 0x00) != I2C_READY)  // All pins outputs.
		return 0;  // Failure
	
	if (mcp23018_write(MCP23018_ADDR, MCP23018_GPIOA, 0x00) != I2C_READY)  // All pins low.
		return 0;  // Failure
	
	if (mcp23018_write(MCP23018_ADDR, MCP23018_GPPUB, 0xff) != I2C_READY)  // All pull-ups enabled.
		return 0;  // Failure
	
	if (mcp23018_write(MCP23018_ADDR, MCP23018_IODIRB, 0x00) != I2C_READY)  // All pins outputs.
		return 0;  // Failure
	
	if (mcp23018_write(MCP23018_ADDR, MCP23018_GPIOB, 0x00) != I2C_READY)  // All pins low.
		return 0;  // Failure
	
	return 1;  // Success!
}

static void init_tasks(void) {
	sched_task task;
	
	task = (sched_task) {
		.st = TASK_ST_MAKE(0, LCD_DRIVE_TASK_CAT, 1),
		.delay = SCHED_TIME_ZERO,
		.handler = lcd_drive_handler
	};
	sched_add(&task);
	
	task = (sched_task) {
		.st = TASK_ST_MAKE(0, ADC_INPUT_TASK_CAT, 0),
		.delay = SCHED_TIME_MS(20),
		.handler = adc_input_handler
	};
	sched_add(&task);
	
	task = (sched_task) {
		.st = TASK_ST_MAKE(0, IOEXP_CTRL_TASK_CAT, 1),
		.delay = SCHED_TIME_ZERO,
		.handler = ioexp_ctrl_handler
	};
	sched_add(&task);
}

// NOTE: This is the entry point, tell compiler not to save/restore registers.
int main(void) __attribute__ ((OS_main));

int main(void) {
	// NOTE: The Arduino Leonardo bootloader leaves some USB interrupts enabled, which
	// causes trouble when you're not using the Arduino application framework.
	disable_usb();
	
	init_pwm_generator();  // Initialize the PWM generator.
	
	// Other I/O initialization.
	EN_DRIVE_PORT |= EN_DRIVE_PORT_PIN;  // Enable pull-up on drive-enable input pin.
	EN_BLITE_PORT |= EN_BLITE_PORT_PIN;  // Enable pull-up on backlight-enable input pin.
	LED_OK_PORT |= LED_OK_PORT_PIN;  // Success indicator output is active-low, start in off state.
	LED_ERR_PORT |= LED_ERR_PORT_PIN;  // Error indicator output is active-low, start in off state.
	
	BLITE_DDR |= BLITE_DDR_PIN;  // Make backlight control pin an output.
	LED_OK_DDR |= LED_OK_DDR_PIN;  // Make success indicator LED pin an output.
	LED_ERR_DDR |= LED_ERR_DDR_PIN;  // Make error indicator LED pin an output.
	
	init_adc();  // Initialize the ADC. (For pot/sensor input.)
	
	sched_init();  // Initialize the task scheduler.
	
	// Configure the input debouncer module.
	TBOUNCER_INIT(
		TASK_ST_MAKE(0, TBOUNCER_TASK_CAT, 0), SCHED_TIME_MS(TBOUNCER_DELAY_MS),
		0, 0, 0, EN_BLITE_PIN_PIN | EN_DRIVE_PIN_PIN,
		0, TASK_ST_CAT_MASK, TASK_ST_CAT(LCD_DRIVE_TASK_CAT));
	
	init_tasks();  // Schedule tasks.
	
	// Initialize the MCP23018 I/O expander.
	if (!init_ioexp()) {  // Initialization failed.
		LED_ERR_PORT &= ~LED_ERR_PORT_PIN;  // Signal the error.
		return 1;  // Die.
	}
	
	LED_OK_PORT &= ~LED_OK_PORT_PIN;  // Signal successful initialization.
	
	sched_run();  // Enter the task scheduler's main loop.
	
	return 0;
}
