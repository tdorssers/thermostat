/*
 * Title   : 2 Channel AC Dimming Thermostat and Timer
 * Hardware: ATmega328P @ 8 MHz, PCD8544 LCD controller,
  *          AM2320 temperature and humidity sensor
 *
 * Created: 06/10/2024 15:21:25
 *  Author: Tim Dorssers
 *
 * This is a graphical menu configured dimming thermostat with two output
 * channels. Each channel can also switch between on and off. The start of
 * daytime and length of day are used to determine day and night temperatures.
 * The dimming hardware uses zero-cross detection which gives a positive edge
 * at the end of a half sine wave and a negative edge at the start of a half
 * sine wave on the ICP1 pin. The OC1x pins connect to photo-TRIACs that drive
 * the power TRIACs to control the leading edge.
 *
 *    __ + -    + - __ + - ICP1     H_  L  H   L  H_  L  H   L OC1x
 *   /  \| |    | |/  \| |          | \           | \
 *  /    | |    | |    | |          |  \          |  \
 * +------+------+------+------+  ==+---+==+---+==+---+==+---+
 *       | |    | |    | |    /            |  /          |  /
 *       | |\__/| |    | |\__/             |_/           |_/
 */ 

#define F_CPU 8000000

#include <avr/io.h>
#include <avr/cpufunc.h>
#include <avr/pgmspace.h>
#include <avr/sfr_defs.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "uart.h"
#include "pcd8544.h"
#include "am2320.h"

char buffer[15];
uint8_t EEMEM nv_magic;
uint8_t ch0_dim = 0, ch1_dim = 0;
bool ch0_auto = true, ch1_auto = true;
bool ch0_on_off = false, ch1_on_off = false;
uint8_t EEMEM nv_ch0_dim, nv_ch1_dim;
uint8_t EEMEM nv_ch0_auto, nv_ch1_auto;
uint8_t EEMEM nv_ch0_on_off, nv_ch1_on_off;
#define DIM_STEPS 50
uint16_t next_ocr1a = 0, next_ocr1b = 0;
volatile bool blink = false;
volatile bool button[4];
volatile uint8_t bl_delay = 0;
#define BL_DELAY 30
enum {HOME, SETUP, CHANNEL, KVAL, ETC};
uint16_t humidity;
int16_t temperature;
uint8_t sensor = 0;
uint8_t time_sec = 0, time_min = 0, time_hour = 0;
uint8_t start_min = 0, start_hour = 8, length_min = 0, length_hour = 10;
uint8_t EEMEM nv_start_min, nv_start_hour, nv_length_min, nv_length_hour;
int16_t min_temp = 200, max_temp = 250;
uint16_t EEMEM nv_min_temp, nv_max_temp;
const char str_auto[] PROGMEM = "Auto";
const char str_on_off[] PROGMEM = "On/Off";
const char str_dimming[] PROGMEM = "Dimming";
const char str_buttons[] PROGMEM = "Back Sel Up Dn";
#define P_ON_M
uint8_t Kp = 90, Ki = 1, Kd = 10, dT = 2;
uint8_t EEMEM nvKp, nvKi, nvKd, nvdT;
volatile uint8_t sample_delay = 0, on_off_delay = 0;
#define ON_OFF_DELAY 30
uint8_t on_off_thres = 15;
uint8_t EEMEM nv_on_off_thres;
enum {OFF, ON, AUTO};
uint8_t bl_mode = AUTO, contrast = 60;
uint8_t EEMEM nv_bl_mode, nv_contrast;

#define button_init() PORTC |= _BV(PC0) | _BV(PC1) | _BV(PC2) | _BV(PC3)
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

static void timer0_init(void) {
	TCCR0A = 0;           // Normal operation
	TCCR0B = _BV(CS00) | _BV(CS01);  // Prescaler /64, TC0 overflows every 2048 us
	TCNT0 = 0;            // Reset timer
	TIMSK0 = _BV(TOIE0);  // Timer0 Overflow Interrupt Enable
}

ISR(TIMER0_OVF_vect) {
	static uint8_t count = 0, push[4];  // These overflow every 524 ms
	static bool hold[4];
	if (++count == 0) blink = !blink;
	for (uint8_t pin = PC0; pin < PC4; pin++) {
		if bit_is_clear(PINC, pin) {
			push[pin]++;
			if (push[pin] == 0) hold[pin] = true;
		} else {
			push[pin] = 0;
			hold[pin] = false;
		}
		// Signal button push after 20,5 ms or every 262 ms when pushed longer then 524 ms
		if (push[pin] == 10 || (hold[pin] && push[pin] == 138)) {
			if (bl_delay || bl_mode != AUTO) button[pin] = true;
			bl_delay = BL_DELAY;
		}
	}
}

static void timer2_init(void) {
	ASSR |= _BV(AS2);     // Set Timer/Counter2 clock source to 32,768 kHz crystal
	TCNT2 = 0;            // Reset timer
	TCCR2B = _BV(CS20) | _BV(CS22);  // Make TC2 overflow precisely once every second
	while (ASSR & (_BV(OCR2AUB) | _BV(TCN2UB) | _BV(TCR2AUB)));  // Wait until TC2 is updated
	TIMSK2 = _BV(TOIE2);  // Enable Timer/Counter2 Overflow Interrupts
	sei();                // Set the Global Interrupt Enable Bit
}

ISR(TIMER2_OVF_vect) {
	// Time keeping
	if (++time_sec > 59) {
		time_sec = 0;
		if (++time_min > 59) {
			time_min = 0;
			if (++time_hour > 23) time_hour = 0;
		}
	}
	// Decrement counters
	if (bl_delay) bl_delay--;
	if (sample_delay) sample_delay--;
	if (on_off_delay) on_off_delay--;
}

static void calibrate(void) {
	uint8_t cycles = 128;
	ASSR |= _BV(AS2);
	TCCR2B = _BV(CS20);  // Enable TC2 without prescaler
	while (ASSR & (_BV(OCR2AUB) | _BV(TCN2UB) | _BV(TCR2AUB)));
	do {
		// Clear interrupt flags
		TIFR1 = TIFR2 = 0xFF;
		// Reset timers
		TCNT1 = TCNT2 = 0;
		while (ASSR & (_BV(OCR2AUB) | _BV(TCN2UB) | _BV(TCR2AUB)));
		// Start Timer/Counter1
		TCCR1B = _BV(CS10);
		// Stop timer on compare match
		loop_until_bit_is_set(TIFR2, OCF2A);
		TCCR1B = 0;
		// Adjust calibration value
		if (TCNT1 > (F_CPU / 32768) * 256 + 128 || bit_is_set(TIFR1, TOV1)) {
			OSCCAL--;
			_NOP();
		} else if (TCNT1 < (F_CPU / 32768) * 256 - 128) {
			OSCCAL++;
			_NOP();
		} else
			break;
	} while(--cycles);
}

static void timer1_init(void) {
	TCCR1A = 0; // Normal operation
	// Input capture noise cancel, positive edge, /8 prescaler
	TCCR1B = _BV(ICNC1) | _BV(ICES1) | _BV(CS11);
	TIFR1 = 0xFF; // Clear interrupt flags
	TCNT1 = 0; // Reset timer
	// Enable input capture and compare match interrupts
	TIMSK1 = _BV(OCIE1A) | _BV(OCIE1B) | _BV(ICIE1);
	DDRB |= _BV(PB1) | _BV(PB2); // Set OC1A and OC1B as outputs
}

ISR(TIMER1_CAPT_vect) {
	static uint16_t last_icr1 = 0, half_zero = 0, dim_period = 0;
	if bit_is_set(TCCR1B, ICES1) { // Positive edge: end of half sine
		dim_period = (ICR1 - last_icr1) / DIM_STEPS;
		uint16_t crossing = ICR1 + half_zero;
		// Determine when the TRIACs are to be triggered
		OCR1A = crossing + dim_period * (DIM_STEPS - ch0_dim);
		OCR1B = crossing + dim_period * (DIM_STEPS - ch1_dim);
		next_ocr1a = dim_period * ch0_dim;
		if (ch0_dim == DIM_STEPS) next_ocr1a += half_zero;
		next_ocr1b = dim_period * ch1_dim;
		if (ch1_dim == DIM_STEPS) next_ocr1b += half_zero;
		// Set OC1x on compare match, only if enabled
		TCCR1A = (ch0_dim ? _BV(COM1A0) | _BV(COM1A1) : 0) | (ch1_dim ? _BV(COM1B0) | _BV(COM1B1) : 0);
		TIFR1 = 0xFF; // Clear interrupt flags
	} else { // Negative edge: begin of half sine
		half_zero = (ICR1 - last_icr1) / 2;
	}
	TCCR1B ^= _BV(ICES1); // Toggle edge trigger
	last_icr1 = ICR1;
}

ISR(TIMER1_COMPA_vect) {
	TCCR1A &= ~_BV(COM1A0); // Clear OC1A on compare match
	OCR1A += next_ocr1a;
}

ISR(TIMER1_COMPB_vect) {
	TCCR1A &= ~_BV(COM1B0); // Clear OC1B on compare match
	OCR1B += next_ocr1b;
}

// Convert (scaled) integer to (zero filled) string
static char *itostr(int16_t num, char *str, uint8_t decimal, uint8_t padding) {
	uint8_t i = 0;
	uint16_t sum = abs(num);
	if (decimal) padding++;
	do {
		str[i++] = '0' + sum % 10;
		if (i == decimal) str[i++] = '.';
	} while ((sum /= 10) || i < decimal);
	while (i < padding) str[i++] = '0';
	if (num < 0) str[i++] = '-';
	str[i] = '\0';
	return strrev(str);
}

static bool is_daytime(void) {
	uint16_t now = time_hour * 60 + time_min;
	uint16_t start = start_hour * 60 + start_min;
	return now >= start && now < start + length_hour * 60 + length_min;
}

static uint8_t home(void) {
	for (uint8_t i = 0; i < 4; i++) {
		if (button[i]) {
			button[i] = false;
			return 4 - i;
		}
	}
	pcd8544_clear();
	pcd8544_write_string(itostr(ch0_dim, buffer, 0, 1), 1);
	pcd8544_write_char('/', 1);
	pcd8544_write_string(itostr(ch1_dim, buffer, 0, 1), 1);
	if (is_daytime()) {
		pcd8544_set_cursor(42, 0);
		pcd8544_write_char('*', 1);
	}
	pcd8544_set_cursor(54, 0);
	itostr(time_hour, buffer, 0, 2);
	buffer[2] = time_sec % 2 ? ':' : ' ';
	itostr(time_min, &buffer[3], 0, 2);
	pcd8544_write_string(buffer, 1);
	switch (sensor) {
		case 0:
			pcd8544_set_cursor(temperature < 0 ? 0 : 12, 8);
			pcd8544_write_string(itostr(temperature, buffer, 1, 2), 2);
			pcd8544_write_string_P("\x7f\x43", 2);
			pcd8544_set_cursor(12, 24);
			pcd8544_write_string(itostr(humidity, buffer, 1, 2), 2);
			pcd8544_write_char('%', 2);
			break;
		case 1:
			pcd8544_write_string_P("\nNo response", 1);
			break;
		case 2:
			pcd8544_write_string_P("\nCRC error", 1);
	}
	pcd8544_set_cursor(0, 40);
	pcd8544_write_string_P("Set Ch Pid Lcd", 1);
	pcd8544_update();
	return HOME;
}

static void eeprom_save(void) {
	eeprom_update_byte(&nv_magic, 0x55);
	eeprom_update_byte(&nv_ch0_auto, ch0_auto);
	eeprom_update_byte(&nv_ch0_dim, ch0_dim);
	eeprom_update_byte(&nv_ch0_on_off, ch0_on_off);
	eeprom_update_byte(&nv_ch1_auto, ch1_auto);
	eeprom_update_byte(&nv_ch1_dim, ch1_dim);
	eeprom_update_byte(&nv_ch1_on_off, ch1_on_off);
	eeprom_update_byte(&nv_start_hour, start_hour);
	eeprom_update_byte(&nv_start_min, start_min);
	eeprom_update_byte(&nv_length_hour, length_hour);
	eeprom_update_byte(&nv_length_min, length_min);
	eeprom_update_word(&nv_min_temp, min_temp);
	eeprom_update_word(&nv_max_temp, max_temp);
	eeprom_update_byte(&nvKp, Kp);
	eeprom_update_byte(&nvKi, Ki);
	eeprom_update_byte(&nvKd, Kd);
	eeprom_update_byte(&nvdT, dT);
	eeprom_update_byte(&nv_bl_mode, bl_mode);
	eeprom_update_byte(&nv_contrast, contrast);
	eeprom_update_byte(&nv_on_off_thres, on_off_thres);
}

static void blink_buffer(void) {
	if (blink) memset(buffer, ' ', strlen(buffer));
}

static uint8_t setup(void) {
	static uint8_t item = 1, select = 0, sub = 0;
	if (button[3]) {  // Back
		button[3] = false;
		if (select) {
			select = 0;
		} else {
			eeprom_save();
			return HOME;
		}
	}
	if (button[2]) {  // Select
		button[2] = false;
		switch (select) {
			case 0:
				select = item;
				sub = 1;
				break;
			case 1:
				if (++sub > 3) select = 0;
				break;
			case 2:
			case 3:
				if (++sub > 2) select = 0;
				break;
			case 4:
			case 5:
				select = 0;
		}
	}
	if (button[1]) {  // Up
		button[1] = false;
		switch (select) {
			case 0:
				if (--item == 0) item = 5;
				break;
			case 1:
				if (sub == 1) if (++time_hour > 23) time_hour = 0;
				if (sub == 2) if (++time_min > 59) time_min = 0;
				if (sub == 3) if (++time_sec > 59) time_sec = 0;
				break;
			case 2:
				if (sub == 1) if (++start_hour > 23) start_hour = 0;
				if (sub == 2) if (++start_min > 59) start_min = 0;
				break;
			case 3:
				if (sub == 1) if (++length_hour > 23) length_hour = 0;
				if (sub == 2) if (++length_min > 59) length_min = 0;
				break;
			case 4:
				if (min_temp < 800) min_temp += 5;
				break;
			case 5:
				if (min_temp < 800) max_temp += 5;
		}
	}
	if (button[0]) {  // Down
		button[0] = false;
		switch (select) {
			case 0:
				if (++item > 5) item = 1;
				break;
			case 1:
				if (sub == 1) if (--time_hour > 23) time_hour = 23;
				if (sub == 2) if (--time_min > 59) time_min = 59;
				if (sub == 3) if (--time_sec > 59) time_sec = 59;
				break;
			case 2:
				if (sub == 1) if (--start_hour > 23) start_hour = 23;
				if (sub == 2) if (--start_min > 59) start_min = 59;
				break;
			case 3:
				if (sub == 1) if (--length_hour > 23) length_hour = 23;
				if (sub == 2) if (--length_min > 59) length_min = 59;
				break;
			case 4:
				if (min_temp > -400) min_temp -= 5;
				break;
			case 5:
				if (min_temp > -400) max_temp -= 5;
		}
	}
	pcd8544_clear();
	uint8_t inv = item == 1 ? 129 : 1;
	pcd8544_write_string_P("Time ", inv);
	memset(buffer, ' ', 8);
	if (!(select == 1 && sub == 1 && blink)) itostr(time_hour, buffer, 0, 2);
	buffer[2] = ':';
	if (!(select == 1 && sub == 2 && blink)) itostr(time_min, &buffer[3], 0, 2);
	buffer[5] = ':';
	if (!(select == 1 && sub == 3 && blink)) itostr(time_sec, &buffer[6], 0, 2);
	buffer[8] = 0;
	pcd8544_write_string(buffer, inv);
	inv = item == 2 ? 129 : 1;
	pcd8544_write_string_P("\nStart ", inv);
	memset(buffer, ' ', 8);
	if (!(select == 2 && sub == 1 && blink)) itostr(start_hour, buffer, 0, 2);
	buffer[2] = ':';
	if (!(select == 2 && sub == 2 && blink)) itostr(start_min, &buffer[3], 0, 2);
	buffer[5] = 0;
	pcd8544_write_string(buffer, inv);
	inv = item == 3 ? 129 : 1;
	pcd8544_write_string_P("\nLength ", inv);
	memset(buffer, ' ', 5);
	if (!(select == 3 && sub == 1 && blink)) itostr(length_hour, buffer, 0, 2);
	buffer[2] = ':';
	if (!(select == 3 && sub == 2 && blink)) itostr(length_min, &buffer[3], 0, 2);
	buffer[5] = 0;
	pcd8544_write_string(buffer, inv);
	inv = item == 4 ? 129 : 1;
	pcd8544_write_string_P("\nMin temp ", inv);
	itostr(min_temp, buffer, 1, 2);
	if (select == 4) blink_buffer();
	pcd8544_write_string(buffer, inv);
	inv = item == 5 ? 129 : 1;
	pcd8544_set_cursor(0, 32);
	pcd8544_write_string_P("Max temp ", inv);
	itostr(max_temp, buffer, 1, 2);
	if (select == 5) blink_buffer();
	pcd8544_write_string(buffer, inv);
	pcd8544_set_cursor(0, 40);
	pcd8544_write_string_p(str_buttons, 1);
	pcd8544_update();
	return SETUP;
}

static uint8_t channel(void) {
	static uint8_t item = 1, select = 0;
	if (button[3]) {  // Back
		button[3] = false;
		if (select) {
			select = 0;
		} else {
			eeprom_save();
			return HOME;
		}
	}
	if (button[2]) {  // Select
		button[2] = false;
		select = select ? 0 : item;
	}
	if (button[1]) {  // Up
		button[1] = false;
		switch (select) {
			case 0:
				if (--item == 0) item = 5;
				break;
			case 1:
				if (ch0_auto) {
					ch0_auto = false;
					ch0_dim = 0;
				} else if (ch0_on_off && ch0_dim == 0) {
					ch0_dim = DIM_STEPS;
				} else if ((ch0_on_off && ch0_dim) || ++ch0_dim > DIM_STEPS) {
					ch0_auto = true;
					ch0_dim = 0;
				}
				break;
			case 2:
				ch0_on_off = !ch0_on_off;
				if (ch0_on_off) ch0_dim = ch0_dim ? DIM_STEPS : 0;
				break;
			case 3:
				if (ch1_auto) {
					ch1_auto = false;
					ch1_dim = 0;
				} else if (ch1_on_off && ch1_dim == 0) {
					ch1_dim = DIM_STEPS;
				} else if ((ch1_on_off && ch1_dim) || ++ch1_dim > DIM_STEPS) {
					ch1_auto = true;
					ch1_dim = 0;
				}
				break;
			case 4:
				ch1_on_off = !ch1_on_off;
				if (ch1_on_off) ch1_dim = ch1_dim ? DIM_STEPS : 0;
				break;
			case 5:
				if (++on_off_thres > DIM_STEPS) on_off_thres = 0;
		}
	}
	if (button[0]) {  // Down
		button[0] = false;
		switch (select) {
			case 0:
				if (++item > 5) item = 1;
				break;
			case 1:
				if (ch0_auto) {
					ch0_auto = false;
					ch0_dim = DIM_STEPS;
				} else if (ch0_on_off && ch0_dim) {
					ch0_dim = 0;
				} else if ((ch0_on_off && ch0_dim == 0) || ch0_dim-- == 0) {
					ch0_auto = true;
					ch0_dim = 0;
				}
				break;
			case 2:
				ch1_on_off = !ch1_on_off;
				if (ch1_on_off) ch1_dim = ch1_dim ? DIM_STEPS : 0;
				break;
			case 3:
				if (ch1_auto) {
					ch1_auto = false;
					ch1_dim = DIM_STEPS;
				} else if (ch1_on_off && ch1_dim) {
					ch1_dim = 0;
				} else if ((ch1_on_off && ch1_dim == 0) || ch1_dim-- == 0) {
					ch1_auto = true;
					ch1_dim = 0;
				}
				break;
			case 4:
				ch1_on_off = !ch1_on_off;
				if (ch1_on_off) ch1_dim = ch1_dim ? DIM_STEPS : 0;
				break;
			case 5:
				if (--on_off_thres > DIM_STEPS) on_off_thres = DIM_STEPS;
		}
	}
	pcd8544_clear();
	uint8_t inv = item == 1 ? 129 : 1;
	pcd8544_write_string_P("Ch 0 ", inv);
	if (ch0_auto)
		strcpy_P(buffer, str_auto);
	else
		itostr(ch0_dim, buffer, 0, 1);
	if (select == 1) blink_buffer();
	pcd8544_write_string(buffer, inv);
	inv = item == 2 ? 129 : 1;
	pcd8544_write_string_P("\nCh 0 ", inv);
	strcpy_P(buffer, ch0_on_off ? str_on_off : str_dimming);
	if (select == 2) blink_buffer();
	pcd8544_write_string(buffer, inv);
	inv = item == 3 ? 129 : 1;
	pcd8544_write_string_P("\nCh 1 ", inv);
	if (ch1_auto)
		strcpy_P(buffer, str_auto);
	else
		itostr(ch1_dim, buffer, 0, 1);
	if (select == 3) blink_buffer();
	pcd8544_write_string(buffer, inv);
	inv = item == 4 ? 129 : 1;
	pcd8544_write_string_P("\nCh 1 ", inv);
	strcpy_P(buffer, ch1_on_off ? str_on_off : str_dimming);
	if (select == 4) blink_buffer();
	pcd8544_write_string(buffer, inv);
	inv = item == 5 ? 129 : 1;
	pcd8544_write_string_P("\nThreshold ", inv);
	itostr(on_off_thres, buffer, 0, 1);
	if (select == 5) blink_buffer();
	pcd8544_write_string(buffer, inv);
	pcd8544_set_cursor(0, 40);
	pcd8544_write_string_p(str_buttons, 1);
	pcd8544_update();
	return CHANNEL;
}

static uint8_t kval(void) {
	static uint8_t item = 1, select = 0;
	if (button[3]) {  // Back
		button[3] = false;
		if (select) {
			select = 0;
		} else {
			eeprom_save();
			return HOME;
		}
	}
	if (button[2]) {  // Select
		button[2] = false;
		select = select ? 0 : item;
	}
	if (button[1]) {  // Up
		button[1] = false;
		switch (select) {
			case 0:
				if (--item == 0) item = 4;
				break;
			case 1:
				Kp++;
				break;
			case 2:
				Ki++;
				break;
			case 3:
				Kd++;
				break;
			case 4:
				if (++dT > 59) dT = 0;
		}
	}
	if (button[0]) {  // Down
		button[0] = false;
		switch (select) {
			case 0:
				if (++item > 4) item = 1;
				break;
			case 1:
				Kp--;
				break;
			case 2:
				Ki--;
				break;
			case 3:
				Kd--;
				break;
			case 4:
				if (--dT > 59) dT = 59;
		}
	}
	pcd8544_clear();
	uint8_t inv = item == 1 ? 129 : 1;
	pcd8544_write_string_P("Kp ", inv);
	itostr(Kp, buffer, 2, 3);
	if (select == 1) blink_buffer();
	pcd8544_write_string(buffer, inv);
	inv = item == 2 ? 129 : 1;
	pcd8544_write_string_P("\nKi ", inv);
	itostr(Ki, buffer, 2, 3);
	if (select == 2) blink_buffer();
	pcd8544_write_string(buffer, inv);
	inv = item == 3 ? 129 : 1;
	pcd8544_write_string_P("\nKd ", inv);
	itostr(Kd, buffer, 2, 3);
	if (select == 3) blink_buffer();
	pcd8544_write_string(buffer, inv);
	inv = item == 4 ? 129 : 1;
	pcd8544_write_string_P("\ndT ", inv);
	itostr(dT, buffer, 0, 1);
	if (select == 4) blink_buffer();
	pcd8544_write_string(buffer, inv);
	pcd8544_write_char('s', inv);
	pcd8544_set_cursor(0, 40);
	pcd8544_write_string_p(str_buttons, 1);
	pcd8544_update();
	return KVAL;
}

static uint8_t etc(void) {
	static uint8_t item = 1, select = 0;
	if (button[3]) {  // Back
		button[3] = false;
		if (select) {
			select = 0;
		} else {
			eeprom_save();
			return HOME;
		}
	}
	if (button[2]) {  // Select
		button[2] = false;
		select = select ? 0 : item;
	}
	if (button[1]) {  // Up
		button[1] = false;
		switch (select) {
			case 0:
				if (--item == 0) item = 2;
				break;
			case 1:
				if (++bl_mode > 2) bl_mode = 0;
				break;
			case 2:
				if (++contrast > 90) contrast = 90;
				pcd8544_contrast(contrast);
		}
	}
	if (button[0]) {  // Down
		button[0] = false;
		switch (select) {
			case 0:
				if (++item > 2) item = 1;
				break;
			case 1:
				if (bl_mode-- == 0) bl_mode = 2;
				break;
			case 2:
				if (--contrast < 30) contrast = 30;
				pcd8544_contrast(contrast);
		}
	}
	pcd8544_clear();
	uint8_t inv = item == 1 ? 129 : 1;
	pcd8544_write_string_P("Backlight ", inv);
	if (bl_mode == ON)
		strcpy_P(buffer, PSTR("On"));
	else if (bl_mode == AUTO)
		strcpy_P(buffer, str_auto);
	else
		strcpy_P(buffer, PSTR("Off"));
	if (select == 1) blink_buffer();
	pcd8544_write_string(buffer, inv);
	inv = item == 2 ? 129 : 1;
	pcd8544_set_cursor(0, 8);
	pcd8544_write_string_P("Contrast ", inv);
	itostr(contrast, buffer, 0, 1);
	if (select == 2) blink_buffer();
	pcd8544_write_string(buffer, inv);
	pcd8544_set_cursor(0, 40);
	pcd8544_write_string_p(str_buttons, 1);
	pcd8544_update();
	return ETC;
}

static int16_t pid(int16_t input, int16_t setpoint) {
	static int16_t lastInput = 0;
	static int16_t outputSum = 0;
	int16_t output = 0;
	int16_t error = setpoint - input;
	int16_t dInput = input - lastInput;
	lastInput = input;
	outputSum += Ki * 2 * error;
#ifdef P_ON_M
	outputSum -= Kp * dInput;  // Proportional on Measurement
#else
	output = Kp * error;  // Proportional on Error
#endif
	outputSum = constrain(outputSum, 0, DIM_STEPS * 100);
	output += outputSum - Kd / 2 * dInput;  // Derivative on Measurement
	output /= 100;
	return constrain(output, 0, DIM_STEPS);
}

static void eeprom_init(void) {
	if (eeprom_read_byte(&nv_magic) != 0x55) return;
	ch0_dim = eeprom_read_byte(&nv_ch0_dim);
	ch1_dim = eeprom_read_byte(&nv_ch1_dim);
	ch0_auto = eeprom_read_byte(&nv_ch0_auto);
	ch1_auto = eeprom_read_byte(&nv_ch1_auto);
	ch0_on_off = eeprom_read_byte(&nv_ch0_on_off);
	ch1_on_off = eeprom_read_byte(&nv_ch1_on_off);
	start_hour = eeprom_read_byte(&nv_start_hour);
	start_min = eeprom_read_byte(&nv_start_min);
	length_hour = eeprom_read_byte(&nv_length_hour);
	length_min = eeprom_read_byte(&nv_length_min);
	min_temp = eeprom_read_word(&nv_min_temp);
	max_temp = eeprom_read_word(&nv_max_temp);
	Kp = eeprom_read_byte(&nvKp);
	Ki = eeprom_read_byte(&nvKi);
	Kd = eeprom_read_byte(&nvKd);
	dT = eeprom_read_byte(&nvdT);
	bl_mode = eeprom_read_byte(&nv_bl_mode);
	contrast = eeprom_read_byte(&nv_contrast);
	on_off_thres = eeprom_read_byte(&nv_on_off_thres);
}

int main(void) {
	uint8_t view = HOME, prev_on_off = 0;
	eeprom_init();
	pcd8544_init();
	pcd8544_led_on();
	pcd8544_write_string_P("Calibrating", 1);
	pcd8544_update();
	calibrate();
	pcd8544_clear();
	pcd8544_update();
    uart_init(UART_BAUD_SELECT(9600, F_CPU));
	uart_puts_P("Ok\r\n");
	button_init();
	timer0_init();
	timer1_init();
	timer2_init();
	// Main loop
    while (1) {
		if (view == HOME) view = home();
		if (view == SETUP) view = setup();
		if (view == CHANNEL) view = channel();
		if (view == KVAL) view = kval();
		if (view == ETC) view = etc();
		if (bl_mode == ON || (bl_mode == AUTO && bl_delay))
			pcd8544_led_on();
		else
			pcd8544_led_off();
		if (sample_delay == 0) {
			sample_delay = dT;
			if ((sensor = am2320_get(&humidity, &temperature))) continue;
			int16_t setpoint = is_daytime() ? max_temp : min_temp;
			uint8_t output = pid(temperature, setpoint);
			uint8_t on_off = output > on_off_thres ? DIM_STEPS : 0;
			if (on_off != prev_on_off) {
				prev_on_off = on_off;
				on_off_delay = ON_OFF_DELAY;
			}
			if (ch0_auto) {
				if (ch0_on_off) {
					if (on_off_delay == 0) ch0_dim = on_off;
				} else {
					ch0_dim = output;
				}
			}
			if (ch1_auto) {
				if (ch1_on_off) {
					if (on_off_delay == 0) ch1_dim = on_off;
				} else {
					ch1_dim = output;
				}
			}
		}
    }
}

