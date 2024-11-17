/*
 * 48x84 LCD Driver
 *
 * Created: 12/10/2024 16:00:21
 *  Author: Tim Dorssers
 *
 * Wiring:
 * PB3 -> DIN
 * PB5 -> CLK
 * PD3 -> VCC
 * PD4 -> RST
 * PD5 -> D/C
 * PD6 -> LED
 * PD7 -> SCE
 */ 

#define F_CPU 8000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/sfr_defs.h>
#include <stdlib.h>
#include <string.h>
#include "pcd8544.h"

/* current font pointer */
const uint8_t *font;

/* screen buffer */
uint8_t screen[504];

/* cursor position */
uint8_t cursor_x;
uint8_t cursor_y;

inline void start_data(void) {
	PORTD &= ~_BV(PD7);  // chip enable - active low
	PORTD |= _BV(PD5);   // data (active high)
}

inline void end_data(void) {
	PORTD |= _BV(PD7);   // chip disable - idle high
}

inline void write_data(uint8_t data) {
	SPDR = data;                         // transmitting data
	loop_until_bit_is_set(SPSR, SPIF);   // wait till data transmit
}

static void write_cmd(uint8_t cmd) {
	PORTD &= ~(_BV(PD7) | _BV(PD5)); // CE chip enable, DC command (active low)
	write_data(cmd);
	end_data();
}

void pcd8544_init(void) {
	/* Set pins as output */
	DDRB |= _BV(PB2) | _BV(PB3) | _BV(PB5);
	DDRD |= _BV(PD3) | _BV(PD4) | _BV(PD5) | _BV(PD6) | _BV(PD7);
	/* Enable VCC */
	PORTD |= _BV(PD3);
	/* SPI Enable, Master device, Prescaler 4 */
	SPCR |= _BV(SPE) | _BV(MSTR);
	/* Reset display */
	PORTD |= _BV(PD4) | _BV(PD7);  // reset high, chip enable high
	_delay_ms(1);
	PORTD &= ~_BV(PD4); // reset low
	_delay_ms(1);
	PORTD |= _BV(PD4);  // reset high
	/* -LCD Extended Commands mode- */
	write_cmd(PCD8544_FUNCTIONSET | PCD8544_EXTENDEDINSTRUCTION);
	/* LCD bias mode 1:48 */
	write_cmd(PCD8544_SETBIAS | 0x4);
	/* Set temperature coefficient */
	write_cmd(PCD8544_SETTEMP | 0x2);
	/* Default VOP (3.06 + 60 * 0.06 = 6.66V) */
	write_cmd(PCD8544_SETVOP | 0x3c);
	/* Standard Commands mode, Horizontal addressing mode */
	write_cmd(PCD8544_FUNCTIONSET | PCD8544_BASICINSTRUCTION | PCD8544_HORIZONTALADDRESS);
	/* LCD in normal mode */
	write_cmd(PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYNORMAL);
}

void pcd8544_clear(void) {
	cursor_x = 0;
	cursor_y = 0;
	memset(screen, 0, sizeof(screen));
}

void pcd8544_power(bool on) {
	write_cmd(PCD8544_FUNCTIONSET | (on ? PCD8544_POWERUP : PCD8544_POWERDOWN));
}

void pcd8544_contrast(uint8_t level) {
	write_cmd(PCD8544_FUNCTIONSET | PCD8544_EXTENDEDINSTRUCTION);
	write_cmd(PCD8544_SETVOP | (level & 0x7f));
	write_cmd(PCD8544_FUNCTIONSET | PCD8544_BASICINSTRUCTION | PCD8544_HORIZONTALADDRESS);
}

void pcd8544_display_mode(pcd8544_display_t mode) {
	write_cmd(PCD8544_DISPLAYCONTROL | mode);
}

void pcd8544_set_pixel(uint8_t x, uint8_t y, uint8_t value) {
	uint16_t p = y / 8 * 84 + x;
	if (p >= sizeof(screen)) return;
	uint8_t bit = (1 << (y % 8));
	if (value)
		screen[p] |= bit;
	else
		screen[p] &= ~bit;
}

void pcd8544_set_font(const uint8_t *f) {
	font = f;
}

void pcd8544_write_char(char code, bool inv) {
	uint8_t x, y = 0;
	const uint8_t *base = font;
	uint8_t width = pgm_read_byte(base++);
	uint8_t height = pgm_read_byte(base++);
	if (code == '\n') {
		cursor_x = 0;
		cursor_y += height + 1;
	} else {
		base += (code - 32) * ((height + 7) / 8) * width;
		for (x = 0; x < width; x++, base++)
			for (y = 0; y < height; y++) {
				uint8_t value = pgm_read_byte(base + (y / 8 * width));
				pcd8544_set_pixel(cursor_x + x, cursor_y + y, (inv ? ~value : value) & _BV(y & 0x7));
			}
		if (inv) {
			pcd8544_draw_hline(cursor_x, cursor_y + height, width + 1);
			pcd8544_draw_vline(cursor_x + width, cursor_y, height + 1);
		}
		cursor_x += width + 1;
		if (cursor_x >= 84) {
			cursor_x = 0;
			cursor_y += height + 1;
		}
	}
	if (cursor_y >= 48) {
		cursor_x = 0;
		cursor_y = 0;
	}
}

void pcd8544_write_string(char *str, bool inv) {
	while(*str)
		pcd8544_write_char(*str++, inv);
}

void pcd8544_write_string_p(const char *str, bool inv) {
	char c;
	while ((c = pgm_read_byte(str++)))
		pcd8544_write_char(c, inv);
}

void pcd8544_set_cursor(uint8_t x, uint8_t y) {
	cursor_x = x;
	cursor_y = y;
}

void pcd8544_update(void) {
	/* Set column and row to 0 */
	write_cmd(PCD8544_SETXADDR);
	write_cmd(PCD8544_SETYADDR);
	/* Write screen to display */
	start_data();
	for (uint16_t i = 0; i < sizeof(screen); i++)
		write_data(screen[i]);
	end_data();
}

void pcd8544_draw_hline(uint8_t x, uint8_t y, uint8_t length) {
	for (; length; length--)
		pcd8544_set_pixel(x++, y, 1);
}

void pcd8544_draw_vline(uint8_t x, uint8_t y, uint8_t length) {
	for (; length; length--)
		pcd8544_set_pixel(x, y++, 1);
}

void pcd8544_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2) {
	int8_t x = x2-x1, y = y2-y1;
	int8_t dx = abs(x), dy = -abs(y);
	int8_t sx = x1<x2 ? 1 : -1;
	int8_t sy = y1<y2 ? 1 : -1;
	int8_t err = dx+dy, e2;			// error value e_xy
	for (;;) {
		pcd8544_set_pixel(x1,y1,1);
		e2 = 2*err;
		if (e2 >= dy) {			// e_xy+e_x > 0
			if (x1 == x2) break;
			err += dy;
			x1 += sx;
		}
		if (e2 <= dx) {	 		// e_xy+e_y < 0
			if (y1 == y2) break;
			err += dx;
			y1 += sy;
		}
	}
}

void pcd8544_draw_rect(uint8_t x1, uint8_t y1, uint8_t width, uint8_t height) {
	pcd8544_draw_hline(x1, y1, width);
	pcd8544_draw_hline(x1, y1+height-1, width);
	pcd8544_draw_vline(x1, y1, height);
	pcd8544_draw_vline(x1+width-1, y1, height);
}

void pcd8544_fill_rect(uint8_t x1, uint8_t y1, uint8_t width, uint8_t height) {
	for (uint8_t x = x1; x <= x1 + width - 1; x++)
		pcd8544_draw_vline(x, y1, height);
}

void pcd8544_draw_circle(uint8_t x1, uint8_t y1, uint8_t r) {
	int8_t x = -r, y = 0, err = 2-2*r, e2;
	do {
		pcd8544_set_pixel(x1-x, y1+y,1);
		pcd8544_set_pixel(x1+x, y1+y,1);
		pcd8544_set_pixel(x1+x, y1-y,1);
		pcd8544_set_pixel(x1-x, y1-y,1);
		e2 = err;
		if (e2 <= y) {
			err += ++y*2+1;
			if (-x == y && e2 <= x) e2 = 0;
		}
		if (e2 > x)
			err += ++x*2+1;
	} while (x <= 0);
}

void pcd8544_fill_circle(uint8_t x1, uint8_t y1, uint8_t r) {
	int8_t x = -r, y = 0, err = 2-2*r, e2;
	do {
		pcd8544_draw_vline(x1-x, y1-y, 2*y);
		pcd8544_draw_vline(x1+x, y1-y, 2*y);
		e2 = err;
		if (e2 <= y) {
			err += ++y*2+1;
			if (-x == y && e2 <= x) e2 = 0;
		}
		if (e2 > x)
			err += ++x*2+1;
	} while (x <= 0);
}
