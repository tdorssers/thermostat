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

const uint8_t CHARSET[][5] PROGMEM = {
	{ 0x00, 0x00, 0x00, 0x00, 0x00 }, // 20 space
	{ 0x00, 0x00, 0x5f, 0x00, 0x00 }, // 21 !
	{ 0x00, 0x07, 0x00, 0x07, 0x00 }, // 22 "
	{ 0x14, 0x7f, 0x14, 0x7f, 0x14 }, // 23 #
	{ 0x24, 0x2a, 0x7f, 0x2a, 0x12 }, // 24 $
	{ 0x23, 0x13, 0x08, 0x64, 0x62 }, // 25 %
	{ 0x36, 0x49, 0x55, 0x22, 0x50 }, // 26 &
	{ 0x00, 0x05, 0x03, 0x00, 0x00 }, // 27 '
	{ 0x00, 0x1c, 0x22, 0x41, 0x00 }, // 28 (
	{ 0x00, 0x41, 0x22, 0x1c, 0x00 }, // 29 )
	{ 0x14, 0x08, 0x3e, 0x08, 0x14 }, // 2a *
	{ 0x08, 0x08, 0x3e, 0x08, 0x08 }, // 2b +
	{ 0x00, 0x50, 0x30, 0x00, 0x00 }, // 2c ,
	{ 0x08, 0x08, 0x08, 0x08, 0x08 }, // 2d -
	{ 0x00, 0x60, 0x60, 0x00, 0x00 }, // 2e .
	{ 0x20, 0x10, 0x08, 0x04, 0x02 }, // 2f /
	{ 0x3e, 0x51, 0x49, 0x45, 0x3e }, // 30 0
	{ 0x00, 0x42, 0x7f, 0x40, 0x00 }, // 31 1
	{ 0x42, 0x61, 0x51, 0x49, 0x46 }, // 32 2
	{ 0x21, 0x41, 0x45, 0x4b, 0x31 }, // 33 3
	{ 0x18, 0x14, 0x12, 0x7f, 0x10 }, // 34 4
	{ 0x27, 0x45, 0x45, 0x45, 0x39 }, // 35 5
	{ 0x3c, 0x4a, 0x49, 0x49, 0x30 }, // 36 6
	{ 0x01, 0x71, 0x09, 0x05, 0x03 }, // 37 7
	{ 0x36, 0x49, 0x49, 0x49, 0x36 }, // 38 8
	{ 0x06, 0x49, 0x49, 0x29, 0x1e }, // 39 9
	{ 0x00, 0x36, 0x36, 0x00, 0x00 }, // 3a :
	{ 0x00, 0x56, 0x36, 0x00, 0x00 }, // 3b ;
	{ 0x08, 0x14, 0x22, 0x41, 0x00 }, // 3c <
	{ 0x14, 0x14, 0x14, 0x14, 0x14 }, // 3d =
	{ 0x00, 0x41, 0x22, 0x14, 0x08 }, // 3e >
	{ 0x02, 0x01, 0x51, 0x09, 0x06 }, // 3f ?
	{ 0x32, 0x49, 0x79, 0x41, 0x3e }, // 40 @
	{ 0x7e, 0x11, 0x11, 0x11, 0x7e }, // 41 A
	{ 0x7f, 0x49, 0x49, 0x49, 0x36 }, // 42 B
	{ 0x3e, 0x41, 0x41, 0x41, 0x22 }, // 43 C
	{ 0x7f, 0x41, 0x41, 0x22, 0x1c }, // 44 D
	{ 0x7f, 0x49, 0x49, 0x49, 0x41 }, // 45 E
	{ 0x7f, 0x09, 0x09, 0x09, 0x01 }, // 46 F
	{ 0x3e, 0x41, 0x49, 0x49, 0x7a }, // 47 G
	{ 0x7f, 0x08, 0x08, 0x08, 0x7f }, // 48 H
	{ 0x00, 0x41, 0x7f, 0x41, 0x00 }, // 49 I
	{ 0x20, 0x40, 0x41, 0x3f, 0x01 }, // 4a J
	{ 0x7f, 0x08, 0x14, 0x22, 0x41 }, // 4b K
	{ 0x7f, 0x40, 0x40, 0x40, 0x40 }, // 4c L
	{ 0x7f, 0x02, 0x0c, 0x02, 0x7f }, // 4d M
	{ 0x7f, 0x04, 0x08, 0x10, 0x7f }, // 4e N
	{ 0x3e, 0x41, 0x41, 0x41, 0x3e }, // 4f O
	{ 0x7f, 0x09, 0x09, 0x09, 0x06 }, // 50 P
	{ 0x3e, 0x41, 0x51, 0x21, 0x5e }, // 51 Q
	{ 0x7f, 0x09, 0x19, 0x29, 0x46 }, // 52 R
	{ 0x46, 0x49, 0x49, 0x49, 0x31 }, // 53 S
	{ 0x01, 0x01, 0x7f, 0x01, 0x01 }, // 54 T
	{ 0x3f, 0x40, 0x40, 0x40, 0x3f }, // 55 U
	{ 0x1f, 0x20, 0x40, 0x20, 0x1f }, // 56 V
	{ 0x3f, 0x40, 0x38, 0x40, 0x3f }, // 57 W
	{ 0x63, 0x14, 0x08, 0x14, 0x63 }, // 58 X
	{ 0x07, 0x08, 0x70, 0x08, 0x07 }, // 59 Y
	{ 0x61, 0x51, 0x49, 0x45, 0x43 }, // 5a Z
	{ 0x00, 0x7f, 0x41, 0x41, 0x00 }, // 5b [
	{ 0x02, 0x04, 0x08, 0x10, 0x20 }, // 5c backslash
	{ 0x00, 0x41, 0x41, 0x7f, 0x00 }, // 5d ]
	{ 0x04, 0x02, 0x01, 0x02, 0x04 }, // 5e ^
	{ 0x40, 0x40, 0x40, 0x40, 0x40 }, // 5f _
	{ 0x00, 0x01, 0x02, 0x04, 0x00 }, // 60 `
	{ 0x20, 0x54, 0x54, 0x54, 0x78 }, // 61 a
	{ 0x7f, 0x48, 0x44, 0x44, 0x38 }, // 62 b
	{ 0x38, 0x44, 0x44, 0x44, 0x20 }, // 63 c
	{ 0x38, 0x44, 0x44, 0x48, 0x7f }, // 64 d
	{ 0x38, 0x54, 0x54, 0x54, 0x18 }, // 65 e
	{ 0x08, 0x7e, 0x09, 0x01, 0x02 }, // 66 f
	{ 0x0c, 0x52, 0x52, 0x52, 0x3e }, // 67 g
	{ 0x7f, 0x08, 0x04, 0x04, 0x78 }, // 68 h
	{ 0x00, 0x44, 0x7d, 0x40, 0x00 }, // 69 i
	{ 0x20, 0x40, 0x44, 0x3d, 0x00 }, // 6a j
	{ 0x7f, 0x10, 0x28, 0x44, 0x00 }, // 6b k
	{ 0x00, 0x41, 0x7f, 0x40, 0x00 }, // 6c l
	{ 0x7c, 0x04, 0x18, 0x04, 0x78 }, // 6d m
	{ 0x7c, 0x08, 0x04, 0x04, 0x78 }, // 6e n
	{ 0x38, 0x44, 0x44, 0x44, 0x38 }, // 6f o
	{ 0x7c, 0x14, 0x14, 0x14, 0x08 }, // 70 p
	{ 0x08, 0x14, 0x14, 0x18, 0x7c }, // 71 q
	{ 0x7c, 0x08, 0x04, 0x04, 0x08 }, // 72 r
	{ 0x48, 0x54, 0x54, 0x54, 0x20 }, // 73 s
	{ 0x04, 0x3f, 0x44, 0x40, 0x20 }, // 74 t
	{ 0x3c, 0x40, 0x40, 0x20, 0x7c }, // 75 u
	{ 0x1c, 0x20, 0x40, 0x20, 0x1c }, // 76 v
	{ 0x3c, 0x40, 0x30, 0x40, 0x3c }, // 77 w
	{ 0x44, 0x28, 0x10, 0x28, 0x44 }, // 78 x
	{ 0x0c, 0x50, 0x50, 0x50, 0x3c }, // 79 y
	{ 0x44, 0x64, 0x54, 0x4c, 0x44 }, // 7a z
	{ 0x00, 0x08, 0x36, 0x41, 0x00 }, // 7b {
	{ 0x00, 0x00, 0x7f, 0x00, 0x00 }, // 7c |
	{ 0x00, 0x41, 0x36, 0x08, 0x00 }, // 7d }
	{ 0x10, 0x08, 0x08, 0x10, 0x08 }, // 7e ~
	{ 0x06, 0x09, 0x09, 0x09, 0x06 }  // 7f degree
};

/* screen buffer */
uint8_t screen[504];

/* cursor position */
uint8_t cursor_x;
uint8_t cursor_y;

inline void start_data(void) {
	PORTD &= ~_BV(PD7);  // chip enable - active low
	PORTD |= 1 << PD5;     // data (active high)
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
	write_cmd(PCD8544_SETBIAS | 0x3);
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

void pcd8544_write_char(char code, uint8_t scale) {
	uint8_t x, y = 0, inv = scale >> 7;
	scale &= 0x7f;
	if (code == '\n') {
		cursor_x = 0;
		cursor_y += 7 * scale + 1;
	} else {
		for (x = 0; x < 5 * scale; x++)
			for (y = 0; y < 7 * scale; y++) {
				uint8_t value = pgm_read_byte(&CHARSET[code - 32][x / scale]);
				pcd8544_set_pixel(cursor_x + x, cursor_y + y, (inv ? ~value : value) & (1 << y / scale));
			}
		if (inv) {
			pcd8544_draw_hline(cursor_x, cursor_y + y, x + 1);
			pcd8544_draw_vline(cursor_x + x, cursor_y, y + 1);
		}
		cursor_x += x + 1;
		if (cursor_x >= 84) {
			cursor_x = 0;
			cursor_y += y + 1;
		}
	}
	if (cursor_y >= 48) {
		cursor_x = 0;
		cursor_y = 0;
	}
}

void pcd8544_write_string(char *str, uint8_t scale) {
	while(*str)
		pcd8544_write_char(*str++, scale);
}

void pcd8544_write_string_p(const char *str, uint8_t scale) {
	char c;
	while ((c = pgm_read_byte(str++)))
		pcd8544_write_char(c, scale);
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
