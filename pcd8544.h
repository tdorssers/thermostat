/*
 * 48x84 LCD Driver
 *
 * Created: 12/10/2024 16:00:39
 *  Author: Tim Dorssers
 */ 


#ifndef PCD8544_H_
#define PCD8544_H_

#include <stdbool.h>

#define PCD8544_POWERUP 0x00
#define PCD8544_POWERDOWN 0x04

#define PCD8544_BASICINSTRUCTION 0x00
#define PCD8544_EXTENDEDINSTRUCTION 0x01

#define PCD8544_HORIZONTALADDRESS 0x00
#define PCD8544_VERTICALADDRESS 0x02

// H = 0
#define PCD8544_FUNCTIONSET 0x20
#define PCD8544_DISPLAYCONTROL 0x08
#define PCD8544_SETYADDR 0x40
#define PCD8544_SETXADDR 0x80

// H = 1
#define PCD8544_SETTEMP 0x04
#define PCD8544_SETBIAS 0x10
#define PCD8544_SETVOP 0x80

typedef enum {
	PCD8544_DISPLAYBLANK = 0,
	PCD8544_DISPLAYNORMAL = 4,
	PCD8544_DISPLAYALLON = 1,
	PCD8544_DISPLAYINVERTED = 5
} pcd8544_display_t;

#define pcd8544_write_string_P(__str, __sca) pcd8544_write_string_p(PSTR(__str), __sca)
#define pcd8544_led_off() PORTD &= ~_BV(PD6)  // Disable BL
#define pcd8544_led_on() PORTD |= _BV(PD6)    // Enable BL

extern void pcd8544_init(void);
extern void pcd8544_clear(void);
extern void pcd8544_power(bool on);
extern void pcd8544_contrast(uint8_t level);
extern void pcd8544_display_mode(pcd8544_display_t mode);
extern void pcd8544_set_pixel(uint8_t x, uint8_t y, uint8_t value);
extern void pcd8544_write_char(char code, uint8_t scale);
extern void pcd8544_write_string(char *str, uint8_t scale);
extern void pcd8544_write_string_p(const char *str, uint8_t scale);
extern void pcd8544_set_cursor(uint8_t x, uint8_t y);
extern void pcd8544_update(void);
extern void pcd8544_draw_hline(uint8_t x, uint8_t y, uint8_t length);
extern void pcd8544_draw_vline(uint8_t x, uint8_t y, uint8_t length);
extern void pcd8544_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
extern void pcd8544_draw_rect(uint8_t x1, uint8_t y1, uint8_t width, uint8_t height);
extern void pcd8544_fill_rect(uint8_t x1, uint8_t y1, uint8_t width, uint8_t height);
extern void pcd8544_draw_circle(uint8_t x1, uint8_t y1, uint8_t r);
extern void pcd8544_fill_circle(uint8_t x1, uint8_t y1, uint8_t r);

#endif /* PCD8544_H_ */