/*
 * I2C (TWI) Master Software Library
 *
 * Created: 10/11/2024 10:29:35
 *  Author: Tim Dorssers
 */ 

#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>
#include "i2c.h"

#define SCL_CLOCK 100000UL

#define SDA     0           // SDA Port D, Pin 0
#define SCL     1           // SCL Port D, Pin 1
#define SDA_DDR DDRD
#define SCL_DDR DDRD
#define SDA_OUT PORTD
#define SCL_OUT PORTD
#define SDA_IN  PIND
#define SCL_IN  PIND

#define I2C_MAXWAIT 5000

uint8_t i2c_bus = 0;

// delay half period
#define i2c_delay_half() _delay_us(500000L / SCL_CLOCK)

// Wait for SCL to actually become high in case the slave keeps
// it low (clock stretching).
inline uint8_t i2c_wait_scl_high(void) {
	uint16_t retry = I2C_MAXWAIT;
	while (bit_is_clear(SCL_IN, SCL)) {
		if (--retry == 0) {
			i2c1_stop();
			return 1;
		}
	}
	return 0;
}

void i2c_select(uint8_t bus) {
	i2c_bus = bus;
}

// Initialization of the I2C bus interface.
void i2c0_init(void) {
	/* initialize TWI clock: 100 kHz clock, TWPS = 0 => prescaler = 1 */
	TWSR = 0;                         /* no prescaler */
	TWBR = ((F_CPU/SCL_CLOCK)-16)/2;  /* must be > 10 for stable operation */
}

void i2c1_init(void) {
	SDA_DDR &= ~_BV(SDA);  // release SDA
	SCL_DDR &= ~_BV(SCL);  // release SCL
	SDA_OUT &= ~_BV(SDA);
	SCL_OUT &= ~_BV(SCL);
}

void i2c_init(void) {
	(i2c_bus) ? i2c1_init() : i2c0_init();
}

// Issues a start condition and sends address and transfer direction.
// return 0 = device accessible, 1= failed to access device
uint8_t i2c0_start(uint8_t address) {
    uint8_t   twst;
	uint16_t  retry;

	// send START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

	// wait until transmission completed
	retry = I2C_MAXWAIT;
	while (!(TWCR & (1<<TWINT))) if (--retry == 0) return 1;

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ((twst != TW_START) && (twst != TW_REP_START)) return 1;

	// send device address
	TWDR = address;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wail until transmission completed and ACK/NACK has been received
	retry = I2C_MAXWAIT;
	while (!(TWCR & (1<<TWINT))) if (--retry == 0) return 1;

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ((twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK)) return 1;

	return 0;
}

uint8_t i2c1_start(uint8_t address) {
	SDA_DDR |= _BV(SDA);  // force SDA low
	i2c_delay_half();
	return i2c1_write(address);
}

uint8_t i2c_start(uint8_t address) {
	return (i2c_bus) ? i2c1_start(address) : i2c0_start(address);
}

// Issues a start condition and sends address and transfer direction.
// If device is busy, use ack polling to wait until device is ready
// Input:   address and transfer direction of I2C device
void i2c0_start_wait(uint8_t address) {
    uint8_t   twst;
	uint16_t  retry;

    while (1) {
	    // send START condition
	    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    
    	// wait until transmission completed
		retry = I2C_MAXWAIT;
    	while (!(TWCR & (1<<TWINT))) if (--retry == 0) return;
    
    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ((twst != TW_START) && (twst != TW_REP_START)) continue;
    
    	// send device address
    	TWDR = address;
    	TWCR = (1<<TWINT) | (1<<TWEN);
    
    	// wail until transmission completed
		retry = I2C_MAXWAIT;
    	while (!(TWCR & (1<<TWINT))) if (--retry == 0) return;
    
    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ((twst == TW_MT_SLA_NACK ) || (twst ==TW_MR_DATA_NACK)) {    	    
    	    /* device busy, send stop condition to terminate write operation */
	        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	        
	        // wait until stop condition is executed and bus released
			retry = I2C_MAXWAIT;
	        while (TWCR & (1<<TWSTO)) if (--retry == 0) return;
	        
    	    continue;
    	}
    	//if( twst != TW_MT_SLA_ACK) return 1;
    	break;
     }
}

void i2c1_start_wait(uint8_t address) {
	uint16_t retry = I2C_MAXWAIT;
	while (!i2c1_start(address)) {
		i2c1_stop();
		if (--retry == 0) return;
	}
}

void i2c_start_wait(uint8_t address) {
	(i2c_bus) ? i2c1_start_wait(address) : i2c0_start_wait(address);
}

// Issues a repeated start condition and sends address and transfer direction 
// Input:   address and transfer direction of I2C device
// Return:  0 device accessible
//          1 failed to access device
uint8_t i2c0_rep_start(uint8_t address) {
    return i2c0_start(address);
}

uint8_t i2c1_rep_start(uint8_t address) {
	SCL_DDR |= _BV(SCL);  // force SCL low
	i2c_delay_half();
	SDA_DDR &= ~_BV(SDA);  // release SDA
	i2c_delay_half();
	SCL_DDR &= ~_BV(SCL);  // release SCL
	i2c_delay_half();
	return i2c1_start(address);
}

uint8_t i2c_rep_start(uint8_t address) {
	return (i2c_bus) ? i2c1_rep_start(address) : i2c0_rep_start(address);
}

// Terminates the data transfer and releases the I2C bus
void i2c0_stop(void) {
	uint16_t  retry = I2C_MAXWAIT;
 
	/* send stop condition */
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
 
	// wait until stop condition is executed and bus released
	while (TWCR & (1<<TWSTO)) if (--retry == 0) return;
}

void i2c1_stop(void) {
	SCL_DDR |= _BV(SCL);  // force SCL low
	SDA_DDR |= _BV(SDA);  // force SDA low
	i2c_delay_half();
	SCL_DDR &= ~_BV(SCL);  // release SCL
	i2c_delay_half();
	SDA_DDR &= ~_BV(SDA);  // release SDA
	i2c_delay_half();
}

void i2c_stop(void) {
	(i2c_bus) ? i2c1_stop() : i2c0_stop();
}

// Send one byte to I2C device
// Input:    byte to be transfered
// Return:   0 write successful 
//           1 write failed
uint8_t i2c0_write(uint8_t data) {	
    uint8_t   twst;
	uint16_t  retry = I2C_MAXWAIT;
    
	// send data to the previously addressed device
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wait until transmission completed
	while (!(TWCR & (1<<TWINT))) if (--retry == 0) return 1;

	// check value of TWI Status Register. Mask prescaler bits
	twst = TW_STATUS & 0xF8;
	if (twst != TW_MT_DATA_ACK) return 1;
	return 0;
}

uint8_t i2c1_write(uint8_t data) {
	for (uint8_t i = 8; i; --i) {
		SCL_DDR |= _BV(SCL);  // force SCL low
		if (data & 0x80) {
			SDA_DDR &= ~_BV(SDA);  // release SDA
		} else {
			SDA_DDR |= _BV(SDA);  // force SDA low
		}
		i2c_delay_half();
		SCL_DDR &= ~_BV(SCL);  // release SCL
		if (i2c_wait_scl_high()) return 1;
		i2c_delay_half();
		data <<= 1;
	}
	// Get ACK
	SCL_DDR |= _BV(SCL);  // force SCL low
	SDA_DDR &= ~_BV(SDA);  // release SDA
	i2c_delay_half();
	SCL_DDR &= ~_BV(SCL);  // release SCL
	if (i2c_wait_scl_high()) return 1;
	uint8_t result = bit_is_set(SDA_IN, SDA) ? 1 : 0;
	i2c_delay_half();
	SCL_DDR |= _BV(SCL);  // Keep SCL low between bytes
	return result;
}

uint8_t i2c_write(uint8_t data) {
	return (i2c_bus) ? i2c1_write(data) : i2c0_write(data);
}

// Read one byte from the I2C device
// Input:  ack 1 send ack, request more data from device
//             0 send nak, read is followed by a stop condition
// Return: byte read from I2C device
uint8_t i2c0_read(uint8_t ack) {
	uint16_t  retry = I2C_MAXWAIT;
	
	if (ack) {
		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	} else {
		TWCR = (1<<TWINT) | (1<<TWEN);
	}
	while (!(TWCR & (1<<TWINT))) if (--retry == 0) return 0xFF;

	return TWDR;
}

uint8_t i2c1_read(uint8_t ack) {
	uint8_t data = 0;

	for (uint8_t i = 8; i; --i) {
		data <<= 1;
		SCL_DDR |= _BV(SCL);  // force SCL low
		SDA_DDR &= ~_BV(SDA);  // release SDA (from previous ACK)
		i2c_delay_half();
		SCL_DDR &= ~_BV(SCL);  // release SCL
		i2c_delay_half();
		// Read clock stretch
		if (i2c_wait_scl_high()) return 0xFF;
		if (bit_is_set(SDA_IN, SDA)) data |= 1;
	}
	// Put ACK/NACK
	SCL_DDR |= _BV(SCL);  // force SCL low
	if (ack) {
		SDA_DDR |= _BV(SDA);  // force SDA low
	} else {
		SDA_DDR &= ~_BV(SDA);  // release SDA
	}
	i2c_delay_half();
	SCL_DDR &= ~_BV(SCL);  // release SCL
	if (i2c_wait_scl_high()) return 0xFF;
	i2c_delay_half();
	SCL_DDR |= _BV(SCL);  // Keep SCL low between bytes
	return data;
}

uint8_t i2c_read(uint8_t ack) {
	return (i2c_bus) ? i2c1_read(ack) : i2c0_read(ack);
}