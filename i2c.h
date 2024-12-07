/*
 * I2C (TWI) Master Software Library
 *
 * Basic routines for communicating with I2C slave devices. This single master
 * implementation is limited to one bus master on the I2C bus. It is based on
 * Peter Fleury's I2C software library.
 * User selectable between built-in TWI hardware and software implementation
 * of the I2C protocol. Use 4.7k pull-up resistor on the SDA and SCL pin.
 * Adapt the SCL and SDA port pin definitions to your target when using the
 * software I2C implementation.
 *
 * Created: 10/11/2024 10:29:50
 *  Author: Tim Dorssers
 */ 


#ifndef I2C_H_
#define I2C_H_

/* defines the data direction (reading from I2C device) in i2cx_start(), i2cx_rep_start() */
#define I2C_READ  1
/* defines the data direction (writing to I2C device) in i2cx_start(), i2cx_rep_start() */
#define I2C_WRITE 0

/* enable software implementation of the I2C protocol which runs on any AVR */
#define SOFTWARE
/* enable TWI hardware interface for all AVR with built-in TWI hardware */
#define HARDWARE

#if defined(HARDWARE) & !defined(SOFTWARE)
	#define i2c_init() i2c0_init()
	#define i2c_stop() i2c0_stop()
	#define i2c_start(__addr) i2c0_start(__addr)
	#define i2c_rep_start(__addr) i2c0_rep_start(__addr)
	#define i2c_start_wait(__addr) i2c0_start_wait(__addr)
	#define i2c_write(__data) i2c0_write(__data)
	#define i2c_read(__ack) i2c0_read(__ack)
	#define i2c_readAck() i2c0_read(1)
	#define i2c_readNak() i2c0_read(0)
#elif !defined(HARDWARE) & defined(SOFTWARE)
	#define i2c_init() i2c1_init()
	#define i2c_stop() i2c1_stop()
	#define i2c_start(__addr) i2c1_start(__addr)
	#define i2c_rep_start(__addr) i2c1_rep_start(__addr)
	#define i2c_start_wait(__addr) i2c1_start_wait(__addr)
	#define i2c_write(__data) i2c1_write(__data)
	#define i2c_read(__ack) i2c1_read(__ack)
	#define i2c_readAck() i2c1_read(1)
	#define i2c_readNak() i2c1_read(0)
#else
	extern void i2c_select(uint8_t bus);
	extern void i2c_init(void);
	extern void i2c_stop(void);
	extern uint8_t i2c_start(uint8_t addr);
	extern uint8_t i2c_rep_start(uint8_t addr);
	extern void i2c_start_wait(uint8_t addr);
	extern uint8_t i2c_write(uint8_t data);
	extern uint8_t i2c_read(uint8_t ack);
	#define i2c_readAck() i2c_read(1)
	#define i2c_readNak() i2c_read(0)
#endif

extern void i2c0_init(void);
extern void i2c0_stop(void);
extern uint8_t i2c0_start(uint8_t addr);
extern uint8_t i2c0_rep_start(uint8_t addr);
extern void i2c0_start_wait(uint8_t addr);
extern uint8_t i2c0_write(uint8_t data);
extern uint8_t i2c0_read(uint8_t ack);
#define i2c0_readAck() i2c0_read(1)
#define i2c0_readNak() i2c0_read(0)

extern void i2c1_init(void);
extern void i2c1_stop(void);
extern uint8_t i2c1_start(uint8_t addr);
extern uint8_t i2c1_rep_start(uint8_t addr);
extern void i2c1_start_wait(uint8_t addr);
extern uint8_t i2c1_write(uint8_t data);
extern uint8_t i2c1_read(uint8_t ack);
#define i2c1_readAck() i2c1_read(1)
#define i2c1_readNak() i2c1_read(0)

#endif /* I2C_H_ */