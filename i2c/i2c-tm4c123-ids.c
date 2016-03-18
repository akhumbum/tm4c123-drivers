#include "i2c-tm4c123-hw.h"
#include "i2c-tm4c123-priv.h"
#include "i2c.h"

enum {
	I2C0 = 0,
	I2C1,
	I2C2,
	I2C3,
};

/*
 * The following GPIO's are used by I2C modules:
 * GPIO_PORTB,		// I2C0 --> Port B
 * GPIO_PORTA,		// I2C1 --> Port A
 * GPIO_PORTE,		// I2C2 --> Port E
 * GPIO_PORTD,		// I2C3 --> Port D
 */
enum {
	GPIO_PORTA = 0,
	GPIO_PORTB,
	GPIO_PORTC,
	GPIO_PORTD,
	GPIO_PORTE,
	GPIO_PORTF,
};

enum {
	I2C0_BASE_ADDR = 0x40020000,
	I2C1_BASE_ADDR = 0x40021000,
	I2C2_BASE_ADDR = 0x40022000,
	I2C3_BASE_ADDR = 0x40023000,
};

/* The following enum represents addresses through AHB bus aperture.
 * For APB, start from 0x40004000.
 * 0x40059000,		// I2C0 --> Port B
 * 0x40058000,		// I2C1 --> Port A
 * 0x4005C000,		// I2C2 --> Port E
 * 0x4005B000,		// I2C3 --> Port D
 */
enum {
	I2C0_GPIO_BASE_ADDR = 0x40005000,
	I2C1_GPIO_BASE_ADDR = 0x40004000,
	I2C2_GPIO_BASE_ADDR = 0x4000C000,
	I2C3_GPIO_BASE_ADDR = 0x40024000,
};

enum {
	I2C0_SCL = 2,
	I2C0_SDA = 3,
	I2C1_SCL = 6,
	I2C1_SDA = 7,
	I2C2_SCL = 4,
	I2C2_SDA = 5,
	I2C3_SCL = 0,
	I2C3_SDA = 1,
};

struct I2C_tm4c123_id I2C_tm4c123_id_i2c0 = {
	{I2C0_SCL, I2C0_SDA},
	/* set the I2C module base address for the current module */
	I2C0_BASE_ADDR,
	/* set the I2C module's respective gpio base address for the
	 * current module
	 */
	I2C0_GPIO_BASE_ADDR,
	/* set the port pins used for the current I2C module */
	GPIO_PORTB,
	I2C0
};

struct I2C_tm4c123_id I2C_tm4c123_id_i2c1 = {
	{I2C1_SCL, I2C1_SDA},
	/* set the I2C module base address for the current module */
	I2C1_BASE_ADDR,
	/* set the I2C module's respective gpio base address for the
	 * current module
	 */
	I2C1_GPIO_BASE_ADDR,
	/* set the port pins used for the current I2C module */
	GPIO_PORTA,
	I2C1
};

struct I2C_tm4c123_id I2C_tm4c123_id_i2c2 = {
	{I2C2_SCL, I2C2_SDA},
	/* set the I2C module base address for the current module */
	I2C2_BASE_ADDR,
	/* set the I2C module's respective gpio base address for the
	 * current module
	 */
	I2C2_GPIO_BASE_ADDR,
	/* set the port pins used for the current I2C module */
	GPIO_PORTE,
	I2C2
};

struct I2C_tm4c123_id I2C_tm4c123_id_i2c3 = {
	{I2C3_SCL, I2C3_SDA},
	/* set the I2C module base address for the current module */
	I2C3_BASE_ADDR,
	/* set the I2C module's respective gpio base address for the
	 * current module
	 */
	I2C3_GPIO_BASE_ADDR,
	/* set the port pins used for the current I2C module */
	GPIO_PORTD,
	I2C3
};
