#ifndef _I2C_TM4C123_PRIV_H_
#define _I2C_TM4C123_PRIV_H_

#include "i2c.h"

/*
 * This struct represents an the GPIO pins used for the
 * current I2C module.
 * Fields:
 * scl:		GPIO pin used for clk
 * sda:		GPIO pin used for data
 */
struct I2C_gpio_pins {
	uint8_t scl;
	uint8_t sda;
};

/*
 * This struct represents an I2C module of tm4c123.
 * Fields:
 * setup:       Module settings for the current module
 * base_addr:   Base address of the register space
 */
struct I2C_tm4c123_id {
	struct I2C_gpio_pins pins;
	uint32_t base_addr;
	uint32_t gpio_base_addr;
	uint8_t gpio_port_num;
	uint8_t id;
};

#endif
