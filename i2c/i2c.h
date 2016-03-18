#ifndef _I2C_H_
#define _I2C_H_

#include <stdint.h>
#include <stdbool.h>

typedef unsigned char uint8_t;
typedef unsigned int uint32_t;

struct I2C_tm4c123_id;

/*
 * This struct represents an I2C packet for an I2C transaction.
 * Fields:
 * addr:    Address of the slave device
 * flag:    To indicate read or write
 * buf:     buffer that contains the data for tx/rx
 * len:     Length of the buffer
 */
struct I2C_packet {
	uint8_t addr;
	bool is_read;
	bool flag;
	uint8_t *buf;
	uint32_t len;
};

/*
 * Initializes the I2C module and configures the respective
 * pins for I2C functionality.
 * Parameters:
 */
void I2C_init(struct I2C_tm4c123_id *id);
/*
 * Perform an I2C transfer for the current packet.
 * Parameters:
 * setting:     Current setting for module
 * xfer:        Struct describing current packet contents
 *
 * Returns:
 * Zero:        Ok
 * Non-zer0:    Error
 */
int I2C_xfer(struct I2C_tm4c123_id *id, struct I2C_packet *xfer);

#endif
