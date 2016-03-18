/*
 * Creates a test that repeatedly read accel id through I2C.
 */

#include <stdio.h>

#include "./i2c.h"
#include "./i2c-tm4c123-priv.h"
#include "./i2c-tm4c123-hw.h"

#define TEST_I2C_ACCEL		I2C_tm4c123_id_i2c0

#define MMA8452_ADDR		0x1D
#define ACCEL_XYZ_DATA_CFG	0x68
#define ACCEL_REG_WHO_AM_I	0x0D
#define CTL_REG1		0x2A

#define I2C_SENSOR_DATA_BUF_SZ	0x06

static int i2c_sensor_read(struct I2C_tm4c123_id *i2cid, uint8_t slave_addr,
			uint8_t cmd_reg, uint8_t *data, uint32_t count)
{
	int ret, i;

	struct I2C_packet xfers[] = {
		{
			.addr = slave_addr,
			.flag = false,
			.buf = &cmd_reg,
			.len = 1,
		},
		{
			.addr = slave_addr,
			.flag = true,
			.buf = data,
			.len = count,
		},
	};

	for (i = 0; i < ARRAY_SIZE(xfers); ++i) {
		ret = I2C_xfer(i2cid, &xfers[i]);
		if (ret)
			printf("ERROR: i2c_sensor_read failed\r\n");
	}

	return ret;
}

void main(void)
{
	int ret;
	uint8_t cmd_reg, buf_data[I2C_SENSOR_DATA_BUF_SZ];

	I2C_init(&TEST_I2C_ACCEL);

	while (1) {
		cmd_reg = ACCEL_REG_WHO_AM_I;
		ret = i2c_sensor_read(&TEST_I2C_ACCEL, MMA8452_ADDR,
					ACCEL_REG_WHO_AM_I, buf_data, 1);
		if (!ret)
			printf("MMA8452 - WHO AM I: %02x\r\n",
					(unsigned int)buf_data[0]);
		//delay100ms(1);
	}
}
