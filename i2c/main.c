#include <stdio.h>   // standard C library
#include <string.h>

#include "uart.h"    // functions to implment input/output
#include "i2c.h"
#include "i2c-tm4c123-priv.h"
#include "i2c-tm4c123-hw.h"
#include "PLL.h"
#include "TExaS.h"

#define ARRAY_SIZE(A)		(sizeof(A)/sizeof(A[0]))
#define TEST_I2C_ACCEL		I2C_tm4c123_id_i2c0

#define MMA8452_ADDR		0x1D
#define ACCEL_XYZ_DATA_CFG	0x68
#define ACCEL_REG_WHO_AM_I	0x0D
#define CTL_REG1		0x2A
#define OUT_x_MSB		0x01

#define GSCALE			2
#define I2C_ERROR		0xFF

#define STANDBY			false
#define ACTIVE			true

static void MMA8452_i2cWrite(uint8_t *data, uint32_t bytes)
{
	struct I2C_packet xfer;
	uint32_t i;

	xfer.addr = MMA8452_ADDR;
	xfer.is_read = false;
	xfer.flag = SR;
	xfer.buf = data;
	xfer.len = bytes;
	if (I2C_xfer(&TEST_I2C_ACCEL, &xfer))
		printf("ERROR: I2C_xfer() in %s failed\r\n", __func__);
}

static void MMA8452_i2cRead(uint8_t reg_addr, uint8_t *data, uint32_t bytes)
{
	struct I2C_packet xfers[2];
	uint32_t ret;
	uint32_t i;

	xfers[0].addr = MMA8452_ADDR;
	xfers[0].is_read = false;
	xfers[0].flag = SR;
	xfers[0].buf = &reg_addr;
	xfers[0].len = 1;
	xfers[1].addr = MMA8452_ADDR;
	xfers[1].is_read = true;
	xfers[1].buf = data;
	xfers[1].len = bytes;
	for (i = 0; i < ARRAY_SIZE(xfers); ++i) {
		if (I2C_xfer(&TEST_I2C_ACCEL, &xfers[i]))
			printf("ERROR: I2C_xfer() in %s failed\r\n", __func__);
	}
}

static int MMA8452_Mode(bool mode)
{
	uint8_t response[2];

	MMA8452_i2cRead(CTRL_REG1, &response[1], 1);
	if (mode == STANDBY)
		response[1] &= ~0x01; // clear the active bit to go into standby
	else
		response[1] |= 0x01; // set the active bit to go into active
	response[0] = CTRL_REG1;
	MMA8452_i2cWrite(response, 2);
}

static void MMA8452_setScale(uint8_t scale)
{
	uint8_t buf[2];
	uint8_t fsr;

	MMA8452_Mode(STANDBY); // must be in standby to change registers
	fsr = scale;
	if (fsr > 8) fsr = 8;
	fsr >>= 2; // see page 22. 00 = 2G, 01 = 4G, 02 = 8G
	buf[0] = ACCEL_XYZ_DATA_CFG;
	buf[1] = fsr;
	MMA8452_i2cWrite(buf, 2);
	MMA8452_Mode(ACTIVE); // set to active to start reading
}

static int MMA8452_Init(void)
{
	uint8_t response;

	MMA8452_i2cRead(ACCEL_REG_WHO_AM_I, &response, 1)
	if (response != 0x2A) {
		printf("Could not connect to MMA8452 : 0x%02x", response);
		return -1;
	}
	printf("MMA8452 is online...\r\n");
	MMA8452_setScale(GSCALE);

	return 0;
}

/* Subroutine to wait 0.1 sec
 * Inputs: None
 * Outputs: None
 * Notes: ...
 */
void Delay(int n)
{
	unsigned long volatile time;

	while (n--) {
		time = 727240*200/91;  // 0.1sec
		while(time){
			time--;
		}
	}
}

static void readAccelData(int *destination)
{
	uint8_t rawData[6];
	int i;
	int gcount;

	MMA8452_i2cRead(OUT_X_MSB, rawData, 6); // Read the six raw data registers into array
	// loop to calculate 12-bit ADC and g value for each axis
	for (i = 0; i < 3; ++i) {
		gcount = (rawData[i*2] << 8) | (rawData[(i*2) + 1]);
		gcount >>= 4; // right align the number
		// if the number is negative, we have to make it so manually.(no 12-bit data type)
		if (rawData[i*2] > 0x7F)
			gcount -= 0x1000;
		destination[i] = gcount;
	}
}

int main (void) {
	int ret;
	int input;
	int accelCount[3]; // store 12-bit signed value
	float accelG[3];

	TExaS_Init(UART_PIN_PA0,UART_PIN_PA1);
	PLL_init1();	// run at 80Mhz
	UART_Init();    // initialize UART for printing
	printf("MMA8452 basic example on TIVA launchpad\r\n");
	I2C_init(&TEST_I2C_ACCEL);
	while (1) {
		printf("1. I2C test\r\n2. Exit\r\n");
		scanf("%d", &input);
		switch (input) {
			case 1:
				ret = MMA8452_Init();
				if (!ret) {
					readAccelData(accelCount);
					for (i = 0; i < 3; ++i) {
						// get actual g value, depending on the scale
						accelG[i] = (float) accelCount[i] / ((1 << 12) / (2 * GSCALE));
					}
					// print out values
					for (i = 0; i < 3; ++i) {
						printf("%f ", accelG[i]);
					}
					printf("\n");
				}
				Delay(2);
				break;
			case 2:
				return 0;
			default:
				break;
		}
	}
}
