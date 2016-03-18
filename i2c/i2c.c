#include <stdint.h>

#include "reg-access.h"
#include "i2c-tm4c123-hw.h"
#include "tm4c123gh6pm.h"

#define I2C_MSA_R	0x000	// Master slave addr reg offset
#define I2C_MCS_R	0x004	// Master Control/Status reg offset
#define I2C_MDR_R	0x008	// Master data reg offset
#define I2C_MTPR_R	0x00C	// Master Time Period reg oggset
#define I2C_MCR_R	0x020	// Master Configuration

#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define AFSEL_R_OFFSET      0x420   // alternate function select reg offset
#define ODR_R_OFFSET        0x50C   // open drain enable reg offset
#define DEN_R_OFFSET        0x51C   // digital enable reg offset
#define AMSEL_R_OFFSET      0x528   // analog mux reg offset
#define PCTL_R_OFFSET       0x52C   // PCTL reg offset

#define	RS		1
#define BIT(x)		(1 << x)
#define MAXRETRIES	3

static void I2C_ready(uint32_t base_addr)
{
    while (readl(base_addr + I2C_MCS_R) &
			I2C_MCS_BUSY) {}; // check if busy bit 0.
}

/*
 * Programming sequence for an I2C master read is as follows:
 * 1. Check for the bus IDLE (SCL and SDA high)
 * 2. When the bus is in IDLE, write the slave address to I2CMSA reg
 * and configure the R/S bit for rx/tx
 * 3. Program the control/status reg for current byte
 * 4. The master reads from the I2CMDR reg
 * 5. If no error on all bytes, generate a STOP condition
 */
static uint8_t I2C_read_byte(struct I2C_tm4c123_id *id, uint8_t slave,
				uint32_t ctl_cmd_reg)
{
	int repeats_on_error = 0;
	uint32_t msa_r;
	uint32_t data;

	do {
	       if (slave) {
			    I2C_ready(id->base_addr);			// wait for I2C ready
			    msa_r = (slave << 1) & 0xFE;	// MSA[7:1] is slave addr
			    msa_r |= 0x01;			// MSA[0] is 1 for rx
			    writel(msa_r, id->base_addr + I2C_MSA_R);
		}
		writel(ctl_cmd_reg, id->base_addr + I2C_MCS_R);
		I2C_ready(id->base_addr);			// wait for transmission done
		repeats_on_error++;
	} while (((readl(id->base_addr + I2C_MCS_R) &
			(I2C_MCS_ADRACK | I2C_MCS_ERROR)) != 0) &&
			(repeats_on_error <= MAXRETRIES)); // repeat if error
	data = readl(id->base_addr + I2C_MDR_R);

	return (data & 0xFF);			// usually returns 0xFF on error
}

static int I2C_read(struct I2C_tm4c123_id *id, struct I2C_packet *xfer)
{
	int i = 0;
	uint32_t ctrl_cmd_val = 0;
	uint8_t data;

	if (xfer->len == 1) {
		ctrl_cmd_val = (0
			| I2C_MCS_STOP     // generate stop
			| I2C_MCS_START    // generate start/restart
			| I2C_MCS_RUN);    // master enable
		data = I2C_read_byte(id, xfer->addr, ctrl_cmd_val);
		if (data != 0xFF)
			*xfer->buf++ = data;
		else
			return -1;
	} else {
		ctrl_cmd_val = (0
			| I2C_MCS_ACK		// positive data ack
			| I2C_MCS_START		// generate start/restart
			| I2C_MCS_RUN);		// master enable
		data = I2C_read_byte(id, xfer->addr, ctrl_cmd_val);
		if (data == 0xFF)
			return -1;
		*xfer->buf++ = data;
		for (i = 1; i < xfer->len - 1; ++i) {
			ctrl_cmd_val = (0
				| I2C_MCS_ACK		// positive data ack
				| I2C_MCS_RUN);		// master enable
			data = I2C_read_byte(id, 0, ctrl_cmd_val);
			if (data == 0xFF)
				return -1;
			*xfer->buf++ = data;
		}
		ctrl_cmd_val = (0
			| I2C_MCS_STOP     // generate stop
			| I2C_MCS_RUN);    // master enable
		data = I2C_read_byte(id, 0, ctrl_cmd_val);
		if (data == 0xFF)
			return -1;
		*xfer->buf++ = data;
	}

	return 0;
}

/*
 * Programming sequence for an I2C master write is as follows:
 * 1. Check for the bus IDLE (SCL and SDA high)
 * 2. When the bus is in IDLE, write the slave address to I2CMSA reg
 * and configure the R/S bit for rx/tx
 * 3. The master writes the data to I2CMDR reg
 * 4. Program the control/status reg for current byte
 * 5. wait for trasmission done
 * 6. If all bytes were tx'ed successfully, generate a STOP condition
 */
static int I2C_write_byte(struct I2C_tm4c123_id *id, uint8_t slave,
				uint8_t data, uint32_t ctl_cmd_reg)
{
	uint32_t msa_r;
	uint32_t mcs_r;

	/* Address of the slave is used only for first byte on multi-byte
	 * transfers.
	 */
	if (slave) {
		I2C_ready(id->base_addr);			// wait for I2C ready
		msa_r = (slave << 1) & 0xFE;	// MSA[7:1] is slave addr
		msa_r &= ~0x01;			// MSA[0] is 1 for tx
		writel(msa_r, id->base_addr + I2C_MSA_R);
	}
	writel(data & 0xFF, id->base_addr + I2C_MDR_R);	// prepare first byte
	writel(ctl_cmd_reg, id->base_addr + I2C_MCS_R);
	I2C_ready(id->base_addr);			// wait for I2C ready
	mcs_r = readl(id->base_addr + I2C_MCS_R);
	//printf("MASTER CTL STATUS : 0x%x", mcs_r);
	if ((mcs_r & (I2C_MCS_DATACK | I2C_MCS_ADRACK | I2C_MCS_ERROR)) != 0) {
		mcs_r = (0 | I2C_MCS_STOP);
		writel(mcs_r, id->base_addr + I2C_MCS_R);
		return -1;
	} else {
		return 0;
	}
}

static int I2C_write(struct I2C_tm4c123_id *id, struct I2C_packet *xfer)
{
	int i = 0;
	uint32_t ctrl_cmd_val = 0;
	uint32_t ret;

	if (xfer->len == 1) {
		ctrl_cmd_val = (0
			| I2C_MCS_START    // generate start/restart
			| I2C_MCS_RUN);    // master enable
		ctrl_cmd_val |= (xfer->flag != SR ? I2C_MCS_STOP : 0);
		ret = I2C_write_byte(id, xfer->addr, *xfer->buf, ctrl_cmd_val);
		if (ret)
			return ret;
	} else {
		ctrl_cmd_val = (0
			| I2C_MCS_START		// generate start/restart
			| I2C_MCS_RUN);		// master enable
		ret = I2C_write_byte(id, xfer->addr, *xfer->buf++, ctrl_cmd_val);
		if (ret)
			return ret;
		for (i = 1; i < xfer->len - 1; ++i) {
			ctrl_cmd_val = (0
				| I2C_MCS_RUN);		// master enable
		    ret = I2C_write_byte(id, 0, *xfer->buf++, ctrl_cmd_val);
			if (ret)
				return ret;
		}
		ctrl_cmd_val = (0
			| I2C_MCS_STOP     // generate stop
			| I2C_MCS_RUN);    // master enable
		ret = I2C_write_byte(id, 0, *xfer->buf, ctrl_cmd_val);
		if (ret)
			return ret;
	}

	return 0;
}

int I2C_xfer(struct I2C_tm4c123_id *id, struct I2C_packet *xfer)
{
	int ret = 0;

	if (xfer->is_read)
		ret = I2C_read(id, xfer);
	else
		ret = I2C_write(id, xfer);

	return ret;
}

void I2C_init(struct I2C_tm4c123_id *id)
{
	struct I2C_tm4c123_id *module = id;
	uint32_t pins;
	uint32_t pctl_r;
	uint8_t amsel_r;

	pins = BIT(module->pins.scl) | BIT(module->pins.sda);
	/* Enable the I2C clock in the system control module */
	SYSCTL_RCGCI2C_R |= BIT(module->id);
	/* Enable the clock to the appropriate GPIO module via the
	 * RCGCGPIO register in the system control module
	 * I2C0 --> PORT B
	 * I2C1 --> PORT A
	 * I2C2 --> PORT E
	 * I2C3 --> PORT D
	 */
	SYSCTL_RCGCGPIO_R |= BIT(module->gpio_port_num);
	while ((SYSCTL_PRGPIO_R & BIT(module->gpio_port_num)) == 0) {};
	/* Enable the appropriate GPIO pins for alternate function */
	frwritel(pins, module->gpio_base_addr + AFSEL_R_OFFSET);
	/* Enable the I2CSDA pin for open drain */
	frwritel(BIT(module->pins.sda), module->gpio_base_addr + ODR_R_OFFSET);
	/* Enable the digital enable for appropriate GPIO pins */
	frwritel(pins, module->gpio_base_addr + DEN_R_OFFSET);
	/* Configure the PMCn fields in the GPIOPCTL register to
	 * assign the I2C signals to the appropriate pins.
	 */
	pctl_r = readl(module->gpio_base_addr + PCTL_R_OFFSET) & 0xFFFF00FF;
	pctl_r += 0x00003300;
	writel(pctl_r, module->gpio_base_addr + PCTL_R_OFFSET);
	/* Disable the Analog multiplexer on the GPIO pins */
	amsel_r = readb(module->gpio_base_addr + AMSEL_R_OFFSET);
	amsel_r &= ~pins;
	writeb(amsel_r, module->gpio_base_addr + AMSEL_R_OFFSET);
	/* Initialize the I2C master by writing to the I2CMCR
	 * register with a value of 0x0000.0010
	 */
	writel(I2C_MCR_MFE, module->base_addr + I2C_MCR_R);
	/* Set the desired SCL clock speed of 100Kbps by writing
	 * to the I2CMTPR register with the correct value. The
	 * value written represents the number of system clock
	 * periods in one SCL clock period. The TPR value is
	 * determined by the following equation:
	 * TPR = (Sys_Clk / (2 * (SCL_LP + SCL_HP) * SCL_CLK)) - 1;
	 * TPR = (50 Mhz / (2 * (6 + 4) * 20ns) * 100000)) - 1;
	 * TPR = 24;
	 */
	writel(39, module->base_addr + I2C_MTPR_R);
}
