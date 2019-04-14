//-----------------------------------------------------------------------------
/*

I2C Rotary Encoder V2 Driver
Author: Jason Harris (https://github.com/deadsy)

https://github.com/Fattoresaimon/I2CEncoderV2
https://www.kickstarter.com/projects/1351830006/i2c-encoder-v2

*/
//-----------------------------------------------------------------------------

#include <nuttx/config.h>

#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/rei2c.h>

//-----------------------------------------------------------------------------
// registers

#define REI2C_GCONF    0x00	// General Configuration (1 byte)
#define REI2C_GP1CONF  0x01	// GP 1 Configuration (1 byte)
#define REI2C_GP2CONF  0x02	// GP 2 Configuration (1 byte)
#define REI2C_GP3CONF  0x03	// GP 3 Configuration (1 byte)
#define REI2C_INTCONF  0x04	// INT pin Configuration (1 byte)
#define REI2C_ESTATUS  0x05	// Encoder Status (1 byte)
#define REI2C_I2STATUS 0x06	// Secondary interrupt status (1 byte)
#define REI2C_FSTATUS  0x07	// Fade process status (1 byte)
#define REI2C_CVAL     0x08	// Counter Value (4 bytes)
#define REI2C_CMAX     0x0C	// Counter Max value (4 bytes)
#define REI2C_CMIN     0x10	// Counter Min value (4 bytes)
#define REI2C_ISTEP    0x14	// Increment step value (4 bytes)
#define REI2C_RLED     0x18	// LED red color intensity (1 byte)
#define REI2C_GLED     0x19	// LED green color intensity (1 byte)
#define REI2C_BLED     0x1A	// LED blue color intensity (1 byte)
#define REI2C_GP1REG   0x1B	// I/O GP1 Register (1 byte)
#define REI2C_GP2REG   0x1C	// I/O GP2 Register (1 byte)
#define REI2C_GP3REG   0x1D	// I/O GP3 Register (1 byte)
#define REI2C_ANTBOUNC 0x1E	// Anti-bouncing period (1 Byte)
#define REI2C_DPPERIOD 0x1F	// Double push period (1 Byte)
#define REI2C_FADERGB  0x20	// Fade timer RGB Encoder (1 Byte)
#define REI2C_FADEGP   0x21	// Fade timer GP ports (1 Byte)
#define REI2C_EEPROM   0x80	// EEPROM memory (128 bytes)

// REI2C_GCONF bits
#define REI2C_GCONF_DTYPE (1 << 0)	// Data type of the register: CVAL, CMAX, CMIN and ISTEP.
#define REI2C_GCONF_WRAPE (1 << 1)	// Enable counter wrap.
#define REI2C_GCONF_DIRE  (1 << 2)	// Direction of the encoder when increment.
#define REI2C_GCONF_IPUD  (1 << 3)	// Interrupt Pull-UP disable.
#define REI2C_GCONF_RMOD  (1 << 4)	// Reading Mode.
#define REI2C_GCONF_ETYPE (1 << 5)	// Set the encoder type (normal/illuminated)
#define REI2C_GCONF_MBANK (1 << 6)	// Select the EEPROM memory bank. Each bank is 128 bytes.
#define REI2C_GCONF_RESET (1 << 7)	// Reset the I2C Encoder V2

// REI2C_ESTATUS bits
#define REI2C_ESTATUS_PUSHR (1 << 0)	// push button has been released
#define REI2C_ESTATUS_PUSHP (1 << 1)	// push button has been pressed
#define REI2C_ESTATUS_PUSHD (1 << 2)	// push button has been double pushed
#define REI2C_ESTATUS_RINC  (1 << 3)	// rotated in the increase direction
#define REI2C_ESTATUS_RDEC  (1 << 4)	// rotated in the decrease direction
#define REI2C_ESTATUS_RMAX  (1 << 5)	// maximum counter value has been reached
#define REI2C_ESTATUS_RMIN  (1 << 6)	// minimum counter value has been reached
#define REI2C_ESTATUS_INT2  (1 << 7)	// Secondary interrupt status

//-----------------------------------------------------------------------------

struct rei2c_dev_s {
	struct i2c_master_s *i2c_dev;	// I2C interface connected to rei2c
	sem_t rei2c_exclsem;	// Supports exclusive access to the device
};

//-----------------------------------------------------------------------------
// file operations

static int rei2c_open(struct file *filep) {
	iinfo("%d\n", __LINE__);
	return OK;
}

static int rei2c_close(struct file *filep) {
	iinfo("%d\n", __LINE__);
	return OK;
}

static ssize_t rei2c_read(struct file *filep, char *buffer, size_t buflen) {
	iinfo("buffer %p buflen %d\n", buffer, buflen);
	return 0;
}

static ssize_t rei2c_write(struct file *filep, const char *buffer, size_t buflen) {
	iinfo("buffer %p buflen %d\n", buffer, buflen);
	return buflen;
}

static int rei2c_ioctl(struct file *filep, int cmd, unsigned long arg) {
	iinfo("%d\n", __LINE__);
	return OK;
}

#ifndef CONFIG_DISABLE_POLL
static int rei2c_poll(struct file *filep, struct pollfd *fds, bool setup) {
	iinfo("%d\n", __LINE__);
	return OK;
}
#endif				// CONFIG_DISABLE_POLL

static const struct file_operations rei2c_fops = {
	rei2c_open,		// open
	rei2c_close,		// close
	rei2c_read,		// read
	rei2c_write,		// write
	NULL,			// seek
	rei2c_ioctl,		// ioctl
#ifndef CONFIG_DISABLE_POLL
	rei2c_poll,		// poll
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
	NULL,			// unlink
#endif
};

//-----------------------------------------------------------------------------

int rei2c_register(const char *devname, struct i2c_master_s *i2c) {
	struct rei2c_dev_s *priv;
	int ret;

	iinfo("%s\n", devname);

	DEBUGASSERT(devname && i2c);

	// Allocate a new rei2c driver instance
	priv = (struct rei2c_dev_s *)kmm_zalloc(sizeof(struct rei2c_dev_s));
	if (!priv) {
		ierr("ERROR: Failed to allocate device structure\n");
		return -ENOMEM;
	}
	// Save the i2c device
	priv->i2c_dev = i2c;

	// Initialize the new rei2c driver instance
	nxsem_init(&priv->rei2c_exclsem, 0, 1);

	// And register the rei2c driver
	ret = register_driver(devname, &rei2c_fops, 0666, priv);
	if (ret < 0) {
		ierr("ERROR: register_driver failed: %d\n", ret);
		goto errout_with_priv;
	}

	return OK;

 errout_with_priv:
	nxsem_destroy(&priv->rei2c_exclsem);
	kmm_free(priv);
	return ret;
}

//-----------------------------------------------------------------------------
