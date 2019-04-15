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
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/input/rei2c.h>

//-----------------------------------------------------------------------------

#define REI2C_ADDR 0x55

//-----------------------------------------------------------------------------

struct rei2c_dev_s {
	struct i2c_master_s *i2c;	// I2C interface connected to rei2c
	sem_t sem;		// Supports exclusive access to the device
	uint8_t addr;		// i2c device address
	const struct rei2c_cfg *cfg;	// user provided device configuration
};

//-----------------------------------------------------------------------------
// basic read/write functions

static int rei2c_i2c_read(struct rei2c_dev_s *dev, uint8_t reg, uint8_t * data, size_t len) {
	struct i2c_msg_s msgv[2] = {
		{
		 .frequency = I2C_SPEED_STANDARD,
		 .addr = dev->addr,
		 .flags = 0,
		 .buffer = &reg,
		 .length = 1,
		 },
		{
		 .frequency = I2C_SPEED_STANDARD,
		 .addr = dev->addr,
		 .flags = I2C_M_READ,
		 .buffer = data,
		 .length = len,
		 }
	};
	int ret = I2C_TRANSFER(dev->i2c, msgv, 2);
	if (ret < 0) {
		ierr("I2C_TRANSFER failed %d\n", ret);
		return ret;
	}
	return OK;
}

static int rei2c_i2c_write(struct rei2c_dev_s *dev, uint8_t reg, uint8_t * data, size_t len) {
	struct i2c_msg_s msgv[2] = {
		{
		 .frequency = I2C_SPEED_STANDARD,
		 .addr = dev->addr,
		 .flags = 0,
		 .buffer = &reg,
		 .length = 1,
		 },
		{
		 .frequency = I2C_SPEED_STANDARD,
		 .addr = dev->addr,
		 .flags = I2C_M_NOSTART,
		 .buffer = data,
		 .length = len,
		 }
	};
	int ret = I2C_TRANSFER(dev->i2c, msgv, 2);
	if (ret < 0) {
		ierr("I2C_TRANSFER failed %d\n", ret);
		return ret;
	}
	return OK;
}

static int rei2c_rd8(struct rei2c_dev_s *dev, uint8_t reg, uint8_t * val) {
	return rei2c_i2c_read(dev, reg, val, 1);
}

static int rei2c_rd24(struct rei2c_dev_s *dev, uint8_t reg, uint32_t * val) {
	return rei2c_i2c_read(dev, reg, (uint8_t *) val, 3);
}

static int rei2c_rd32(struct rei2c_dev_s *dev, uint8_t reg, uint32_t * val) {
	return rei2c_i2c_read(dev, reg, (uint8_t *) val, 4);
}

static int rei2c_wr8(struct rei2c_dev_s *dev, uint8_t reg, uint8_t val) {
	return rei2c_i2c_write(dev, reg, &val, 1);
}

static int rei2c_wr24(struct rei2c_dev_s *dev, uint8_t reg, uint32_t val) {
	return rei2c_i2c_write(dev, reg, (uint8_t *) & val, 3);
}

static int rei2c_wr32(struct rei2c_dev_s *dev, uint8_t reg, uint32_t val) {
	return rei2c_i2c_write(dev, reg, (uint8_t *) & val, 4);
}

//-----------------------------------------------------------------------------

static inline int rei2c_takesem(sem_t * sem) {
	int ret;
	// Take a count from the semaphore, possibly waiting */
	ret = nxsem_wait(sem);
	// The only case that an error should occur here is if the wait was awakened by a signal
	DEBUGASSERT(ret == OK || ret == -EINTR);
	return ret;
}

static inline int rei2c_givesem(sem_t * sem) {
	return nxsem_post(sem);
}

//-----------------------------------------------------------------------------

static int rei2c_init(struct rei2c_dev_s *dev) {
	int idx = 0;
	int rc;

	rc = rei2c_wr8(dev, REI2C_GCONF, REI2C_GCONF_RESET);
	if (rc < 0) {
		ierr("device reset failed %d\n", rc);
		goto exit;
	}
	// wait > 400 usecs
	nxsig_usleep(800);

	// check some register values
	uint8_t val0, val1;
	rei2c_rd8(dev, REI2C_GP1CONF, &val0);
	rei2c_rd8(dev, REI2C_ANTBOUNC, &val1);
	if ((val0 != 0) || (val1 != 25)) {
		ierr("bad device values\n");
		goto exit;
	}
	// apply the per-object register configuration
	if (dev->cfg != NULL) {
		while (dev->cfg[idx].reg != 0xff) {
			uint8_t reg = dev->cfg[idx].reg;
			if ((reg == REI2C_CVAL) || (reg == REI2C_CMAX) || (reg == REI2C_CMIN) || (reg == REI2C_ISTEP)) {
				rei2c_wr32(dev, reg, dev->cfg[idx].val);
			} else {
				rei2c_wr8(dev, reg, (uint8_t) dev->cfg[idx].val);
			}
			idx += 1;
		}
	}

 exit:
	return rc;
}

//-----------------------------------------------------------------------------
// file operations

static int rei2c_open(struct file *filep) {
	struct inode *inode;
	struct rei2c_dev_s *dev;
	int ret;

	DEBUGASSERT(filep && filep->f_inode);
	inode = filep->f_inode;
	DEBUGASSERT(inode->i_private);
	dev = (struct rei2c_dev_s *)inode->i_private;

	iinfo("filep %p\n", filep);

	ret = rei2c_takesem(&dev->sem);
	if (ret < 0) {
		ierr("ERROR: rei2c_takesem failed: %d\n", ret);
		return ret;
	}

 exit:
	rei2c_givesem(&dev->sem);
	return ret;
}

static int rei2c_close(struct file *filep) {
	iinfo("filep %p\n", filep);
	return OK;
}

static ssize_t rei2c_read(struct file *filep, char *buffer, size_t buflen) {
	iinfo("filep %p buffer %p buflen %d\n", filep, buffer, buflen);
	return 0;
}

static ssize_t rei2c_write(struct file *filep, const char *buffer, size_t buflen) {
	iinfo("filep %p buffer %p buflen %d\n", filep, buffer, buflen);
	return buflen;
}

static int rei2c_ioctl(struct file *filep, int cmd, unsigned long arg) {
	iinfo("filep %p cmd %d arg %08x\n", filep, cmd, arg);
	return OK;
}

#ifndef CONFIG_DISABLE_POLL
static int rei2c_poll(struct file *filep, struct pollfd *fds, bool setup) {
	iinfo("filep %p fds %p setup %d\n", filep, fds, setup);
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

int rei2c_register(const char *devname, struct i2c_master_s *i2c, const struct rei2c_cfg *cfg) {
	struct rei2c_dev_s *dev;
	int ret;

	iinfo("%s\n", devname);

	DEBUGASSERT(devname && i2c);

	// Allocate a new rei2c driver instance
	dev = (struct rei2c_dev_s *)kmm_zalloc(sizeof(struct rei2c_dev_s));
	if (!dev) {
		ierr("ERROR: Failed to allocate device structure\n");
		return -ENOMEM;
	}
	// Save the i2c device
	dev->i2c = i2c;
	dev->addr = REI2C_ADDR;
	dev->cfg = cfg;

	// initialize the new rei2c driver instance
	nxsem_init(&dev->sem, 0, 1);

	// register the rei2c driver
	ret = register_driver(devname, &rei2c_fops, 0666, dev);
	if (ret < 0) {
		ierr("ERROR: register_driver failed: %d\n", ret);
		goto exit;
	}

	return OK;

 exit:
	nxsem_destroy(&dev->sem);
	kmm_free(dev);
	return ret;
}

//-----------------------------------------------------------------------------
