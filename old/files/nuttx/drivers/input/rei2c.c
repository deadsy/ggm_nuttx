/****************************************************************************
 * drivers/input/rei2c.c
 * I2C Rotary Encoder V2 Driver
 * https://github.com/Fattoresaimon/I2CEncoderV2
 * https://www.kickstarter.com/projects/1351830006/i2c-encoder-v2
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Jason T Harris <sirmnalypowers@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/input/rei2c.h>

/****************************************************************************/

struct rei2c_dev_s
{
  struct i2c_master_s *i2c;     // I2C interface connected to rei2c
  sem_t sem;                    // Supports exclusive access to the device
  uint32_t speed;               // i2c bus speed
  uint8_t addr;                 // i2c device address
  uint8_t crefs;                // Number of times the device has been opened
};

/****************************************************************************/
// basic read/write functions

static int rei2c_i2c_read(struct rei2c_dev_s *dev, uint8_t reg, uint8_t * data,
                          size_t len)
{
  struct i2c_msg_s msgv[2] = {
    {
     .frequency = dev->speed,
     .addr = dev->addr,
     .flags = 0,
     .buffer = &reg,
     .length = 1,
     },
    {
     .frequency = dev->speed,
     .addr = dev->addr,
     .flags = I2C_M_READ,
     .buffer = data,
     .length = len,
     }
  };
  int rc = I2C_TRANSFER(dev->i2c, msgv, 2);
  if (rc < 0)
    {
      ierr("I2C_TRANSFER failed %d\n", rc);
      return rc;
    }
  return OK;
}

static int rei2c_i2c_write(struct rei2c_dev_s *dev, uint8_t reg, uint8_t * data,
                           size_t len)
{
  struct i2c_msg_s msgv[2] = {
    {
     .frequency = dev->speed,
     .addr = dev->addr,
     .flags = 0,
     .buffer = &reg,
     .length = 1,
     },
    {
     .frequency = dev->speed,
     .addr = dev->addr,
     .flags = I2C_M_NOSTART,
     .buffer = data,
     .length = len,
     }
  };
  int rc = I2C_TRANSFER(dev->i2c, msgv, 2);
  if (rc < 0)
    {
      ierr("I2C_TRANSFER failed %d\n", rc);
      return rc;
    }
  return OK;
}

static int rei2c_rd8(struct rei2c_dev_s *dev, uint8_t reg, uint8_t * val)
{
  return rei2c_i2c_read(dev, reg, val, 1);
}

static int rei2c_rd24(struct rei2c_dev_s *dev, uint8_t reg, uint32_t * val)
{
  return rei2c_i2c_read(dev, reg, (uint8_t *) val, 3);
}

static int rei2c_rd32(struct rei2c_dev_s *dev, uint8_t reg, uint32_t * val)
{
  return rei2c_i2c_read(dev, reg, (uint8_t *) val, 4);
}

static int rei2c_wr8(struct rei2c_dev_s *dev, uint8_t reg, uint8_t val)
{
  return rei2c_i2c_write(dev, reg, &val, 1);
}

static int rei2c_wr24(struct rei2c_dev_s *dev, uint8_t reg, uint32_t val)
{
  return rei2c_i2c_write(dev, reg, (uint8_t *) & val, 3);
}

static int rei2c_wr32(struct rei2c_dev_s *dev, uint8_t reg, uint32_t val)
{
  return rei2c_i2c_write(dev, reg, (uint8_t *) & val, 4);
}

/****************************************************************************/

static inline int rei2c_takesem(sem_t * sem)
{
  int rc;
  // Take a count from the semaphore, possibly waiting */
  rc = nxsem_wait(sem);
  // The only case that an error should occur here is if the wait was awakened by a signal
  DEBUGASSERT(rc == OK || rc == -EINTR);
  return rc;
}

static inline int rei2c_givesem(sem_t * sem)
{
  return nxsem_post(sem);
}

/****************************************************************************/

static int rei2c_init(struct rei2c_dev_s *dev, const struct rei2c_regs *regs)
{
  int rc;

  rc = rei2c_wr8(dev, REI2C_GCONF, REI2C_GCONF_RESET);
  if (rc < 0)
    {
      ierr("device reset failed %d\n", rc);
      goto exit;
    }
  // wait > 400 usecs
  nxsig_usleep(800);

  // check some register values
  uint8_t val0, val1;
  rei2c_rd8(dev, REI2C_GP1CONF, &val0);
  rei2c_rd8(dev, REI2C_ANTBOUNC, &val1);
  if ((val0 != 0) || (val1 != 25))
    {
      ierr("bad device values\n");
      rc = -1;
      goto exit;
    }
  // apply the user register configuration
  if (regs != NULL)
    {
      int idx = 0;
      while (regs[idx].reg != 0xff)
        {
          uint8_t reg = regs[idx].reg;
          uint32_t val = regs[idx].val;
          if ((reg == REI2C_CVAL) || (reg == REI2C_CMAX) ||
              (reg == REI2C_CMIN) || (reg == REI2C_ISTEP))
            {
              rei2c_wr32(dev, reg, val);
            }
          else
            {
              rei2c_wr8(dev, reg, (uint8_t) val);
            }
          idx += 1;
        }
    }

exit:
  return rc;
}

/****************************************************************************/
// file operations

static int rei2c_open(struct file *filep)
{
  iinfo("filep %p\n", filep);

  DEBUGASSERT(filep && filep->f_inode);
  struct inode *inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  struct rei2c_dev_s *dev = (struct rei2c_dev_s *)inode->i_private;

  int ret = rei2c_takesem(&dev->sem);
  if (ret < 0)
    {
      ierr("ERROR: rei2c_takesem failed: %d\n", ret);
      return ret;
    }
  // Increment the reference count
  uint8_t tmp = dev->crefs + 1;
  if (tmp == 0)
    {
      // More than 255 opens; uint8_t overflows to zero
      ret = -EMFILE;
      goto exit;
    }
  dev->crefs = tmp;

exit:
  rei2c_givesem(&dev->sem);
  return ret;
}

static int rei2c_close(struct file *filep)
{
  iinfo("filep %p\n", filep);

  DEBUGASSERT(filep && filep->f_inode);
  struct inode *inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  struct rei2c_dev_s *dev = (struct rei2c_dev_s *)inode->i_private;

  int ret = rei2c_takesem(&dev->sem);
  if (ret < 0)
    {
      ierr("ERROR: rei2c_takesem failed: %d\n", ret);
      return ret;
    }
  // Decrement the reference count
  if (dev->crefs >= 1)
    {
      dev->crefs--;
    }

  rei2c_givesem(&dev->sem);
  return ret;
}

/****************************************************************************/

static int rei2c_rd_status(struct rei2c_dev_s *dev, struct rei2c_status *status)
{
  uint8_t buf[7];
  int ret = rei2c_i2c_read(dev, REI2C_ESTATUS, buf, sizeof(buf));
  if (ret < 0)
    {
      goto exit;
    }
  status->enc = buf[0];
  status->int2 = buf[1];
  status->fade = buf[2];
  status->cnt = *(uint32_t *) & buf[3];
exit:
  return ret;
}

static ssize_t rei2c_read(struct file *filep, char *buffer, size_t buflen)
{
  iinfo("filep %p buffer %p buflen %d\n", filep, buffer, buflen);

  DEBUGASSERT(filep && filep->f_inode);
  struct inode *inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  struct rei2c_dev_s *dev = (struct rei2c_dev_s *)inode->i_private;

  if (buflen < sizeof(struct rei2c_status))
    {
      ierr("read buffer is too small %d < %d\n", buflen,
           sizeof(struct rei2c_status));
      return -ENOSYS;
    }

  int ret = rei2c_takesem(&dev->sem);
  if (ret < 0)
    {
      ierr("ERROR: rei2c_takesem failed: %d\n", ret);
      return ret;
    }
  // read the status value
  struct rei2c_status *status = (struct rei2c_status *)buffer;
  ret = rei2c_rd_status(dev, status);
  if (ret >= 0)
    {
      iinfo("enc %02x int2 %02x fade %02x cnt %08x\n", status->enc,
            status->int2, status->fade, status->cnt);
      ret = sizeof(struct rei2c_status);
    }

  rei2c_givesem(&dev->sem);
  return ret;
}

/****************************************************************************/

static ssize_t rei2c_write(struct file *filep, const char *buffer,
                           size_t buflen)
{
  iinfo("filep %p buffer %p buflen %d\n", filep, buffer, buflen);
  return buflen;
}

static int rei2c_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  iinfo("filep %p cmd %d arg %08x\n", filep, cmd, arg);
  return OK;
}

#ifndef CONFIG_DISABLE_POLL
static int rei2c_poll(struct file *filep, struct pollfd *fds, bool setup)
{
  iinfo("filep %p fds %p setup %d\n", filep, fds, setup);
  return OK;
}
#endif // CONFIG_DISABLE_POLL

static const struct file_operations rei2c_fops = {
  rei2c_open,                   // open
  rei2c_close,                  // close
  rei2c_read,                   // read
  rei2c_write,                  // write
  NULL,                         // seek
  rei2c_ioctl,                  // ioctl
#ifndef CONFIG_DISABLE_POLL
  rei2c_poll,                   // poll
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  NULL,                         // unlink
#endif
};

/****************************************************************************/

int rei2c_register(const char *devname, struct i2c_master_s *i2c,
                   const struct rei2c_cfg *cfg)
{
  struct rei2c_dev_s *dev;
  int rc;

  iinfo("%s\n", devname);

  DEBUGASSERT(devname && i2c && cfg);

  // Allocate a new rei2c driver instance
  dev = (struct rei2c_dev_s *)kmm_zalloc(sizeof(struct rei2c_dev_s));
  if (!dev)
    {
      ierr("failed to allocate device structure\n");
      return -ENOMEM;
    }
  // setup the device structure
  dev->i2c = i2c;
  dev->speed = cfg->speed;
  dev->addr = cfg->addr;
  nxsem_init(&dev->sem, 0, 1);

  // initialize the hardware
  rc = rei2c_init(dev, cfg->regs);
  if (rc < 0)
    {
      ierr("rei2c_init failed %d\n", rc);
      goto exit;
    }
  // register the rei2c driver
  rc = register_driver(devname, &rei2c_fops, 0666, dev);
  if (rc < 0)
    {
      ierr("register_driver failed %d\n", rc);
      goto exit;
    }

  return OK;

exit:
  nxsem_destroy(&dev->sem);
  kmm_free(dev);
  return rc;
}

/****************************************************************************/
