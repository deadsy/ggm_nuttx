/****************************************************************************
 * include/nuttx/audio/rei2c.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author:  Jason T. Harris <sirmanlypowers@gmail.com>
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

#ifndef __INCLUDE_NUTTX_INPUT_REI2C_H
#define __INCLUDE_NUTTX_INPUT_REI2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/input/ioctl.h>
#include <nuttx/i2c/i2c_master.h>

/* Registers Addresses ******************************************************/

#define REI2C_GCONF             0x00    /* General Configuration (1 byte) */
#define REI2C_GP1CONF           0x01    /* GP 1 Configuration (1 byte) */
#define REI2C_GP2CONF           0x02    /* GP 2 Configuration (1 byte) */
#define REI2C_GP3CONF           0x03    /* GP 3 Configuration (1 byte) */
#define REI2C_INTCONF           0x04    /* INT pin Configuration (1 byte) */
#define REI2C_ESTATUS           0x05    /* Encoder Status (1 byte) */
#define REI2C_I2STATUS          0x06    /* Secondary interrupt status (1 byte) */
#define REI2C_FSTATUS           0x07    /* Fade process status (1 byte) */
#define REI2C_CVAL              0x08    /* Counter Value (4 bytes) */
#define REI2C_CMAX              0x0C    /* Counter Max value (4 bytes) */
#define REI2C_CMIN              0x10    /* Counter Min value (4 bytes) */
#define REI2C_ISTEP             0x14    /* Increment step value (4 bytes) */
#define REI2C_RLED              0x18    /* LED red color intensity (1 byte) */
#define REI2C_GLED              0x19    /* LED green color intensity (1 byte) */
#define REI2C_BLED              0x1A    /* LED blue color intensity (1 byte) */
#define REI2C_GP1REG            0x1B    /* I/O GP1 Register (1 byte) */
#define REI2C_GP2REG            0x1C    /* I/O GP2 Register (1 byte) */
#define REI2C_GP3REG            0x1D    /* I/O GP3 Register (1 byte) */
#define REI2C_ANTBOUNC          0x1E    /* Anti-bouncing period (1 Byte) */
#define REI2C_DPPERIOD          0x1F    /* Double push period (1 Byte) */
#define REI2C_FADERGB           0x20    /* Fade timer RGB Encoder (1 Byte) */
#define REI2C_FADEGP            0x21    /* Fade timer GP ports (1 Byte) */
#define REI2C_EEPROM            0x80    /* EEPROM memory (128 bytes) */

/* REI2C_GCONF bits */

#define REI2C_GCONF_DTYPE       (1 << 0)    /* Data type of the register: CVAL, CMAX, CMIN and ISTEP. */
#define REI2C_GCONF_WRAPE       (1 << 1)    /* Enable counter wrap. */
#define REI2C_GCONF_DIRE        (1 << 2)    /* Direction of the encoder when increment. */
#define REI2C_GCONF_IPUD        (1 << 3)    /* Interrupt Pull-UP disable. */
#define REI2C_GCONF_RMOD        (1 << 4)    /* Reading Mode. */
#define REI2C_GCONF_ETYPE       (1 << 5)    /* Set the encoder type (normal/illuminated) */
#define REI2C_GCONF_MBANK       (1 << 6)    /* Select the EEPROM memory bank. Each bank is 128 bytes. */
#define REI2C_GCONF_RESET       (1 << 7)    /* Reset the I2C Encoder V2 */

/* REI2C_ESTATUS bits */

#define REI2C_ESTATUS_PUSHR     (1 << 0)    /* push button has been released */
#define REI2C_ESTATUS_PUSHP     (1 << 1)    /* push button has been pressed */
#define REI2C_ESTATUS_PUSHD     (1 << 2)    /* push button has been double pushed */
#define REI2C_ESTATUS_RINC      (1 << 3)    /* rotated in the increase direction */
#define REI2C_ESTATUS_RDEC      (1 << 4)    /* rotated in the decrease direction */
#define REI2C_ESTATUS_RMAX      (1 << 5)    /* maximum counter value has been reached */
#define REI2C_ESTATUS_RMIN      (1 << 6)    /* minimum counter value has been reached */
#define REI2C_ESTATUS_INT2      (1 << 7)    /* Secondary interrupt status */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* status returned by rei2c_read operation */

struct rei2c_status
{
  uint8_t enc;          /* Encoder Status */
  uint8_t int2;         /* Secondary interrupt status */
  uint8_t fade;         /* Fade process status */
  uint32_t cnt;         /* Counter Value */
};

/* rei2c user provided register values */

struct rei2c_regs
{
  uint8_t reg;
  uint32_t val;
};

/* rei2c configuration */

struct rei2c_cfg
{
  uint32_t speed;                   /* i2c bus speed */
  uint8_t addr;                     /* i2c device address */
  const struct rei2c_regs *regs;    /* user provided register values */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int rei2c_register(const char *devname, struct i2c_master_s *i2c,
                   const struct rei2c_cfg *cfg);

#endif /* __INCLUDE_NUTTX_INPUT_REI2C_H */
