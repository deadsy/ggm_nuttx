/************************************************************************************
 * configs/axoloti/src/stm32_adau1361.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Jason T. Harris <sirmanlypowers@gmail.com>
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
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/audio/adau1361.h>

#include <arch/board/board.h>

#include "stm32.h"
#include "axoloti.h"

/************************************************************************************/

static void adau1361_hw_reset(FAR const struct adau1361_lower_s *lower)
{
  // TODO...
}

/****************************************************************************
 * Name: stm32_adau1361_initialize
 *
 * Description:
 *   This function is called by platform-specific, setup logic to configure
 *   and register the ADAU1361 device.  This function will register the driver
 *   as /dev/audio/pcm[x] where x is determined by the minor device number.
 *
 * Input Parameters:
 *   minor - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int stm32_adau1361_initialize(int minor)
{
  FAR struct audio_lowerhalf_s *adau1361;
  FAR struct audio_lowerhalf_s *pcm;
  FAR struct i2c_master_s *i2c;
  FAR struct i2s_dev_s *i2s;
  static bool initialized = false;
  char devname[12];
  int ret;

  audinfo("minor %d\n", minor);
  DEBUGASSERT(minor >= 0 && minor <= 25);

  /* Have we already initialized?  Since we never uninitialize we must prevent
   * multiple initializations.  This is necessary, for example, when the
   * touchscreen example is used as a built-in application in NSH and can be
   * called numerous time.  It will attempt to initialize each time.
   */

  if (!initialized)
    {
      /* Get an instance of the I2C interface for the ADAU1361 device */
      i2c = stm32_i2cbus_initialize(ADAU1361_I2C_BUS);
      if (!i2c)
        {
          auderr("ERROR: Failed to initialize TWI%d\n", ADAU1361_I2C_BUS);
          ret = -ENODEV;
          goto errout;
        }

      /* Get an instance of the I2S interface for the ADAU1361 data channel */
      i2s = stm32_i2sdev_initialize(ADAU1361_I2S_BUS);
      if (!i2s)
        {
          auderr("ERROR: Failed to initialize I2S%d\n", ADAU1361_I2S_BUS);
          ret = -ENODEV;
          goto errout_with_i2c;
        }

      /* Now we can use these I2C and I2S interfaces to initialize the
       * ADAU1361 which will return an audio interface.
       */
      adau1361 = adau1361_initialize(i2c, i2s, &g_adau1361info.lower);
      if (!cs43l22)
        {
          auderr("ERROR: Failed to initialize the ADAU1361\n");
          ret = -ENODEV;
          goto error;
        }

      /* Now we embed the ADAU1361/I2C/I2S conglomerate into a PCM decoder
       * instance so that we will have a PCM front end for the the ADAU1361
       * driver.
       */
      pcm = pcm_decode_initialize(adau1361);
      if (!pcm)
        {
          auderr("ERROR: Failed create the PCM decoder\n");
          ret = -ENODEV;
          goto error;
        }

      /* Create a device name */
      snprintf(devname, 12, "pcm%d", minor);

      /* Finally, we can register the PCM/ADAU1361/I2C/I2S audio device. */
      ret = audio_register(devname, pcm);
      if (ret < 0)
        {
          auderr("ERROR: Failed to register /dev/%s device: %d\n", devname,
                 ret);
          goto errout_with_pcm;
        }

      /* Now we are initialized */
      initialized = true;
    }

  return OK;

  /* Error exits.  Unfortunately there is no mechanism in place now to
   * recover resources from most errors on initialization failures.
   */

error:
  return ret;
}

/************************************************************************************/
