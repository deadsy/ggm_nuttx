/****************************************************************************
 * drivers/audio/adau1961.c
 * Audio device driver for Analog Devices ADAU1961 Stereo CODEC.
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
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <syslog.h>

#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/adau1961.h>

#include "adau1961.h"

/****************************************************************************/
/* ADAU1961 Register Addresses
 * Note: The addresses are all 0x40xx.
 * The leading 0x40 is handled in the code.
 */

#define ADAU1961_REG_Clock_Ctl              0x00
#define ADAU1961_REG_PLL_Ctl                0x02        /* 6 bytes */
#define ADAU1961_REG_Mic_Jack_Detect        0x08
#define ADAU1961_REG_Rec_Power_Mgmt         0x09
#define ADAU1961_REG_Rec_Mixer_Left0        0x0A
#define ADAU1961_REG_Rec_Mixer_Left1        0x0B
#define ADAU1961_REG_Rec_Mixer_Right0       0x0C
#define ADAU1961_REG_Rec_Mixer_Right1       0x0D
#define ADAU1961_REG_Left_Diff_Input_Vol    0x0E
#define ADAU1961_REG_Right_Diff_Input_Vol   0x0F
#define ADAU1961_REG_Record_Mic_Bias        0x10
#define ADAU1961_REG_ALC0                   0x11
#define ADAU1961_REG_ALC1                   0x12
#define ADAU1961_REG_ALC2                   0x13
#define ADAU1961_REG_ALC3                   0x14
#define ADAU1961_REG_Serial_Port0           0x15
#define ADAU1961_REG_Serial_Port1           0x16
#define ADAU1961_REG_Converter0             0x17
#define ADAU1961_REG_Converter1             0x18
#define ADAU1961_REG_ADC_Ctl                0x19
#define ADAU1961_REG_Left_Digital_Vol       0x1A
#define ADAU1961_REG_Right_Digital_Vol      0x1B
#define ADAU1961_REG_Play_Mixer_Left0       0x1C
#define ADAU1961_REG_Play_Mixer_Left1       0x1D
#define ADAU1961_REG_Play_Mixer_Right0      0x1E
#define ADAU1961_REG_Play_Mixer_Right1      0x1F
#define ADAU1961_REG_Play_LR_Mixer_Left     0x20
#define ADAU1961_REG_Play_LR_Mixer_Right    0x21
#define ADAU1961_REG_Play_LR_Mixer_Mono     0x22
#define ADAU1961_REG_Play_HP_Left_Vol       0x23
#define ADAU1961_REG_Play_HP_Right_Vol      0x24
#define ADAU1961_REG_Line_Output_Left_Vol   0x25
#define ADAU1961_REG_Line_Output_Right_Vol  0x26
#define ADAU1961_REG_Play_Mono_Output       0x27
#define ADAU1961_REG_Pop_Click_Suppress     0x28
#define ADAU1961_REG_Play_Power_Mgmt        0x29
#define ADAU1961_REG_DAC_Ctl0               0x2A
#define ADAU1961_REG_DAC_Ctl1               0x2B
#define ADAU1961_REG_DAC_Ctl2               0x2C
#define ADAU1961_REG_Serial_Port_Pad        0x2D
#define ADAU1961_REG_Ctl_Port_Pad0          0x2F
#define ADAU1961_REG_Ctl_Port_Pad1          0x30
#define ADAU1961_REG_Jack_Detect_Pin        0x31
#define ADAU1961_REG_Dejitter_Ctl           0x36

/****************************************************************************/

struct adau1961_dev_s
{
  struct audio_lowerhalf_s dev; /* ADAU1961 audio lower half (this device) */

  /* Our specific driver data goes here */
  FAR const struct adau1961_lower_s *lower;     /* Low-level board specific functions */
  FAR struct i2c_master_s *i2c; /* I2C driver to use */
  FAR struct i2s_dev_s *i2s;    /* I2S driver to use */
  struct dq_queue_s pendq;      /* Queue of pending buffers to be sent */
  struct dq_queue_s doneq;      /* Queue of sent buffers to be returned */
  sem_t pendsem;                /* Protect pendq */

};

/****************************************************************************
 * i2c read/write routines
 */

/* read n bytes at a device address */
static int adau1961_rdbuf(FAR struct adau1961_dev_s *priv, uint8_t addr,
                          FAR uint8_t * buf, size_t n)
{
  uint8_t regaddr[2] = { 0x40, addr };
  struct i2c_msg_s msg[2];
  int retries;

  /* Write the device register */
  msg[0].frequency = priv->lower->frequency;
  msg[0].addr = priv->lower->address;
  msg[0].flags = 0;
  msg[0].buffer = regaddr;
  msg[0].length = sizeof(regaddr);

  /* Read the data buffer */
  msg[1].frequency = priv->lower->frequency;
  msg[1].addr = priv->lower->address;
  msg[1].flags = I2C_M_READ;
  msg[1].buffer = buf;
  msg[1].length = n;

  /* Try up to three times to read the register */
  for (retries = 1; retries <= 3; retries++)
    {
      int ret = I2C_TRANSFER(priv->i2c, msg, 2);
      if (ret < 0)
        {
#ifdef CONFIG_I2C_RESET
          /* Perhaps the I2C bus is locked up?  Try resetting the bus. */
          audwarn("I2C_TRANSFER failed %d (resetting)\n", ret);
          ret = I2C_RESET(priv->i2c);
          if (ret < 0)
            {
              auderr("I2C_RESET failed %d\n", ret);
              return -1;
            }
#else
          auderr("I2C_TRANSFER failed %d (retrying)\n", ret);
#endif
        }
      else
        {
          /* done */
          return 0;
        }
    }

  return -1;
}

/* write n bytes at a device address */
static int adau1961_wrbuf(FAR struct adau1961_dev_s *priv, uint8_t addr,
                          FAR const uint8_t * buf, size_t n)
{
  uint8_t regaddr[2] = { 0x40, addr };
  struct i2c_msg_s msg[2];
  int retries;

  /* Write the device register */
  msg[0].frequency = priv->lower->frequency;
  msg[0].addr = priv->lower->address;
  msg[0].flags = 0;
  msg[0].buffer = regaddr;
  msg[0].length = sizeof(regaddr);

  /* Write the data buffer */
  msg[1].frequency = priv->lower->frequency;
  msg[1].addr = priv->lower->address;
  msg[1].flags = I2C_M_NOSTART;
  msg[1].buffer = (FAR uint8_t *) buf;
  msg[1].length = n;

  /* Try up to three times to read the register */
  for (retries = 1; retries <= 3; retries++)
    {
      int ret = I2C_TRANSFER(priv->i2c, msg, 2);
      if (ret < 0)
        {
#ifdef CONFIG_I2C_RESET
          /* Perhaps the I2C bus is locked up?  Try resetting the bus. */
          audwarn("I2C_TRANSFER failed %d (resetting)\n", ret);
          ret = I2C_RESET(priv->i2c);
          if (ret < 0)
            {
              auderr("I2C_RESET failed %d\n", ret);
              return -1;
            }
#else
          auderr("I2C_TRANSFER failed %d (retrying)\n", ret);
#endif
        }
      else
        {
          /* done */
          return 0;
        }
    }

  return -1;
}

/* read 1 byte at a device address */
static int adau1961_rd(FAR struct adau1961_dev_s *priv, uint8_t addr,
                       FAR uint8_t * val)
{
  return adau1961_rdbuf(priv, addr, val, 1);
}

/* write 1 byte at a device address */
static int adau1961_wr(FAR struct adau1961_dev_s *priv, uint8_t addr,
                       uint8_t val)
{
  return adau1961_wrbuf(priv, addr, &val, 1);
}

/****************************************************************************/

struct adau1961_regdump_s
{
  const char *name;
  uint8_t addr;
};

static const struct adau1961_regdump_s g_adau1961_debug[] = {
  {"Clock_Ctl", ADAU1961_REG_Clock_Ctl},
  {"PLL_Ctl", ADAU1961_REG_PLL_Ctl},
  {"Mic_Jack_Detect", ADAU1961_REG_Mic_Jack_Detect},
  {"Rec_Power_Mgmt", ADAU1961_REG_Rec_Power_Mgmt},
  {"Rec_Mixer_Left0", ADAU1961_REG_Rec_Mixer_Left0},
  {"Rec_Mixer_Left1", ADAU1961_REG_Rec_Mixer_Left1},
  {"Rec_Mixer_Right0", ADAU1961_REG_Rec_Mixer_Right0},
  {"Rec_Mixer_Right1", ADAU1961_REG_Rec_Mixer_Right1},
  {"Left_Diff_Input_Vol", ADAU1961_REG_Left_Diff_Input_Vol},
  {"Right_Diff_Input_Vol", ADAU1961_REG_Right_Diff_Input_Vol},
  {"Record_Mic_Bias", ADAU1961_REG_Record_Mic_Bias},
  {"ALC0", ADAU1961_REG_ALC0},
  {"ALC1", ADAU1961_REG_ALC1},
  {"ALC2", ADAU1961_REG_ALC2},
  {"ALC3", ADAU1961_REG_ALC3},
  {"Serial_Port0", ADAU1961_REG_Serial_Port0},
  {"Serial_Port1", ADAU1961_REG_Serial_Port1},
  {"Converter0", ADAU1961_REG_Converter0},
  {"Converter1", ADAU1961_REG_Converter1},
  {"ADC_Ctl", ADAU1961_REG_ADC_Ctl},
  {"Left_Digital_Vol", ADAU1961_REG_Left_Digital_Vol},
  {"Right_Digital_Vol", ADAU1961_REG_Right_Digital_Vol},
  {"Play_Mixer_Left0", ADAU1961_REG_Play_Mixer_Left0},
  {"Play_Mixer_Left1", ADAU1961_REG_Play_Mixer_Left1},
  {"Play_Mixer_Right0", ADAU1961_REG_Play_Mixer_Right0},
  {"Play_Mixer_Right1", ADAU1961_REG_Play_Mixer_Right1},
  {"Play_LR_Mixer_Left", ADAU1961_REG_Play_LR_Mixer_Left},
  {"Play_LR_Mixer_Right", ADAU1961_REG_Play_LR_Mixer_Right},
  {"Play_LR_Mixer_Mono", ADAU1961_REG_Play_LR_Mixer_Mono},
  {"Play_HP_Left_Vol", ADAU1961_REG_Play_HP_Left_Vol},
  {"Play_HP_Right_Vol", ADAU1961_REG_Play_HP_Right_Vol},
  {"Line_Output_Left_Vol", ADAU1961_REG_Line_Output_Left_Vol},
  {"Line_Output_Right_Vol", ADAU1961_REG_Line_Output_Right_Vol},
  {"Play_Mono_Output", ADAU1961_REG_Play_Mono_Output},
  {"Pop_Click_Suppress", ADAU1961_REG_Pop_Click_Suppress},
  {"Play_Power_Mgmt", ADAU1961_REG_Play_Power_Mgmt},
  {"DAC_Ctl0", ADAU1961_REG_DAC_Ctl0},
  {"DAC_Ctl1", ADAU1961_REG_DAC_Ctl1},
  {"DAC_Ctl2", ADAU1961_REG_DAC_Ctl2},
  {"Serial_Port_Pad", ADAU1961_REG_Serial_Port_Pad},
  {"Ctl_Port_Pad0", ADAU1961_REG_Ctl_Port_Pad0},
  {"Ctl_Port_Pad1", ADAU1961_REG_Ctl_Port_Pad1},
  {"Jack_Detect_Pin", ADAU1961_REG_Jack_Detect_Pin},
  {"Dejitter_Ctl", ADAU1961_REG_Dejitter_Ctl},
};

#define ADAU1961_NREGISTERS (sizeof(g_adau1961_debug)/sizeof(struct adau1961_regdump_s))

static void adau1961_dump_registers(FAR struct audio_lowerhalf_s *dev)
{
  syslog(LOG_INFO, "ADAU1961 Registers:\n");
  for (int i = 0; i < ADAU1961_NREGISTERS; i++)
    {
      const char *name = g_adau1961_debug[i].name;
      uint8_t addr = g_adau1961_debug[i].addr;
      if (addr == ADAU1961_REG_PLL_Ctl)
        {
          /*6 byte register */
          uint8_t val[6] = { 0, 0, 0, 0, 0, 0 };
          adau1961_rdbuf((struct adau1961_dev_s *)dev, addr, val, 6);
          syslog(LOG_INFO, "%21s[40%02x]: %02x %02x %02x %02x %02x %02x\n",
                 name, addr, val[0], val[1], val[2], val[3], val[4], val[5]);
        }
      else
        {
          /*1 byte register */
          uint8_t val = 0;
          adau1961_rd((struct adau1961_dev_s *)dev, addr, &val);
          syslog(LOG_INFO, "%21s[40%02x]: %02x\n", name, addr, val);
        }
    }
}

/****************************************************************************/

/* Reset and re-initialize the ADAU1961 */
static int adau1961_reset(FAR struct adau1961_dev_s *priv)
{
  audinfo("\n");
  return OK;
}

/****************************************************************************/

// Get the lower-half device capabilities
static int adau1961_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                            FAR struct audio_caps_s *pCaps)
{
  audinfo("\n");
  return 0;
}

// Shutdown the driver (called after close)
static int adau1961_shutdown(FAR struct audio_lowerhalf_s *dev)
{
  audinfo("\n");
  return 0;
}

#if 0
// Allocate an audio pipeline buffer
static int adau1961_allocbuffer(FAR struct audio_lowerhalf_s *dev,
                                FAR struct audio_buf_desc_s *apb)
{
  audinfo("\n");
  return 0;
}
#endif

#if 0
// Free an audio pipeline buffer
static int adau1961_freebuffer(FAR struct audio_lowerhalf_s *dev,
                               FAR struct audio_buf_desc_s *apb)
{
  audinfo("\n");
  return 0;
}
#endif

// Enqueue a buffer for processing
static int adau1961_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                  FAR struct ap_buffer_s *apb)
{
  audinfo("\n");
  return 0;
}

// Cancel a previously enqueued buffer
static int adau1961_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                                 FAR struct ap_buffer_s *apb)
{
  audinfo("\n");
  return 0;
}

// Driver specific ioctl commands
static int adau1961_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                          unsigned long arg)
{
  audinfo("\n");
  return 0;
}

#if 0
// Driver specific read commands
static int adau1961_read(FAR struct audio_lowerhalf_s *dev, FAR char *buffer,
                         size_t buflen)
{
  audinfo("\n");
  return 0;
}
#endif

#if 0
// Driver specific write commands
static int adau1961_write(FAR struct audio_lowerhalf_s *dev,
                          FAR const char *buffer, size_t buflen)
{
  audinfo("\n");
  return 0;
}
#endif

#ifdef CONFIG_AUDIO_MULTI_SESSION

// Release a session
static int adau1961_release(FAR struct audio_lowerhalf_s *dev,
                            FAR void *session)
{
  audinfo("\n");
  return 0;
}

// Reserve a session
static int adau1961_reserve(FAR struct audio_lowerhalf_s *dev,
                            FAR void **psession)
{
  audinfo("\n");
  return 0;
}

// Resumes audio streaming after a pause
static int adau1961_resume(FAR struct audio_lowerhalf_s *dev, FAR void *session)
{
  audinfo("\n");
  return 0;
}

// Pause the audio stream
static int adau1961_pause(FAR struct audio_lowerhalf_s *dev, FAR void *session)
{
  audinfo("\n");
  return 0;
}

// Start audio streaming
static int adau1961_start(FAR struct audio_lowerhalf_s *dev, FAR void *session)
{
  audinfo("\n");
  return 0;
}

// Stop audio streaming
static int adau1961_stop(FAR struct audio_lowerhalf_s *dev, FAR void *session)
{
  audinfo("\n");
  return 0;
}

// Configure the driver
static int adau1961_configure(FAR struct audio_lowerhalf_s *dev, void *session,
                              FAR const struct audio_caps_s *pCaps)
{
  audinfo("\n");
  return 0;
}

#else

// Release a session
static int adau1961_release(FAR struct audio_lowerhalf_s *dev)
{
  audinfo("\n");
  return 0;
}

// Reserve a session
static int adau1961_reserve(FAR struct audio_lowerhalf_s *dev)
{
  audinfo("\n");
  return 0;
}

// Resumes audio streaming after a pause
static int adau1961_resume(FAR struct audio_lowerhalf_s *dev)
{
  audinfo("\n");
  return 0;
}

// Pause the audio stream
static int adau1961_pause(FAR struct audio_lowerhalf_s *dev)
{
  audinfo("\n");
  return 0;
}

// Start audio streaming
static int adau1961_start(FAR struct audio_lowerhalf_s *dev)
{
  audinfo("\n");
  return 0;
}

// Stop audio streaming
static int adau1961_stop(FAR struct audio_lowerhalf_s *dev)
{
  audinfo("\n");
  return 0;
}

// Configure the driver
static int adau1961_configure(FAR struct audio_lowerhalf_s *dev,
                              FAR const struct audio_caps_s *pCaps)
{
  audinfo("\n");
  return 0;
}

#endif // not CONFIG_AUDIO_MULTI_SESSION

/****************************************************************************/

static const struct audio_ops_s g_audioops = {
  .getcaps = adau1961_getcaps,
  .configure = adau1961_configure,
  .shutdown = adau1961_shutdown,
  .start = adau1961_start,
  .stop = adau1961_stop,
  .pause = adau1961_pause,
  .resume = adau1961_resume,
  .allocbuffer = NULL /* adau1961_allocbuffer */ ,
  .freebuffer = NULL /* adau1961_freebuffer */ ,
  .enqueuebuffer = adau1961_enqueuebuffer,
  .cancelbuffer = adau1961_cancelbuffer,
  .ioctl = adau1961_ioctl,
  .read = NULL /* adau1961_read */ ,
  .write = NULL /* adau1961_write */ ,
  .reserve = adau1961_reserve,
  .release = adau1961_release,
};

/****************************************************************************/

FAR struct audio_lowerhalf_s *adau1961_initialize(FAR struct i2c_master_s *i2c,
                                                  FAR struct i2s_dev_s *i2s,
                                                  FAR const struct
                                                  adau1961_lower_s *lower)
{
  DEBUGASSERT(i2c && i2s && lower);

  /* Allocate a ADAU1961 device structure */
  struct adau1961_dev_s *priv =
    (struct adau1961_dev_s *)kmm_zalloc(sizeof(struct adau1961_dev_s));
  if (priv)
    {
      int rc;

      /* Initialize the ADAU1961 device structure */
      priv->dev.ops = &g_audioops;
      priv->lower = lower;
      priv->i2c = i2c;
      priv->i2s = i2s;

      nxsem_init(&priv->pendsem, 0, 1);
      dq_init(&priv->pendq);
      dq_init(&priv->doneq);

      /* Reset and reconfigure the CODEC */
      rc = adau1961_reset(priv);
      if (rc != 0)
        {
          goto error;
        }
      adau1961_dump_registers(&priv->dev);
      return &priv->dev;
    }

  return NULL;

error:
  nxsem_destroy(&priv->pendsem);
  kmm_free(priv);
  return NULL;
}

/****************************************************************************/
