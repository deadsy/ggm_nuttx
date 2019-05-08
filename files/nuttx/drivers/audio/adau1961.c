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
#include <gcd.h>

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

  uint32_t sample_rate;         /* Configured samprate (samples/sec) */
  uint8_t n_channels;           /* Number of channels (1 or 2) */
  uint8_t bits_per_sample;      /* Bits per sample (8 or 16) */

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
  int rc;

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

  rc = I2C_TRANSFER(priv->i2c, msg, 2);
  if (rc < 0)
    {
      auderr("adau1961_rdbuf failed %d\n", rc);
      return -1;
    }
  return 0;
}

/* write n bytes at a device address */
static int adau1961_wrbuf(FAR struct adau1961_dev_s *priv, uint8_t addr,
                          FAR const uint8_t * buf, size_t n)
{
  uint8_t regaddr[2] = { 0x40, addr };
  struct i2c_msg_s msg[2];
  int rc;

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

  rc = I2C_TRANSFER(priv->i2c, msg, 2);
  if (rc < 0)
    {
      auderr("adau1961_wrbuf failed %d\n", rc);
      return -1;
    }
  return 0;
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

/****************************************************************************
 * Name: adau1961_gen_pll
 *
 * Description:
 *   Generate the pll register values.
 */

#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))

int adau1961_gen_pll(uint32_t freq_in, uint32_t freq_out, uint8_t pll[6])
{
  uint32_t r, n, m, i, j;
  uint32_t div;

  if (!freq_out)
    {
      r = 0;
      n = 0;
      m = 0;
      div = 0;
    }
  else
    {
      if (freq_out % freq_in != 0)
        {
          div = DIV_ROUND_UP(freq_in, 13500000);
          freq_in /= div;
          r = freq_out / freq_in;
          i = freq_out % freq_in;
          j = gcd(i, freq_in);
          n = i / j;
          m = freq_in / j;
          div--;
        }
      else
        {
          r = freq_out / freq_in;
          n = 0;
          m = 0;
          div = 0;
        }
      if (n > 0xffff || m > 0xffff || div > 3 || r > 8 || r < 2)
        {
          return -1;
        }
    }

  /* set the pll bytes */
  pll[0] = m >> 8;
  pll[1] = m & 0xff;
  pll[2] = n >> 8;
  pll[3] = n & 0xff;
  pll[4] = (r << 3) | (div << 1);
  if (m != 0)
    {
      pll[4] |= 1;              /* fractional mode */
    }
  pll[5] = 1;                   /* pll enabled */

  return 0;
}

/****************************************************************************
 * Name: adau1961_setpll
 *
 * Description:
 *   Setup the CODEC PLL and enable the core. The PLL settings are a
 *   function of the master clock and the desired sample rate.
 */

static int adau1961_set_pll(FAR struct adau1961_dev_s *priv)
{
  uint32_t mclk = priv->lower->mclk;
  uint32_t sample_rate = priv->sample_rate;
  uint8_t pll[6];
  int rc, i;

  audinfo("mclk %u sample_rate %u\n", mclk, sample_rate);

  /* disable the core */
  rc = adau1961_wr(priv, ADAU1961_REG_Clock_Ctl, 0);
  if (rc < 0)
    {
      return -1;
    }

  /* setup the pll */
  rc = adau1961_gen_pll(priv->lower->mclk, 1024 * priv->sample_rate, pll);
  if (rc < 0)
    {
      auderr("adau1961_gen_pll failed %d\n", rc);
      return -1;
    }

  audinfo("pll %02x %02x %02x %02x %02x %02x\n", pll[0], pll[1], pll[2], pll[3],
          pll[4], pll[5]);

//#if 0
  pll[0] = 0x1F;
  pll[1] = 0x40;
  pll[2] = 0x04;
  pll[3] = 0x81;
  pll[4] = 0x31;
  pll[5] = 0x01;
//#endif

  rc = adau1961_wrbuf(priv, ADAU1961_REG_PLL_Ctl, pll, sizeof(pll));
  if (rc < 0)
    {
      return -1;
    }

  /* wait for the pll to lock */
  i = 100;
  while (i > 0)
    {
      rc = adau1961_rdbuf(priv, ADAU1961_REG_PLL_Ctl, pll, sizeof(pll));
      if (rc < 0)
        {
          return -1;
        }
      if (pll[5] & (1 << 1 /* Locked? */ ))
        {
          break;
        }
      nxsig_usleep(1000);
      i--;
    }
  if (i == 0)
    {
      auderr("codec pll did not lock\n");
      return -1;
    }

  audinfo("lock %d\n", i);

  /* enable the core and clocking via pll */
  rc = adau1961_wr(priv, ADAU1961_REG_Clock_Ctl,
                   (1 << 3 /*pll */ ) | (1 << 0 /*coren */ ));
  if (rc < 0)
    {
      return -1;
    }

  return 0;
}

/****************************************************************************
 * Name: adau1961_init
 *
 * Description:
 *   Set initial values for the CODEC registers.
 */

struct adau1961_regval_s
{
  uint8_t addr;
  uint8_t val;
};

static const struct adau1961_regval_s adau1961_regvals[] = {
  {ADAU1961_REG_Mic_Jack_Detect, 0},
  {ADAU1961_REG_Rec_Power_Mgmt, 0},
  {ADAU1961_REG_Rec_Mixer_Left0, 0},
  {ADAU1961_REG_Rec_Mixer_Left1, 0},
  {ADAU1961_REG_Rec_Mixer_Right0, 0},
  {ADAU1961_REG_Rec_Mixer_Right1, 0},
  {ADAU1961_REG_Left_Diff_Input_Vol, 0},
  {ADAU1961_REG_Right_Diff_Input_Vol, 0},
  {ADAU1961_REG_Record_Mic_Bias, 0},
  {ADAU1961_REG_ALC0, 0},
  {ADAU1961_REG_ALC1, 0},
  {ADAU1961_REG_ALC2, 0},
  {ADAU1961_REG_ALC3, 0},
  {ADAU1961_REG_Serial_Port0, (1 << 0 /*master */ )},
  {ADAU1961_REG_Serial_Port1, 0},
  {ADAU1961_REG_Converter0, 0},
  {ADAU1961_REG_Converter1, 0},
  {ADAU1961_REG_ADC_Ctl, 0},
  {ADAU1961_REG_Left_Digital_Vol, 0},
  {ADAU1961_REG_Right_Digital_Vol, 0},
  {ADAU1961_REG_Play_Mixer_Left0, 0},
  {ADAU1961_REG_Play_Mixer_Left1, 0},
  {ADAU1961_REG_Play_Mixer_Right0, 0},
  {ADAU1961_REG_Play_Mixer_Right1, 0},
  {ADAU1961_REG_Play_LR_Mixer_Left, 0},
  {ADAU1961_REG_Play_LR_Mixer_Right, 0},
  {ADAU1961_REG_Play_LR_Mixer_Mono, 0},
  {ADAU1961_REG_Play_HP_Left_Vol, 0},
  {ADAU1961_REG_Play_HP_Right_Vol, 0},
  {ADAU1961_REG_Line_Output_Left_Vol, 0},
  {ADAU1961_REG_Line_Output_Right_Vol, 0},
  {ADAU1961_REG_Play_Mono_Output, 0},
  {ADAU1961_REG_Pop_Click_Suppress, 0},
  {ADAU1961_REG_Play_Power_Mgmt, 0},
  {ADAU1961_REG_DAC_Ctl0, 0},
  {ADAU1961_REG_DAC_Ctl1, 0},
  {ADAU1961_REG_DAC_Ctl2, 0},
  {ADAU1961_REG_Serial_Port_Pad, 0},
  {ADAU1961_REG_Ctl_Port_Pad0, 0},
  {ADAU1961_REG_Ctl_Port_Pad1, 0},
  {ADAU1961_REG_Jack_Detect_Pin, 0},
  {ADAU1961_REG_Dejitter_Ctl, 0},
  {0xff, 0}                     /* eol */
};

static int adau1961_init_registers(FAR struct adau1961_dev_s *priv)
{
  const struct adau1961_regval_s *reg = adau1961_regvals;
  int i = 0;

  while (reg[i].addr != 0xff)
    {
      int rc = adau1961_wr(priv, reg[i].addr, reg[i].val);
      if (rc < 0)
        {
          return -1;
        }
      i++;
    }

  return 0;
}

/****************************************************************************
 * Name: adau1961_reset
 *
 * Description:
 *   Reset and re-initialize the ADAU1961.
 */

static int adau1961_reset(FAR struct adau1961_dev_s *priv)
{
  int rc;

  /*set some default values */
  priv->sample_rate = 48000;
  priv->n_channels = 2;
  priv->bits_per_sample = 16;

  rc = adau1961_set_pll(priv);
  if (rc < 0)
    {
      auderr("adau1961_set_pll failed %d\n", rc);
      return -1;
    }

  rc = adau1961_init_registers(priv);
  if (rc < 0)
    {
      auderr("adau1961_init_registers failed %d\n", rc);
      return -1;
    }

  return 0;
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
