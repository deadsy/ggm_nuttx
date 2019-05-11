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
#include <stdio.h>
#include <fcntl.h>
#include <syslog.h>
#include <gcd.h>

#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/adau1961.h>

#include "adau1961.h"

#ifdef CONFIG_AUDIO_MULTI_SESSION
#error "CONFIG_AUDIO_MULTI_SESSION not supported"
#endif

/****************************************************************************/
/* ADAU1961 Register Addresses
 * Note: The addresses are all 0x40xx.
 * The leading 0x40 is handled in the code.
 */

#define ADAU1961_R0_Clock_Ctl              0x00
#define ADAU1961_R1_PLL_Ctl                0x02 /* 6 bytes */
#define ADAU1961_R2_Mic_Jack_Detect        0x08
#define ADAU1961_R3_Rec_Power_Mgmt         0x09
#define ADAU1961_R4_Rec_Mixer_Left0        0x0A
#define ADAU1961_R5_Rec_Mixer_Left1        0x0B
#define ADAU1961_R6_Rec_Mixer_Right0       0x0C
#define ADAU1961_R7_Rec_Mixer_Right1       0x0D
#define ADAU1961_R8_Left_Diff_Input_Vol    0x0E
#define ADAU1961_R9_Right_Diff_Input_Vol   0x0F
#define ADAU1961_R10_Record_Mic_Bias        0x10
#define ADAU1961_R11_ALC0                   0x11
#define ADAU1961_R12_ALC1                   0x12
#define ADAU1961_R13_ALC2                   0x13
#define ADAU1961_R14_ALC3                   0x14
#define ADAU1961_R15_Serial_Port0           0x15
#define ADAU1961_R16_Serial_Port1           0x16
#define ADAU1961_R17_Converter0             0x17
#define ADAU1961_R18_Converter1             0x18
#define ADAU1961_R19_ADC_Ctl                0x19
#define ADAU1961_R20_Left_Digital_Vol       0x1A
#define ADAU1961_R21_Right_Digital_Vol      0x1B
#define ADAU1961_R22_Play_Mixer_Left0       0x1C
#define ADAU1961_R23_Play_Mixer_Left1       0x1D
#define ADAU1961_R24_Play_Mixer_Right0      0x1E
#define ADAU1961_R25_Play_Mixer_Right1      0x1F
#define ADAU1961_R26_Play_LR_Mixer_Left     0x20
#define ADAU1961_R27_Play_LR_Mixer_Right    0x21
#define ADAU1961_R28_Play_LR_Mixer_Mono     0x22
#define ADAU1961_R29_Play_HP_Left_Vol       0x23
#define ADAU1961_R30_Play_HP_Right_Vol      0x24
#define ADAU1961_R31_Line_Output_Left_Vol   0x25
#define ADAU1961_R32_Line_Output_Right_Vol  0x26
#define ADAU1961_R33_Play_Mono_Output       0x27
#define ADAU1961_R34_Pop_Click_Suppress     0x28
#define ADAU1961_R35_Play_Power_Mgmt        0x29
#define ADAU1961_R36_DAC_Ctl0               0x2A
#define ADAU1961_R37_DAC_Ctl1               0x2B
#define ADAU1961_R38_DAC_Ctl2               0x2C
#define ADAU1961_R39_Serial_Port_Pad        0x2D
#define ADAU1961_R40_Ctl_Port_Pad0          0x2F
#define ADAU1961_R41_Ctl_Port_Pad1          0x30
#define ADAU1961_R42_Jack_Detect_Pin        0x31
#define ADAU1961_R67_Dejitter_Ctl           0x36

#define CONVSR_DIV1 0           /* fs/1 */
#define CONVSR_DIV6 1           /* fs/6 */
#define CONVSR_DIV4 2           /* fs/4 */
#define CONVSR_DIV3 3           /* fs/3 */
#define CONVSR_DIV2 4           /* fs/2 */
#define CONVSR_DIV1_5 5         /* fs/1.5 */
#define CONVSR_DIV0_5 6         /* fs/0.5 */

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
  mqd_t mq;                     /* Message queue for receiving messages */
  char mqname[16];              /* Our message queue name */
  pthread_t threadid;           /* ID of our thread */
  sem_t pendsem;                /* Protect pendq */
  uint32_t sample_rate;         /* Configured samprate (samples/sec) */
  uint8_t n_channels;           /* Number of channels (1 or 2) */
  uint8_t bits_per_sample;      /* Bits per sample (8 or 16) */
  uint16_t volume;              /*line out volume 0..1000 */
  uint16_t balance;             /*line out balance 0..1000, 0 = left, 1000 = right */
  volatile uint8_t inflight;    /* Number of audio buffers in-flight */
  bool terminating;             /* True: Stop requested */
  bool reserved;                /* True: Device is reserved */
  bool running;                 /* True: Worker thread is running */
  bool paused;                  /* True: Playing is paused */
  bool mute;                    /* True: Output is muted */
  volatile int result;          /* The result of the last transfer */
};

/************************************************************************************
 * Name: adau1961_takesem
 *
 * Description:
 *  Take a semaphore count, handling the nasty EINTR return if we are interrupted
 *  by a signal.
 */

static void adau1961_takesem(sem_t * sem)
{
  int ret;
  do
    {
      ret = nxsem_wait(sem);
      DEBUGASSERT(ret == 0 || ret == -EINTR);
    }
  while (ret == -EINTR);
}

#define adau1961_givesem(s) nxsem_post(s)

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

static const struct adau1961_regdump_s g_adau1961_regdump[] = {
  {"R0_Clock_Ctl", ADAU1961_R0_Clock_Ctl},
  {"R1_PLL_Ctl", ADAU1961_R1_PLL_Ctl},
  {"R2_Mic_Jack_Detect", ADAU1961_R2_Mic_Jack_Detect},
  {"R3_Rec_Power_Mgmt", ADAU1961_R3_Rec_Power_Mgmt},
  {"R4_Rec_Mixer_Left0", ADAU1961_R4_Rec_Mixer_Left0},
  {"R5_Rec_Mixer_Left1", ADAU1961_R5_Rec_Mixer_Left1},
  {"R6_Rec_Mixer_Right0", ADAU1961_R6_Rec_Mixer_Right0},
  {"R7_Rec_Mixer_Right1", ADAU1961_R7_Rec_Mixer_Right1},
  {"R8_Left_Diff_Input_Vol", ADAU1961_R8_Left_Diff_Input_Vol},
  {"R9_Right_Diff_Input_Vol", ADAU1961_R9_Right_Diff_Input_Vol},
  {"R10_Record_Mic_Bias", ADAU1961_R10_Record_Mic_Bias},
  {"R11_ALC0", ADAU1961_R11_ALC0},
  {"R12_ALC1", ADAU1961_R12_ALC1},
  {"R13_ALC2", ADAU1961_R13_ALC2},
  {"R14_ALC3", ADAU1961_R14_ALC3},
  {"R15_Serial_Port0", ADAU1961_R15_Serial_Port0},
  {"R16_Serial_Port1", ADAU1961_R16_Serial_Port1},
  {"R17_Converter0", ADAU1961_R17_Converter0},
  {"R18_Converter1", ADAU1961_R18_Converter1},
  {"R19_ADC_Ctl", ADAU1961_R19_ADC_Ctl},
  {"R20_Left_Digital_Vol", ADAU1961_R20_Left_Digital_Vol},
  {"R21_Right_Digital_Vol", ADAU1961_R21_Right_Digital_Vol},
  {"R22_Play_Mixer_Left0", ADAU1961_R22_Play_Mixer_Left0},
  {"R23_Play_Mixer_Left1", ADAU1961_R23_Play_Mixer_Left1},
  {"R24_Play_Mixer_Right0", ADAU1961_R24_Play_Mixer_Right0},
  {"R25_Play_Mixer_Right1", ADAU1961_R25_Play_Mixer_Right1},
  {"R26_Play_LR_Mixer_Left", ADAU1961_R26_Play_LR_Mixer_Left},
  {"R27_Play_LR_Mixer_Right", ADAU1961_R27_Play_LR_Mixer_Right},
  {"R28_Play_LR_Mixer_Mono", ADAU1961_R28_Play_LR_Mixer_Mono},
  {"R29_Play_HP_Left_Vol", ADAU1961_R29_Play_HP_Left_Vol},
  {"R30_Play_HP_Right_Vol", ADAU1961_R30_Play_HP_Right_Vol},
  {"R31_Line_Output_Left_Vol", ADAU1961_R31_Line_Output_Left_Vol},
  {"R32_Line_Output_Right_Vol", ADAU1961_R32_Line_Output_Right_Vol},
  {"R33_Play_Mono_Output", ADAU1961_R33_Play_Mono_Output},
  {"R34_Pop_Click_Suppress", ADAU1961_R34_Pop_Click_Suppress},
  {"R35_Play_Power_Mgmt", ADAU1961_R35_Play_Power_Mgmt},
  {"R36_DAC_Ctl0", ADAU1961_R36_DAC_Ctl0},
  {"R37_DAC_Ctl1", ADAU1961_R37_DAC_Ctl1},
  {"R38_DAC_Ctl2", ADAU1961_R38_DAC_Ctl2},
  {"R39_Serial_Port_Pad", ADAU1961_R39_Serial_Port_Pad},
  {"R40_Ctl_Port_Pad0", ADAU1961_R40_Ctl_Port_Pad0},
  {"R41_Ctl_Port_Pad1", ADAU1961_R41_Ctl_Port_Pad1},
  {"R42_Jack_Detect_Pin", ADAU1961_R42_Jack_Detect_Pin},
  {"R67_Dejitter_Ctl", ADAU1961_R67_Dejitter_Ctl},
};

#define ADAU1961_NDUMPS (sizeof(g_adau1961_regdump)/sizeof(struct adau1961_regdump_s))

static void adau1961_dump_registers(FAR struct audio_lowerhalf_s *dev)
{
  syslog(LOG_INFO, "ADAU1961 Registers:\n");
  for (int i = 0; i < ADAU1961_NDUMPS; i++)
    {
      const char *name = g_adau1961_regdump[i].name;
      uint8_t addr = g_adau1961_regdump[i].addr;
      if (addr == ADAU1961_R1_PLL_Ctl)
        {
          /*6 byte register */
          uint8_t val[6] = { 0, 0, 0, 0, 0, 0 };
          adau1961_rdbuf((struct adau1961_dev_s *)dev, addr, val, 6);
          syslog(LOG_INFO, "%25s[40%02x]: %02x %02x %02x %02x %02x %02x\n",
                 name, addr, val[0], val[1], val[2], val[3], val[4], val[5]);
        }
      else
        {
          /*1 byte register */
          uint8_t val = 0;
          adau1961_rd((struct adau1961_dev_s *)dev, addr, &val);
          syslog(LOG_INFO, "%25s[40%02x]: %02x\n", name, addr, val);
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

  audinfo("pll[] = {%02x %02x %02x %02x %02x %02x}\n",
          pll[0], pll[1], pll[2], pll[3], pll[4], pll[5]);

  return 0;
}

/****************************************************************************
 * Name: adau1961_setpll
 *
 * Description:
 *   Setup the CODEC PLL and enable the core. The PLL settings are a
 *   function of the master clock and the desired sample rate.
 */

static int adau1961_set_pll(FAR struct adau1961_dev_s *priv, uint32_t fs)
{
  uint32_t mclk = priv->lower->mclk;
  uint8_t pll[6];
  int rc, i;

  audinfo("mclk %u fs %u\n", mclk, fs);

  /* disable the core */
  rc = adau1961_wr(priv, ADAU1961_R0_Clock_Ctl, 0);
  if (rc < 0)
    {
      return -1;
    }

  /* setup the pll */
  rc = adau1961_gen_pll(mclk, 1024 * fs, pll);
  if (rc < 0)
    {
      auderr("adau1961_gen_pll failed %d\n", rc);
      return -1;
    }
  rc = adau1961_wrbuf(priv, ADAU1961_R1_PLL_Ctl, pll, sizeof(pll));
  if (rc < 0)
    {
      return -1;
    }

  /* wait for the pll to lock */
  i = 10;
  while (i > 0)
    {
      rc = adau1961_rdbuf(priv, ADAU1961_R1_PLL_Ctl, pll, sizeof(pll));
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

  /* enable the core and clocking via pll */
  rc = adau1961_wr(priv, ADAU1961_R0_Clock_Ctl,
                   (1 << 3 /*pll */ ) | (1 << 0 /*coren */ ));
  if (rc < 0)
    {
      return -1;
    }

  audinfo("pll enabled\n");
  return 0;
}

/****************************************************************************
 * Name: adau1961_set_sample_rate
 *
 * Description:
 *   Setup the PLL for the CODEC based on the sample rate.
 */

struct adau1961_rates_s
{
  uint32_t sr;                  /*desired sample rate */
  uint32_t fs;                  /*base sample rate */
  uint8_t convsr;               /*divider value */
};

static const struct adau1961_rates_s g_adau1961_rates[] = {
  {48000, 48000, CONVSR_DIV1},
  {8000, 48000, CONVSR_DIV6},
  {12000, 48000, CONVSR_DIV4},
  {16000, 48000, CONVSR_DIV3},
  {24000, 48000, CONVSR_DIV2},
  {32000, 48000, CONVSR_DIV1_5},
  {96000, 48000, CONVSR_DIV0_5},
  {44100, 44100, CONVSR_DIV1},
  {7350, 44100, CONVSR_DIV6},
  {11025, 44100, CONVSR_DIV4},
  {14700, 44100, CONVSR_DIV3},
  {22050, 44100, CONVSR_DIV2},
  {29400, 44100, CONVSR_DIV1_5},
  {88200, 44100, CONVSR_DIV0_5},
};

#define ADAU1961_NRATES (sizeof(g_adau1961_rates)/sizeof(struct adau1961_rates_s))

int adau1961_set_sample_rate(FAR struct adau1961_dev_s *priv, uint32_t sr)
{
  int i;

  audinfo("sample rate %u\n", sr);

  /* find the sample rate */
  for (i = 0; i < ADAU1961_NRATES; i++)
    {
      const struct adau1961_rates_s *rate = &g_adau1961_rates[i];
      if (rate->sr == sr)
        {
          /*setup the pll with the base sample rate */
          int rc = adau1961_set_pll(priv, rate->fs);
          if (rc < 0)
            {
              return -1;
            }
          /*set the sample rate divider */
          rc = adau1961_wr(priv, ADAU1961_R17_Converter0, rate->convsr);
          if (rc < 0)
            {
              return -1;
            }
          /*store the sample rate */
          priv->sample_rate = sr;
          return 0;
        }
    }
  auderr("sample rate %u not found\n", sr);
  return -1;
}

/****************************************************************************
 * Name: adau1961_set_lineout_volume
 *
 * Description:
 *   Set the line out volume.
 */

static int adau1961_set_lineout_volume(FAR struct adau1961_dev_s *priv,
                                       uint16_t volume, bool mute)
{
  uint8_t lout, rout;
  int rc;

  audinfo("volume %d mute %d\n", volume, mute);

  if (volume > 1000)
    {
      return -EDOM;
    }

  /* read the register values */
  rc = adau1961_rd(priv, ADAU1961_R31_Line_Output_Left_Vol, &lout);
  if (rc < 0)
    {
      return -EIO;
    }
  rc = adau1961_rd(priv, ADAU1961_R32_Line_Output_Right_Vol, &rout);
  if (rc < 0)
    {
      return -EIO;
    }

  if ((volume == 0) || mute)
    {
      lout = 0;
      rout = 0;
    }
  else
    {
      uint32_t lvol, rvol;

      /* map volume to 0..999 */
      volume -= 1;

      /* left channel volume */
      if (priv->balance <= 500)
        {
          lvol = volume;
        }
      else if (priv->balance == 1000)
        {
          lvol = 0;
        }
      else
        {
          lvol = ((((1000 - priv->balance) << 8) / 500) * volume) >> 8;
        }

      /* right channel volume */
      if (priv->balance >= 500)
        {
          rvol = volume;
        }
      else if (priv->balance == 0)
        {
          rvol = 0;
        }
      else
        {
          rvol = (((priv->balance << 8) / 500) * volume) >> 8;
        }

      /* scale the volumes from 0..999 to 0..63 */
      lvol = (((lvol << 8) / 1000) * 64) >> 8;
      rvol = (((rvol << 8) / 1000) * 64) >> 8;
      /* work out the register value */
      lout = ((lvol & 0x3f) << 2) | (1 << 1 /*unmute */ );
      rout = ((rvol & 0x3f) << 2) | (1 << 1 /*unmute */ );
    }

  /* set the register values */
  rc = adau1961_wr(priv, ADAU1961_R31_Line_Output_Left_Vol, lout);
  if (rc < 0)
    {
      return -EIO;
    }
  rc = adau1961_wr(priv, ADAU1961_R32_Line_Output_Right_Vol, rout);
  if (rc < 0)
    {
      return -EIO;
    }

  /* store the volume and mute settings */
  priv->volume = volume;
  priv->mute = mute;
  return OK;
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

static const struct adau1961_regval_s g_adau1961_regval[] = {
  {ADAU1961_R2_Mic_Jack_Detect, 0},
  {ADAU1961_R3_Rec_Power_Mgmt, 0},

  {ADAU1961_R4_Rec_Mixer_Left0, 0x01},  /*mixer1 enabled */
  {ADAU1961_R5_Rec_Mixer_Left1, 0x08},
  {ADAU1961_R6_Rec_Mixer_Right0, 0x01}, /*mixer2 enabled */
  {ADAU1961_R7_Rec_Mixer_Right1, 0x08},
  {ADAU1961_R8_Left_Diff_Input_Vol, 0},
  {ADAU1961_R9_Right_Diff_Input_Vol, 0},
  {ADAU1961_R10_Record_Mic_Bias, 0},
  {ADAU1961_R11_ALC0, 0},
  {ADAU1961_R12_ALC1, 0},
  {ADAU1961_R13_ALC2, 0},
  {ADAU1961_R14_ALC3, 0},
  {ADAU1961_R15_Serial_Port0, 1},       /*master mode */
  {ADAU1961_R16_Serial_Port1, 0},

  /* {ADAU1961_R17_Converter0, 0}, */
  {ADAU1961_R18_Converter1, 0},

  {ADAU1961_R20_Left_Digital_Vol, 0},   /*0 dB */
  {ADAU1961_R21_Right_Digital_Vol, 0},  /*0 dB */

  {ADAU1961_R22_Play_Mixer_Left0, 0x21},        /*mixer3, unmute left */
  {ADAU1961_R23_Play_Mixer_Left1, 0},
  {ADAU1961_R24_Play_Mixer_Right0, 0x41},       /*mixer4, unmute right */
  {ADAU1961_R25_Play_Mixer_Right1, 0},
  {ADAU1961_R26_Play_LR_Mixer_Left, 0x05},      /*mixer5, 6 dB on left */
  {ADAU1961_R27_Play_LR_Mixer_Right, 0x11},     /*mixer6, 6 dB on right */
  {ADAU1961_R28_Play_LR_Mixer_Mono, 0x01},      /*mixer7 enabled */

  {ADAU1961_R29_Play_HP_Left_Vol, 3},   /* left headphone, unmute, enabled, -57dB */
  {ADAU1961_R30_Play_HP_Right_Vol, 3},  /* right headphone, unmute, enabled, -57dB */
  {ADAU1961_R33_Play_Mono_Output, 2},   /*0dB, unmute, line out */

  {ADAU1961_R34_Pop_Click_Suppress, 0},
  {ADAU1961_R39_Serial_Port_Pad, 0xaa},
  {ADAU1961_R40_Ctl_Port_Pad0, 0xaa},
  {ADAU1961_R41_Ctl_Port_Pad1, 0},
  {ADAU1961_R42_Jack_Detect_Pin, 0x08},
  {ADAU1961_R67_Dejitter_Ctl, 0x03},
  {ADAU1961_R19_ADC_Ctl, 3},    /* enable left/right ADC */
  {ADAU1961_R36_DAC_Ctl0, 3},   /*enable left/right DAC */
  {ADAU1961_R37_DAC_Ctl1, 0},   /*0dB */
  {ADAU1961_R38_DAC_Ctl2, 0},   /*0dB */
  {ADAU1961_R35_Play_Power_Mgmt, 3},    /*enable left and right */
};

#define ADAU1961_NINITS (sizeof(g_adau1961_regval)/sizeof(struct adau1961_regval_s))

static int adau1961_init_registers(FAR struct adau1961_dev_s *priv)
{
  int i;
  for (i = 0; i < ADAU1961_NINITS; i++)
    {
      const struct adau1961_regval_s *reg = &g_adau1961_regval[i];
      int rc = adau1961_wr(priv, reg->addr, reg->val);
      if (rc < 0)
        {
          return -1;
        }
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

  priv->bits_per_sample = 16;
  priv->n_channels = 2;

  rc = adau1961_set_sample_rate(priv, 48000);
  if (rc < 0)
    {
      auderr("adau1961_set_sample_rate failed %d\n", rc);
      return -1;
    }

  rc = adau1961_init_registers(priv);
  if (rc < 0)
    {
      auderr("adau1961_init_registers failed %d\n", rc);
      return -1;
    }

  priv->balance = 500;
  rc = adau1961_set_lineout_volume(priv, CONFIG_ADAU1961_INITVOLUME, false);
  if (rc < 0)
    {
      auderr("adau1961_set_lineout_volume failed %d\n", rc);
      return -1;
    }

  return 0;
}

/****************************************************************************
 * Name: adau1961_senddone
 *
 * Description:
 *   This is the I2S callback function that is invoked when the transfer completes.
 */

static void adau1961_senddone(FAR struct i2s_dev_s *i2s,
                              FAR struct ap_buffer_s *apb, FAR void *arg,
                              int result)
{
  FAR struct adau1961_dev_s *priv = (FAR struct adau1961_dev_s *)arg;
  struct audio_msg_s msg;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(i2s && priv && priv->running && apb);
  audinfo("apb %p inflight %d result %d\n", apb, priv->inflight, result);

  /* We do not place any restriction on the context in which this function
   * is called.  It may be called from an interrupt handler.  Therefore, the
   * doneq and in-flight values might be accessed from the interrupt level.
   * Not the best design.  But we will use interrupt controls to protect
   * against that possibility.
   */

  flags = enter_critical_section();

  /* Add the completed buffer to the end of our doneq.  We do not yet
   * decrement the reference count.
   */
  dq_addlast((FAR dq_entry_t *) apb, &priv->doneq);

  /* And decrement the number of buffers in-flight */
  DEBUGASSERT(priv->inflight > 0);
  priv->inflight--;

  /* Save the result of the transfer */
  priv->result = result;
  leave_critical_section(flags);

  /* Now send a message to the worker thread, informing it that there are
   * buffers in the done queue that need to be cleaned up.
   */
  msg.msgId = AUDIO_MSG_COMPLETE;
  ret = nxmq_send(priv->mq, (FAR const char *)&msg, sizeof(msg),
                  CONFIG_ADAU1961_MSG_PRIO);
  if (ret < 0)
    {
      auderr("nxmq_send failed %d\n", ret);
    }
}

/****************************************************************************
 * Name: adau1961_returnbuffers
 *
 * Description:
 *   This function is called after the complete of one or more data
 *   transfers.  This function will empty the done queue and release our
 *   reference to each buffer.
 */

static void adau1961_returnbuffers(FAR struct adau1961_dev_s *priv)
{

#if 0

  FAR struct ap_buffer_s *apb;
  irqstate_t flags;

  /* The doneq and in-flight values might be accessed from the interrupt
   * level in some implementations.  Not the best design.  But we will
   * use interrupt controls to protect against that possibility.
   */

  flags = enter_critical_section();
  while (dq_peek(&priv->doneq) != NULL)
    {
      /* Take the next buffer from the queue of completed transfers */

      apb = (FAR struct ap_buffer_s *)dq_remfirst(&priv->doneq);
      leave_critical_section(flags);

      audinfo("Returning: apb=%p curbyte=%d nbytes=%d flags=%04x\n",
              apb, apb->curbyte, apb->nbytes, apb->flags);

      /* Are we returning the final buffer in the stream? */

      if ((apb->flags & AUDIO_APB_FINAL) != 0)
        {
          /* Both the pending and the done queues should be empty and there
           * should be no buffers in-flight.
           */

          DEBUGASSERT(dq_empty(&priv->doneq) && dq_empty(&priv->pendq) &&
                      priv->inflight == 0);

          /* Set the terminating flag.  This will, eventually, cause the
           * worker thread to exit (if it is not already terminating).
           */

          audinfo("Terminating\n");
          priv->terminating = true;
        }

      /* Release our reference to the audio buffer */

      apb_free(apb);

      /* Send the buffer back up to the previous level. */

#ifdef CONFIG_AUDIO_MULTI_SESSION
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK, NULL);
#else
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
#endif
      flags = enter_critical_section();
    }

  leave_critical_section(flags);

#endif
}

/****************************************************************************
 * Name: adau1961_sendbuffer
 *
 * Description:
 *   Start the transfer an audio buffer to the ADAU1961 via I2S/SAI. This
 *   will not wait for the transfer to complete but will return immediately.
 *   the adau1961_senddone called will be invoked when the transfer
 *   completes, stimulating the worker thread to call this function again.
 */

static int adau1961_sendbuffer(FAR struct adau1961_dev_s *priv)
{
  int ret = OK;

  /* Loop while there are audio buffers to be sent and we have few than
   * CONFIG_ADAU1961_INFLIGHT then "in-flight"
   *
   * The 'inflight' value might be modified from the interrupt level in some
   * implementations.  We will use interrupt controls to protect against
   * that possibility.
   *
   * The 'pendq', on the other hand, is protected via a semaphore.  Let's
   * hold the semaphore while we are busy here and disable the interrupts
   * only while accessing 'inflight'.
   */

  adau1961_takesem(&priv->pendsem);
  while (priv->inflight < CONFIG_ADAU1961_INFLIGHT
         && dq_peek(&priv->pendq) != NULL && !priv->paused)
    {
      FAR struct ap_buffer_s *apb;
      irqstate_t flags;
      uint32_t timeout;
      int shift;

      /* Take next buffer from the queue of pending transfers */

      apb = (FAR struct ap_buffer_s *)dq_remfirst(&priv->pendq);
      audinfo("send apb %p size %d inflight %d\n", apb, apb->nbytes,
              priv->inflight);

      /* Increment the number of buffers in-flight before sending in order
       * to avoid a possible race condition.
       */
      flags = enter_critical_section();
      priv->inflight++;
      leave_critical_section(flags);

      /* Send the entire audio buffer via I2S.  What is a reasonable timeout
       * to use?  This would depend on the bit rate and size of the buffer.
       *
       * Samples in the buffer (samples):
       *   = buffer_size * 8 / bpsamp                           samples
       * Sample rate (samples/second):
       *   = samplerate * nchannels
       * Expected transfer time (seconds):
       *   = (buffer_size * 8) / bpsamp / samplerate / nchannels
       *
       * We will set the timeout about twice that.
       *
       * NOTES:
       * - The multiplier of 8 becomes 16000 for 2x and units of
       *   milliseconds.
       * - 16000 is a approximately 16384 (1 << 14), bpsamp is either
       *   (1 << 3) or (1 << 4), and nchannels is either (1 << 0) or
       *   (1 << 1).  So this can be simplifies to (milliseconds):
       *
       *   = (buffer_size << shift) / samplerate
       */

      shift = (priv->bits_per_sample == 8) ? 14 - 3 : 14 - 4;
      shift -= (priv->n_channels > 1) ? 1 : 0;

      timeout = MSEC2TICK(((uint32_t) (apb->nbytes - apb->curbyte) << shift) /
                          (uint32_t) priv->sample_rate);

      ret = I2S_SEND(priv->i2s, apb, adau1961_senddone, priv, timeout);
      if (ret < 0)
        {
          auderr("I2S_SEND failed %d\n", ret);
          break;
        }
    }
  adau1961_givesem(&priv->pendsem);

  return ret;
}

/****************************************************************************
 * Name: adau1961_workerthread
 *
 *  This is the thread that feeds data to the chip and keeps the audio
 *  stream going.
 */

static void *adau1961_workerthread(pthread_addr_t pvarg)
{
  FAR struct adau1961_dev_s *priv = (FAR struct adau1961_dev_s *)pvarg;
  struct audio_msg_s msg;
  FAR struct ap_buffer_s *apb;
  int msglen;
  unsigned int prio;

  audinfo("entry\n");

  priv->terminating = false;
  priv->running = true;

  adau1961_set_lineout_volume(priv, priv->volume, 0);

  while (priv->running || priv->inflight > 0)
    {
      /* Check if we have been asked to terminate.  We have to check if we
       * still have buffers in-flight.  If we do, then we can't stop until
       * birds come back to roost.
       */

      if (priv->terminating && priv->inflight <= 0)
        {
          /* We are IDLE.  Break out of the loop and exit. */
          break;
        }
      else
        {
          /* Check if we can send more audio buffers to the ADAU1961 */
          adau1961_sendbuffer(priv);
        }

      /* Wait for messages from our message queue */
      msglen = nxmq_receive(priv->mq, (FAR char *)&msg, sizeof(msg), &prio);

      /* Handle the case when we return with no message */
      if (msglen < sizeof(struct audio_msg_s))
        {
          auderr("message too small: %d\n", msglen);
          continue;
        }

      /* Process the message */
      switch (msg.msgId)
        {
        case AUDIO_MSG_DATA_REQUEST:
          /* The ISR has requested more data */
          audinfo("AUDIO_MSG_DATA_REQUEST\n");
          break;

        case AUDIO_MSG_STOP:
          /* Stop the playback */
          audinfo("AUDIO_MSG_STOP\n");
          priv->terminating = true;
          break;

        case AUDIO_MSG_ENQUEUE:
          /*new buffer to send */
          audinfo("AUDIO_MSG_ENQUEUE\n");
          break;

        case AUDIO_MSG_COMPLETE:
          /* We will wake up from the I2S callback with this message */
          audinfo("AUDIO_MSG_COMPLETE\n");
          adau1961_returnbuffers(priv);
          break;

        default:
          auderr("ignoring %d\n", msg.msgId);
          break;
        }
    }

  /* Reset the CS43L22 hardware */

  adau1961_reset(priv);

  /* Return any pending buffers in our pending queue */
  adau1961_takesem(&priv->pendsem);
  while ((apb = (FAR struct ap_buffer_s *)dq_remfirst(&priv->pendq)) != NULL)
    {
      /* Release our reference to the buffer */
      apb_free(apb);
      /* Send the buffer back up to the previous level. */
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
    }
  adau1961_givesem(&priv->pendsem);

  /* Return any pending buffers in our done queue */
  adau1961_returnbuffers(priv);

  /* Close the message queue */
  mq_close(priv->mq);
  mq_unlink(priv->mqname);
  priv->mq = NULL;

  /* Send an AUDIO_MSG_COMPLETE message to the client */
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK);

  audinfo("exit\n");
  return NULL;
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

/****************************************************************************
 * Name: adau1961_start
 *
 * Description:
 *   Start the configured operation (audio streaming, volume enabled, etc.).
 */

static int adau1961_start(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct adau1961_dev_s *priv = (FAR struct adau1961_dev_s *)dev;
  struct sched_param sparam;
  struct mq_attr attr;
  pthread_attr_t tattr;
  FAR void *value;
  int ret;

  audinfo("entry\n");

  /* Create a message queue for the worker thread */
  snprintf(priv->mqname, sizeof(priv->mqname), "/tmp/%X", priv);
  attr.mq_maxmsg = 16;
  attr.mq_msgsize = sizeof(struct audio_msg_s);
  attr.mq_curmsgs = 0;
  attr.mq_flags = 0;

  priv->mq = mq_open(priv->mqname, O_RDWR | O_CREAT, 0644, &attr);
  if (priv->mq == NULL)
    {
      auderr("couldn't allocate message queue\n");
      return -ENOMEM;
    }

  /* Join any old worker thread we had created to prevent a memory leak */
  if (priv->threadid != 0)
    {
      audinfo("joining old thread\n");
      pthread_join(priv->threadid, &value);
    }

  /* Start our thread for sending data to the device */
  pthread_attr_init(&tattr);
  sparam.sched_priority = sched_get_priority_max(SCHED_FIFO) - 3;
  pthread_attr_setschedparam(&tattr, &sparam);
  pthread_attr_setstacksize(&tattr, CONFIG_ADAU1961_WORKER_STACKSIZE);

  audinfo("starting worker thread\n");
  ret =
    pthread_create(&priv->threadid, &tattr, adau1961_workerthread,
                   (pthread_addr_t) priv);
  if (ret != OK)
    {
      auderr("pthread_create failed %d\n", ret);
    }
  else
    {
      pthread_setname_np(priv->threadid, "adau1961");
      audinfo("created worker thread\n");
    }

  return ret;
}

/****************************************************************************
 * Name: adau1961_stop
 *
 * Description:
 *   Stop the configured operation (audio streaming, volume disabled, etc.).
 */

static int adau1961_stop(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct adau1961_dev_s *priv = (FAR struct adau1961_dev_s *)dev;
  struct audio_msg_s term_msg;
  FAR void *value;

  /* Send a message to stop all audio streaming */
  term_msg.msgId = AUDIO_MSG_STOP;
  term_msg.u.data = 0;
  nxmq_send(priv->mq, (FAR const char *)&term_msg, sizeof(term_msg),
            CONFIG_ADAU1961_MSG_PRIO);

  /* Join the worker thread */
  pthread_join(priv->threadid, &value);
  priv->threadid = 0;

  audinfo("ok\n");
  return OK;
}

/****************************************************************************
 * Name: adau1961_configure
 *
 * Description:
 *   Configure the audio device for the specified  mode of operation.
 */

static int adau1961_configure(FAR struct audio_lowerhalf_s *dev,
                              FAR const struct audio_caps_s *caps)
{
  FAR struct adau1961_dev_s *priv = (FAR struct adau1961_dev_s *)dev;
  int ret = OK;

  DEBUGASSERT(priv != NULL && caps != NULL);

  audinfo("ac_type %d\n", caps->ac_type);
  switch (caps->ac_type)
    {
    case AUDIO_TYPE_FEATURE:
      audinfo("AUDIO_TYPE_FEATURE ac_format.hw %d\n", caps->ac_format.hw);
      switch (caps->ac_format.hw)
        {
        case AUDIO_FU_VOLUME:
          {
            uint16_t vol = caps->ac_controls.hw[0];
            ret = adau1961_set_lineout_volume(priv, vol, priv->mute);
            break;
          }

        default:
          audinfo("ignored\n");
          break;
        }
      break;

    default:
      audinfo("ignored\n");
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: adau1961_reserve
 *
 * Description:
 *   Reserves a session (the only one we have).
 */

static int adau1961_reserve(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct adau1961_dev_s *priv = (FAR struct adau1961_dev_s *)dev;
  int ret = OK;
  adau1961_takesem(&priv->pendsem);
  if (priv->reserved)
    {
      ret = -EBUSY;
    }
  else
    {
      /* Initialize the session context */
      priv->inflight = 0;
      priv->running = false;
      priv->paused = false;
      priv->terminating = false;
      priv->reserved = true;
    }
  adau1961_givesem(&priv->pendsem);
  audinfo("%s\n", (ret == OK) ? "ok" : "error");
  return ret;
}

/****************************************************************************
 * Name: adau1961_release
 *
 * Description:
 *   Releases the session (the only one we have).
 */

static int adau1961_release(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct adau1961_dev_s *priv = (FAR struct adau1961_dev_s *)dev;
  /* Join any old worker thread we had created to prevent a memory leak */
  if (priv->threadid != 0)
    {
      void *value;
      pthread_join(priv->threadid, &value);
      priv->threadid = 0;
    }
  adau1961_takesem(&priv->pendsem);
  /* Really we should free any queued buffers here */
  priv->reserved = false;
  adau1961_givesem(&priv->pendsem);
  audinfo("ok\n");
  return OK;
}

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
