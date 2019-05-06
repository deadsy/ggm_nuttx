/****************************************************************************
 * drivers/audio/adau1391.c
 * Audio device driver for Analog Devices ADAU1391 Stereo CODEC.
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
#include <nuttx/audio/audio.h>
#include <nuttx/audio/adau1391.h>

#include "adau1391.h"

/****************************************************************************/

#define ADAU1391_REG_BASE 0x4000

#define ADAU1391_REG_Clock_Ctl              0x00
#define ADAU1391_REG_PLL_Ctl                0x02
#define ADAU1391_REG_Mic_Jack_Detect        0x08
#define ADAU1391_REG_Rec_Power_Mgmt         0x09
#define ADAU1391_REG_Rec_Mixer_Left0        0x0A
#define ADAU1391_REG_Rec_Mixer_Left1        0x0B
#define ADAU1391_REG_Rec_Mixer_Right0       0x0C
#define ADAU1391_REG_Rec_Mixer_Right1       0x0D
#define ADAU1391_REG_Left_Diff_Input_Vol    0x0E
#define ADAU1391_REG_Right_Diff_Input_Vol   0x0F
#define ADAU1391_REG_Record_Mic_Bias        0x10
#define ADAU1391_REG_ALC0                   0x11
#define ADAU1391_REG_ALC1                   0x12
#define ADAU1391_REG_ALC2                   0x13
#define ADAU1391_REG_ALC3                   0x14
#define ADAU1391_REG_Serial_Port0           0x15
#define ADAU1391_REG_Serial_Port1           0x16
#define ADAU1391_REG_Converter0             0x17
#define ADAU1391_REG_Converter1             0x18
#define ADAU1391_REG_ADC_Ctl                0x19
#define ADAU1391_REG_Left_Digital_Vol       0x1A
#define ADAU1391_REG_Right_Digital_Vol      0x1B
#define ADAU1391_REG_Play_Mixer_Left0       0x1C
#define ADAU1391_REG_Play_Mixer_Left1       0x1D
#define ADAU1391_REG_Play_Mixer_Right0      0x1E
#define ADAU1391_REG_Play_Mixer_Right1      0x1F
#define ADAU1391_REG_Play_LR_Mixer_Left     0x20
#define ADAU1391_REG_Play_LR_Mixer_Right    0x21
#define ADAU1391_REG_Play_LR_Mixer_Mono     0x22
#define ADAU1391_REG_Play_HP_Left_Vol       0x23
#define ADAU1391_REG_Play_HP_Right_Vol      0x24
#define ADAU1391_REG_Line_Output_Left_Vol   0x25
#define ADAU1391_REG_Line_Output_Right_Vol  0x26
#define ADAU1391_REG_Play_Mono_Output       0x27
#define ADAU1391_REG_Pop_Click_Suppress     0x28
#define ADAU1391_REG_Play_Power_Mgmt        0x29
#define ADAU1391_REG_DAC_Ctl0               0x2A
#define ADAU1391_REG_DAC_Ctl1               0x2B
#define ADAU1391_REG_DAC_Ctl2               0x2C
#define ADAU1391_REG_Serial_Port_Pad        0x2D
#define ADAU1391_REG_Ctl_Port_Pad0          0x2F
#define ADAU1391_REG_Ctl_Port_Pad1          0x30
#define ADAU1391_REG_Jack_Detect_Pin        0x31
#define ADAU1391_REG_Dejitter_Ctl           0x36

/****************************************************************************/

struct adau1391_dev_s
{
  struct audio_lowerhalf_s dev; // ADAU1391 audio lower half (this device)

  // Our specific driver data goes here
  FAR const struct adau1391_lower_s *lower;     // Low-level board specific functions
  FAR struct i2c_master_s *i2c; // I2C driver to use
  FAR struct i2s_dev_s *i2s;    // I2S driver to use
  struct dq_queue_s pendq;      // Queue of pending buffers to be sent
  struct dq_queue_s doneq;      // Queue of sent buffers to be returned
  sem_t pendsem;                // Protect pendq

};

/****************************************************************************
 * i2c read/write routines
 */

/* read n bytes at a device offset */
static int adau1391_rdbuf(FAR struct adau1391_dev_s *priv, uint8_t ofs,
                          uint8_t * buf, size_t n)
{
  return OK;
}

/* write n bytes at a device offset */
static int adau1391_wrbuf(FAR struct adau1391_dev_s *priv, uint8_t ofs,
                          const uint8_t * buf, size_t n)
{
  return OK;
}

/* read 1 byte at a device offset */
static int adau1391_rd8(FAR struct adau1391_dev_s *priv, uint8_t ofs,
                        uint8_t * val)
{
  return adau1391_rdbuf(priv, ofs, val, 1);
}

/* read 6 bytes at a device offset */
static int adau1391_rd48(FAR struct adau1391_dev_s *priv, uint8_t ofs,
                         uint8_t * val)
{
  return adau1391_rdbuf(priv, ofs, val, 6);
}

/* write 1 byte at a device offset */
static int adau1391_wr8(FAR struct adau1391_dev_s *priv, uint8_t ofs,
                        uint8_t val)
{
  return adau1391_wrbuf(priv, ofs, &val, 1);
}

/* write 6 bytes at a device offset */
static int adau1391_wr48(FAR struct adau1391_dev_s *priv, uint8_t ofs,
                         const uint8_t * val)
{
  return adau1391_wrbuf(priv, ofs, val, 6);
}

/****************************************************************************/

struct adau1391_regdump_s
{
  const char *name;
  uint8_t addr;
};

static const struct adau1391_regdump_s g_adau1391_debug[] = {
  {"Clock_Ctl", ADAU1391_REG_Clock_Ctl},
  {"PLL_Ctl", ADAU1391_REG_PLL_Ctl},
  {"Mic_Jack_Detect", ADAU1391_REG_Mic_Jack_Detect},
  {"Rec_Power_Mgmt", ADAU1391_REG_Rec_Power_Mgmt},
  {"Rec_Mixer_Left0", ADAU1391_REG_Rec_Mixer_Left0},
  {"Rec_Mixer_Left1", ADAU1391_REG_Rec_Mixer_Left1},
  {"Rec_Mixer_Right0", ADAU1391_REG_Rec_Mixer_Right0},
  {"Rec_Mixer_Right1", ADAU1391_REG_Rec_Mixer_Right1},
  {"Left_Diff_Input_Vol", ADAU1391_REG_Left_Diff_Input_Vol},
  {"Right_Diff_Input_Vol", ADAU1391_REG_Right_Diff_Input_Vol},
  {"Record_Mic_Bias", ADAU1391_REG_Record_Mic_Bias},
  {"ALC0", ADAU1391_REG_ALC0},
  {"ALC1", ADAU1391_REG_ALC1},
  {"ALC2", ADAU1391_REG_ALC2},
  {"ALC3", ADAU1391_REG_ALC3},
  {"Serial_Port0", ADAU1391_REG_Serial_Port0},
  {"Serial_Port1", ADAU1391_REG_Serial_Port1},
  {"Converter0", ADAU1391_REG_Converter0},
  {"Converter1", ADAU1391_REG_Converter1},
  {"ADC_Ctl", ADAU1391_REG_ADC_Ctl},
  {"Left_Digital_Vol", ADAU1391_REG_Left_Digital_Vol},
  {"Right_Digital_Vol", ADAU1391_REG_Right_Digital_Vol},
  {"Play_Mixer_Left0", ADAU1391_REG_Play_Mixer_Left0},
  {"Play_Mixer_Left1", ADAU1391_REG_Play_Mixer_Left1},
  {"Play_Mixer_Right0", ADAU1391_REG_Play_Mixer_Right0},
  {"Play_Mixer_Right1", ADAU1391_REG_Play_Mixer_Right1},
  {"Play_LR_Mixer_Left", ADAU1391_REG_Play_LR_Mixer_Left},
  {"Play_LR_Mixer_Right", ADAU1391_REG_Play_LR_Mixer_Right},
  {"Play_LR_Mixer_Mono", ADAU1391_REG_Play_LR_Mixer_Mono},
  {"Play_HP_Left_Vol", ADAU1391_REG_Play_HP_Left_Vol},
  {"Play_HP_Right_Vol", ADAU1391_REG_Play_HP_Right_Vol},
  {"Line_Output_Left_Vol", ADAU1391_REG_Line_Output_Left_Vol},
  {"Line_Output_Right_Vol", ADAU1391_REG_Line_Output_Right_Vol},
  {"Play_Mono_Output", ADAU1391_REG_Play_Mono_Output},
  {"Pop_Click_Suppress", ADAU1391_REG_Pop_Click_Suppress},
  {"Play_Power_Mgmt", ADAU1391_REG_Play_Power_Mgmt},
  {"DAC_Ctl0", ADAU1391_REG_DAC_Ctl0},
  {"DAC_Ctl1", ADAU1391_REG_DAC_Ctl1},
  {"DAC_Ctl2", ADAU1391_REG_DAC_Ctl2},
  {"Serial_Port_Pad", ADAU1391_REG_Serial_Port_Pad},
  {"Ctl_Port_Pad0", ADAU1391_REG_Ctl_Port_Pad0},
  {"Ctl_Port_Pad1", ADAU1391_REG_Ctl_Port_Pad1},
  {"Jack_Detect_Pin", ADAU1391_REG_Jack_Detect_Pin},
  {"Dejitter_Ctl", ADAU1391_REG_Dejitter_Ctl},
};

#define ADAU1391_NREGISTERS (sizeof(g_adau1391_debug)/sizeof(struct adau1391_regdump_s))

void adau1391_dump_registers(FAR struct audio_lowerhalf_s *dev,
                             FAR const char *msg)
{
  syslog(LOG_INFO, "ADAU1391 Registers: %s\n", msg);
  for (int i = 0; i < ADAU1391_NREGISTERS; i++)
    {
      const char *name = g_adau1391_debug[i].name;
      uint8_t addr = g_adau1391_debug[i].addr;
      uint8_t val = adau1391_rd8((struct adau1391_dev_s *)dev, addr);
      syslog(LOG_INFO, "%16s[%04x]: %02x\n", name, addr, val);
    }
}

/****************************************************************************/

// Reset the ADAU1391 chip
static void adau1391_hw_reset(FAR const struct adau1391_lower_s *lower)
{
  DEBUGASSERT(lower && lower->reset);
  lower->reset(lower);
}

// Reset and re-initialize the ADAU1391
static void adau1391_reset(FAR struct adau1391_dev_s *priv)
{
}

/****************************************************************************/

// Get the lower-half device capabilities
static int adau1391_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                            FAR struct audio_caps_s *pCaps)
{
  audinfo("\n");
  return 0;
}

// Shutdown the driver (called after close)
static int adau1391_shutdown(FAR struct audio_lowerhalf_s *dev)
{
  audinfo("\n");
  return 0;
}

#if 0
// Allocate an audio pipeline buffer
static int adau1391_allocbuffer(FAR struct audio_lowerhalf_s *dev,
                                FAR struct audio_buf_desc_s *apb)
{
  audinfo("\n");
  return 0;
}
#endif

#if 0
// Free an audio pipeline buffer
static int adau1391_freebuffer(FAR struct audio_lowerhalf_s *dev,
                               FAR struct audio_buf_desc_s *apb)
{
  audinfo("\n");
  return 0;
}
#endif

// Enqueue a buffer for processing
static int adau1391_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                  FAR struct ap_buffer_s *apb)
{
  audinfo("\n");
  return 0;
}

// Cancel a previously enqueued buffer
static int adau1391_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                                 FAR struct ap_buffer_s *apb)
{
  audinfo("\n");
  return 0;
}

// Driver specific ioctl commands
static int adau1391_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                          unsigned long arg)
{
  audinfo("\n");
  return 0;
}

#if 0
// Driver specific read commands
static int adau1391_read(FAR struct audio_lowerhalf_s *dev, FAR char *buffer,
                         size_t buflen)
{
  audinfo("\n");
  return 0;
}
#endif

#if 0
// Driver specific write commands
static int adau1391_write(FAR struct audio_lowerhalf_s *dev,
                          FAR const char *buffer, size_t buflen)
{
  audinfo("\n");
  return 0;
}
#endif

#ifdef CONFIG_AUDIO_MULTI_SESSION

// Release a session
static int adau1391_release(FAR struct audio_lowerhalf_s *dev,
                            FAR void *session)
{
  audinfo("\n");
  return 0;
}

// Reserve a session
static int adau1391_reserve(FAR struct audio_lowerhalf_s *dev,
                            FAR void **psession)
{
  audinfo("\n");
  return 0;
}

// Resumes audio streaming after a pause
static int adau1391_resume(FAR struct audio_lowerhalf_s *dev, FAR void *session)
{
  audinfo("\n");
  return 0;
}

// Pause the audio stream
static int adau1391_pause(FAR struct audio_lowerhalf_s *dev, FAR void *session)
{
  audinfo("\n");
  return 0;
}

// Start audio streaming
static int adau1391_start(FAR struct audio_lowerhalf_s *dev, FAR void *session)
{
  audinfo("\n");
  return 0;
}

// Stop audio streaming
static int adau1391_stop(FAR struct audio_lowerhalf_s *dev, FAR void *session)
{
  audinfo("\n");
  return 0;
}

// Configure the driver
static int adau1391_configure(FAR struct audio_lowerhalf_s *dev, void *session,
                              FAR const struct audio_caps_s *pCaps)
{
  audinfo("\n");
  return 0;
}

#else

// Release a session
static int adau1391_release(FAR struct audio_lowerhalf_s *dev)
{
  audinfo("\n");
  return 0;
}

// Reserve a session
static int adau1391_reserve(FAR struct audio_lowerhalf_s *dev)
{
  audinfo("\n");
  return 0;
}

// Resumes audio streaming after a pause
static int adau1391_resume(FAR struct audio_lowerhalf_s *dev)
{
  audinfo("\n");
  return 0;
}

// Pause the audio stream
static int adau1391_pause(FAR struct audio_lowerhalf_s *dev)
{
  audinfo("\n");
  return 0;
}

// Start audio streaming
static int adau1391_start(FAR struct audio_lowerhalf_s *dev)
{
  audinfo("\n");
  return 0;
}

// Stop audio streaming
static int adau1391_stop(FAR struct audio_lowerhalf_s *dev)
{
  audinfo("\n");
  return 0;
}

// Configure the driver
static int adau1391_configure(FAR struct audio_lowerhalf_s *dev,
                              FAR const struct audio_caps_s *pCaps)
{
  audinfo("\n");
  return 0;
}

#endif // not CONFIG_AUDIO_MULTI_SESSION

/****************************************************************************/

static const struct audio_ops_s g_audioops = {
  .getcaps = adau1391_getcaps,
  .configure = adau1391_configure,
  .shutdown = adau1391_shutdown,
  .start = adau1391_start,
  .stop = adau1391_stop,
  .pause = adau1391_pause,
  .resume = adau1391_resume,
  .allocbuffer = NULL /* adau1391_allocbuffer */ ,
  .freebuffer = NULL /* adau1391_freebuffer */ ,
  .enqueuebuffer = adau1391_enqueuebuffer,
  .cancelbuffer = adau1391_cancelbuffer,
  .ioctl = adau1391_ioctl,
  .read = NULL /* adau1391_read */ ,
  .write = NULL /* adau1391_write */ ,
  .reserve = adau1391_reserve,
  .release = adau1391_release,
};

/****************************************************************************/

FAR struct audio_lowerhalf_s *adau1391_initialize(FAR struct i2c_master_s *i2c,
                                                  FAR struct i2s_dev_s *i2s,
                                                  FAR const struct
                                                  adau1391_lower_s *lower)
{
  // Sanity check
  DEBUGASSERT(i2c && i2s && lower);

  // Allocate a ADAU1391 device structure
  struct adau1391_dev_s *priv =
    (struct adau1391_dev_s *)kmm_zalloc(sizeof(struct adau1391_dev_s));
  if (priv)
    {
      // Initialize the ADAU1391 device structure.

      priv->dev.ops = &g_audioops;
      priv->lower = lower;
      priv->i2c = i2c;
      priv->i2s = i2s;

      nxsem_init(&priv->pendsem, 0, 1);
      dq_init(&priv->pendq);
      dq_init(&priv->doneq);

      // Initialize I2C
      audinfo("address=%02x frequency=%d\n", lower->address, lower->frequency);

      // Reset the ADAU1391
      adau1391_hw_reset(priv->lower);
      adau1391_dump_registers(&priv->dev, "After reset");

      // Verify the ADAU1391 is present and available on this I2C
      uint8_t val = adau1391_readreg(priv, ADAU1391_REG_Serial_Port_Pad);
      if (val != 0xaa)
        {
          auderr("adau1391 not found\n");
          goto errout_with_dev;
        }
      // Reset and reconfigure the ADAU1391 hardware
      adau1391_reset(priv);
      return &priv->dev;
    }

  return NULL;

errout_with_dev:
  nxsem_destroy(&priv->pendsem);
  kmm_free(priv);
  return NULL;
}

/****************************************************************************/
