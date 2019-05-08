/****************************************************************************
 * include/nuttx/audio/adau1961.h
 * Audio device driver for Analog Devices ADAU1961 Stereo CODEC.
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

#ifndef __INCLUDE_NUTTX_AUDIO_ADAU1961_H
#define __INCLUDE_NUTTX_AUDIO_ADAU1961_H

/****************************************************************************
 * Public Types
 ****************************************************************************/
/* This is the type of the ADAU1961 interrupt handler.  The lower level code
 * will intercept the interrupt and provide the upper level with the private
 * data that was provided when the interrupt was attached.
 */

struct adau1961_lower_s;        /* Forward reference. Defined below */

typedef CODE int (*adau1961_handler_t) (FAR const struct adau1961_lower_s *
                                        lower, FAR void *arg);

/* A reference to a structure of this type must be passed to the ADAU1961
 * driver.  This structure provides information about the configuration
 * of the ADAU1961 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active.
 */

struct adau1961_lower_s
{
  /* I2C characterization */
  uint32_t frequency;           /* Initial I2C frequency */
  uint8_t address;              /* 7-bit I2C address (only bits 0-6 used) */

  /* Clocking is provided via MCLK.  The ADAU1961 driver will need to know
   * the frequency of MCLK in order to generate the correct bitrates.
   */
  uint32_t mclk;                /* ADAU1961 Master clock frequency */

  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the ADAU1961 driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.  If possible,
   * interrupts should be configured on both rising and falling edges
   * so that contact and loss-of-contact events can be detected.
   *
   * attach  - Attach or detach the ADAU1961 interrupt handler to the GPIO
   *           interrupt
   * enable  - Enable or disable the GPIO interrupt.  Returns the
   *           previous interrupt state.
   * reset   - HW reset of the ADAU1961 chip
   */

  CODE int (*attach) (FAR const struct adau1961_lower_s * lower,
                      adau1961_handler_t isr, FAR void *arg);
  CODE bool(*enable) (FAR const struct adau1961_lower_s * lower, bool enable);
  CODE void (*reset) (FAR const struct adau1961_lower_s * lower);
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: adau1961_initialize
 *
 * Description:
 *   Initialize the ADAU1961 device.
 *
 * Input Parameters:
 *   i2c     - An I2C driver instance
 *   i2s     - An I2S driver instance
 *   lower   - Persistent board configuration data
 *
 * Returned Value:
 *   A new lower half audio interface for the ADAU1961 device is returned on
 *   success; NULL is returned on failure.
 *
 ****************************************************************************/

struct i2c_master_s;            /* Forward reference. Defined in include/nuttx/i2c/i2c_master.h */
struct i2s_dev_s;               /* Forward reference. Defined in include/nuttx/audio/i2s.h */
struct audio_lowerhalf_s;       /* Forward reference. Defined in nuttx/audio/audio.h */

FAR struct audio_lowerhalf_s *adau1961_initialize(FAR struct i2c_master_s *i2c,
                                                  FAR struct i2s_dev_s *i2s,
                                                  FAR const struct
                                                  adau1961_lower_s *lower);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __INCLUDE_NUTTX_AUDIO_ADAU1961_H */
