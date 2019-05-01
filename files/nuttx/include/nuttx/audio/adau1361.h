/****************************************************************************
 * include/nuttx/audio/adau1361.h
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

#ifndef __INCLUDE_NUTTX_AUDIO_ADAU1361_H
#define __INCLUDE_NUTTX_AUDIO_ADAU1361_H

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct adau1361_lower_s; /* Forward reference.  Defined below */

typedef CODE int (*adau1361_handler_t) (FAR const struct adau1361_lower_s *lower,
                                        FAR void *arg);

struct adau1361_lower_s
{
  /* I2C characterization */

  uint32_t frequency;   /* Initial I2C frequency */
  uint8_t address;      /* 7-bit I2C address (only bits 0-6 used) */

  /* IRQ/GPIO access callbacks.  These operations all hidden behind
    * callbacks to isolate the ADAU1361 driver from differences in GPIO
    * interrupt handling by varying boards and MCUs.  If possible,
    * interrupts should be configured on both rising and falling edges
    * so that contact and loss-of-contact events can be detected.
    *
    * attach  - Attach or detach the ADAU1361 interrupt handler to the GPIO
    *           interrupt
    * enable  - Enable or disable the GPIO interrupt.  Returns the
    *           previous interrupt state.
    * reset   - HW reset of the ADAU1361 chip
    */

  CODE int (*attach) (FAR const struct adau1361_lower_s * lower,
                      adau1361_handler_t isr, FAR void *arg);
  CODE bool (*enable) (FAR const struct adau1361_lower_s * lower, bool enable);
  CODE void (*reset) (FAR const struct adau1361_lower_s * lower);
};

#endif /* __INCLUDE_NUTTX_AUDIO_ADAU1361_H */
