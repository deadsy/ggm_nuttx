/************************************************************************************
 * configs/axoloti/src/stm32_adau1361.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Taras Drozdovskiy <t.drozdovskiy@gmail.com>
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
  int i;

  /* Reset the codec */

  stm32_gpiowrite(GPIO_ADAU1361_RESET, false);
  for (i = 0; i < 0x4fff; i++)
    {
      __asm__ volatile ("nop");
    }

  stm32_gpiowrite(GPIO_ADAU1361_RESET, true);
}

/************************************************************************************/
