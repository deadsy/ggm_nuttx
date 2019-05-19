/************************************************************************************
 * configs/axoloti/src/stm32_fmc.c
 *
 *   Copyright (C) 2012-2019 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Jason T. Harris <sirmanlypowers@gmail.com>
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"

#include "stm32.h"
#include "axoloti.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#ifndef CONFIG_STM32_FMC
#warning "FMC is not enabled"
#endif

/****************************************************************************
 * Axoloti SDRAM GPIO configuration
 */

static const uint32_t g_sdram_config[] = {
  /* data lines */
  GPIO_FMC_D0, GPIO_FMC_D1, GPIO_FMC_D2, GPIO_FMC_D3,
  GPIO_FMC_D4, GPIO_FMC_D5, GPIO_FMC_D6, GPIO_FMC_D7,
  GPIO_FMC_D8, GPIO_FMC_D9, GPIO_FMC_D10, GPIO_FMC_D11,
  GPIO_FMC_D12, GPIO_FMC_D13, GPIO_FMC_D14, GPIO_FMC_D15,
  /* address lines */
  GPIO_FMC_A0, GPIO_FMC_A1, GPIO_FMC_A2, GPIO_FMC_A3,
  GPIO_FMC_A4, GPIO_FMC_A5, GPIO_FMC_A6, GPIO_FMC_A7,
  GPIO_FMC_A8, GPIO_FMC_A9, GPIO_FMC_A10, GPIO_FMC_A11,
  GPIO_FMC_A12,
  /* control lines */
  GPIO_FMC_BA0,                 /* ba0 */
  GPIO_FMC_BA1,                 /* ba1 */
  GPIO_FMC_NBL0,                /* ldqm */
  GPIO_FMC_NBL1,                /* udqm */
  GPIO_FMC_SDCLK,               /* clk */
  GPIO_FMC_SDCKE0_1,            /* cke */
  GPIO_FMC_SDNWE_2,             /* we */
  GPIO_FMC_SDNCAS,              /* cas */
  GPIO_FMC_SDNRAS,              /* ras */
  GPIO_FMC_SDNE0_1,             /* cs0 */
  GPIO_FMC_SDNE1_2,             /* cs1 */
  //GPIO_FMC_SDNE1_1,             /* cs1 ?? */
};

#define NUM_SDRAM_GPIOS (sizeof(g_sdram_config) / sizeof(uint32_t))

/****************************************************************************
 * Name: stm32_sdram_initialize
 *
 * Description:
 *   Called from stm32_bringup to initialize external SDRAM access.
 */

int stm32_sdram_initialize(void)
{
  uint32_t val;
  int i;

  /* Configure GPIOs */
  for (i = 0; i < NUM_SDRAM_GPIOS; i++)
    {
      stm32_configgpio(g_sdram_config[i]);
    }

  /* Enable AHB clocking to the FMC */
  val = getreg32(STM32_RCC_AHB3ENR);
  val |= RCC_AHB3ENR_FMCEN;
  putreg32(val, STM32_RCC_AHB3ENR);

/*
SDCR1 : a0000140[31:0] = 0x00003954   SDRAM Control Register 1
SDCR2 : a0000144[31:0] = 0x000002d0   SDRAM Control Register 2
SDTR1 : a0000148[31:0] = 0x01116361   SDRAM Timing register 1
SDTR2 : a000014c[31:0] = 0x0fffffff   SDRAM Timing register 2
SDCMR : a0000150[31:0] = 0x00044200   SDRAM Command Mode register
SDRTR : a0000154[31:0] = 0x00000556   SDRAM Refresh Timer register
SDSR  : a0000158[31:0] = 0            SDRAM Status register
*/

  return OK;
}
