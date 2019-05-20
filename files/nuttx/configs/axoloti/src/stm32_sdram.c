/************************************************************************************
 * configs/axoloti/src/stm32_fmc.c
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

  /* Configure SDRAM GPIOs */
  for (i = 0; i < NUM_SDRAM_GPIOS; i++)
    {
      stm32_configgpio(g_sdram_config[i]);
    }

  /* Enable AHB clocking to the FMC */
  val = getreg32(STM32_RCC_AHB3ENR);
  val |= RCC_AHB3ENR_FMCEN;
  putreg32(val, STM32_RCC_AHB3ENR);

  /* Go throught the SDRAM initialization steps per the reference manual.
   * hclk period = 1/84 MHz = 11.9 ns (??)
   */

  /* Step 1:
   * Program the memory device features into the FMC_SDCRx register.The SDRAM clock
   * frequency, RBURST and RPIPE must be programmed in the FMC_SDCR1 register.
   */
  val = getreg32(STM32_FMC_SDCR1);
  val &= (0x1ffff << 15);       /* reserved */
  val |= (1 << 13);             /* rpipe = 1 hclk */
  val |= (1 << 12);             /* rburst enabled */
  val |= (2 << 10);             /* sdclk = 2 hclk */
  val |= (0 << 9);              /* wp disabled */
  val |= (2 << 7);              /* cas = 2 cycles */
  val |= (1 << 6);              /* nb = 4 internal banks */
  val |= (1 << 4);              /* mwid = 16 bits */
  val |= (1 << 2);              /* nr = 12 bits */
  val |= (0 << 0);              /* nc = 8 bits */
  putreg32(val, STM32_FMC_SDCR1);

  /* Step 2:
   * Program the memory device timing into the FMC_SDTRx register. The TRP and TRC
   * timings must be programmed in the FMC_SDTR1 register.
   */

  /* TODO - review, I'm not convinced the hclk is 11.9 ns */
  val = getreg32(STM32_FMC_SDTR1);
  val &= (15 << 28);            /* reserved */
  val |= (1 << 24);             /* trcd 15ns => 2x11.90ns */
  val |= (1 << 20);             /* trp 15ns => 2x11.90ns */
  val |= (1 << 16);             /* twr 2 clock cycles */
  val |= (6 << 12);             /* trc min=63 (6x11.90ns) */
  val |= (3 << 8);              /* tras min=42ns (4x11.90ns) max=120k (ns) */
  val |= (6 << 4);              /* txsr min=70ns (6x11.90ns) */
  val |= (1 << 0);              /* tmrd 2 clock cycles */
  putreg32(val, STM32_FMC_SDTR1);

  /* Step 3:
   * Set MODE bits to ‘001’ and configure the Target Bank bits (CTB1 and/or CTB2) in the
   * FMC_SDCMR register to start delivering the clock to the memory (SDCKE is driven
   * high).
   */

  /* Step 4:
   * Wait during the prescribed delay period. Typical delay is around 100 μs (refer to the
   * SDRAM datasheet for the required delay after power-up).
   */

  /* Step 5:
   * Set MODE bits to ‘010’ and configure the Target Bank bits (CTB1 and/or CTB2) in the
   * FMC_SDCMR register to issue a “Precharge All” command.
   */

  /* Step 6:
   * Set MODE bits to ‘011’, and configure the Target Bank bits (CTB1 and/or CTB2) as well
   * as the number of consecutive Auto-refresh commands (NRFS) in the FMC_SDCMR
   * register. Refer to the SDRAM datasheet for the number of Auto-refresh commands that
   * should be issued. Typical number is 8.
   */

  /* Step 7:
   * Configure the MRD field according to your SDRAM device, set the MODE bits to '100',
   * and configure the Target Bank bits (CTB1 and/or CTB2) in the FMC_SDCMR register
   * to issue a "Load Mode Register" command in order to program the SDRAM. In
   * particular:
   * a) The CAS latency must be selected following configured value in FMC_SDCR1/2
   *    registers
   * b) The Burst Length (BL) of 1 must be selected by configuring the M[2:0] bits to 000
   *    in the mode register (refer to the SDRAM datasheet). If the Mode Register is not
   *    the same for both SDRAM banks, this step has to be repeated twice, once for
   *    each bank, and the Target Bank bits set accordingly.
   */

  /* Step 8:
   * Program the refresh rate in the FMC_SDRTR register
   * The refresh rate corresponds to the delay between refresh cycles. Its value must be
   * adapted to SDRAM devices.
   */

  /* Step 9:
   * For mobile SDRAM devices, to program the extended mode register it should be done
   * once the SDRAM device is initialized: First, a dummy read access should be performed
   * while BA1=1 and BA=0 (refer to SDRAM address mapping section for BA[1:0] address
   * mapping) in order to select the extended mode register instead of Load mode register
   * and then program the needed value.
   */

  /* We don't have a mobile SDRAM device ... */

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
