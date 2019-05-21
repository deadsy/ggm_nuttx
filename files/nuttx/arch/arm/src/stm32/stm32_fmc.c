/************************************************************************************
 * arch/arm/src/stm32/stm32_fmc.c
 *
 *   Copyright (C) 20019 Gregory Nutt. All rights reserved.
 *   Author: Jason T. Harris <dirmanlypowers@gmail.com>
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

#include <nuttx/signal.h>

#include "stm32.h"

#if defined(CONFIG_STM32_FMC)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Types
 ************************************************************************************/

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Private Data
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/****************************************************************************
 * Name: stm32_fmc_sdram_wait
 *
 * Description:
 *   Wait for the SDRAM controller to be ready.
 *
 ****************************************************************************/

void stm32_fmc_sdram_wait(void)
{
  int timeout = 5;
  while (timeout > 0)
    {
      if ((getreg32(STM32_FMC_SDSR) & (1 << 5 /*busy */ )) == 0)
        {
          break;
        }
      timeout--;
      nxsig_usleep(1000);
    }
  DEBUGASSERT(timeout > 0);
}

/****************************************************************************
 * Name: stm32_fmc_enable
 *
 * Description:
 *   Enable clocking to the FMC.
 *
 ****************************************************************************/

void stm32_fmc_enable(void)
{
  uint32_t val = getreg32(STM32_RCC_AHB3ENR);
  val |= RCC_AHB3ENR_FMCEN;
  putreg32(val, STM32_RCC_AHB3ENR);
}

/****************************************************************************
 * Name: stm32_fmc_disable
 *
 * Description:
 *   Disable clocking to the FMC.
 *
 ****************************************************************************/

void stm32_fmc_disable(void)
{
  uint32_t val = getreg32(STM32_RCC_AHB3ENR);
  val &= ~RCC_AHB3ENR_FMCEN;
  putreg32(val, STM32_RCC_AHB3ENR);
}

/****************************************************************************
 * Name: stm32_fmc_sdram_write_enable
 *
 * Description:
 *   Enable writes to an SDRAM.
 *
 ****************************************************************************/

void stm32_fmc_sdram_write_enable(void)
{
  uint32_t val = getreg32(STM32_FMC_SDCR1);
  stm32_fmc_sdram_wait();
  val &= ~(1 << 9);             /* wp == 0 */
  putreg32(val, STM32_FMC_SDCR1);
}

/****************************************************************************
 * Name: stm32_fmc_sdram_write_disable
 *
 * Description:
 *   Disable writes to an SDRAM.
 *
 ****************************************************************************/

void stm32_fmc_sdram_write_disable(void)
{
  uint32_t val = getreg32(STM32_FMC_SDCR1);
  stm32_fmc_sdram_wait();
  val |= (1 << 9);              /* wp == 1 */
  putreg32(val, STM32_FMC_SDCR1);
}

/****************************************************************************
 * Name: stm32_fmc_sdram_set_refresh_rate
 *
 * Description:
 *   Set the SDRAM refresh rate.
 *
 ****************************************************************************/

void stm32_fmc_sdram_set_refresh_rate(int count)
{
  uint32_t val;

  count &= 0x1fff;              /*13 bits */
  DEBUGASSERT(count >= 0x29);

  stm32_fmc_sdram_wait();
  val = getreg32(STM32_FMC_SDRTR);
  val &= ~(0x1fff << 1);        /*preserve non-count bits */
  val |= (count << 1);
  putreg32(val, STM32_FMC_SDRTR);
}

#endif /* CONFIG_STM32_FMC */
