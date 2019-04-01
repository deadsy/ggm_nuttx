//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------

#ifndef __CONFIG_AXOLOTI_INCLUDE_BOARD_H
#define __CONFIG_AXOLOTI_INCLUDE_BOARD_H

//-----------------------------------------------------------------------------

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#include <stdint.h>
#include <stdbool.h>
#endif

//-----------------------------------------------------------------------------
/* Clocking
/* The Axoloti board has an external 8MHz crystal.
 * The SoC can run at 180MHz, but the required USB clock of 48MHz cannot be
 * configured at that system clock rate, so the core clock is 168MHz.
 *
 * This is the canonical configuration:
 *   System Clock source           : PLL (HSE)
 *   SYSCLK(Hz)                    : 168000000    Determined by PLL configuration
 *   HCLK(Hz)                      : 168000000    (STM32_RCC_CFGR_HPRE)
 *   AHB Prescaler                 : 1            (STM32_RCC_CFGR_HPRE)
 *   APB1 Prescaler                : 4            (STM32_RCC_CFGR_PPRE1)
 *   APB2 Prescaler                : 2            (STM32_RCC_CFGR_PPRE2)
 *   HSE Frequency(Hz)             : 8000000      (STM32_BOARD_XTAL)
 *   PLLM                          : 8            (STM32_PLLCFG_PLLM)
 *   PLLN                          : 336          (STM32_PLLCFG_PLLN)
 *   PLLP                          : 2            (STM32_PLLCFG_PLLP)
 *   PLLQ                          : 7            (STM32_PLLCFG_PLLQ)
 *   Main regulator output voltage : Scale1 mode  Needed for high speed SYSCLK
 *   Flash Latency(WS)             : 5
 *   Prefetch Buffer               : OFF
 *   Instruction cache             : ON
 *   Data cache                    : ON
 *   Require 48MHz for USB OTG FS, : Enabled
 *   SDIO and RNG clock
 */

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * HSE - On-board crystal frequency is 8MHz
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL        8000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* Main PLL Configuration.
 *
 * PLL source is HSE
 * PLL_VCO = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 *         = (8,000,000 / 8) * 336
 *         = 336,000,000
 * SYSCLK  = PLL_VCO / PLLP
 *         = 336,000,000 / 2 = 168,000,000
 * USB OTG FS, SDIO and RNG Clock
 *         =  PLL_VCO / PLLQ
 *         = 48,000,000
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(8)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(336)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(7)

#define STM32_SYSCLK_FREQUENCY  168000000ul

// AHB clock (HCLK) is SYSCLK (168MHz)
#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK	/* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY	/* same as above, to satisfy compiler */

// APB1 clock (PCLK1) is HCLK/4 (42MHz)
#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd4	/* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/4)

// APB2 clock (PCLK2) is HCLK/2 (84MHz)
#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2	/* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

//-----------------------------------------------------------------------------
// LEDs

//LED index values for use with board_userled()
#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_NLEDS       2
#define BOARD_LED_GREEN   BOARD_LED1
#define BOARD_LED_RED     BOARD_LED2

// LED bits for use with board_userled_all()
#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)

//-----------------------------------------------------------------------------

// USART6 - midi in/out
#define GPIO_USART6_RX GPIO_USART6_RX_2	// (AF8, PG9)
#define GPIO_USART6_TX GPIO_USART6_TX_2	// (AF8, PG14)

//-----------------------------------------------------------------------------
// MIDI

// In - USART6, PG9
// Out - USART6, PG14

//-----------------------------------------------------------------------------

#endif				// __CONFIG_AXOLOTI_INCLUDE_BOARD_H

//-----------------------------------------------------------------------------
