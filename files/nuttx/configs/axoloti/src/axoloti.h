//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------

#ifndef __CONFIGS_AXOLOTI_SRC_AXOLOTI_H
#define __CONFIGS_AXOLOTI_SRC_AXOLOTI_H

//-----------------------------------------------------------------------------

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <arch/stm32/chip.h>

//-----------------------------------------------------------------------------
// SDIO Configuration

#define HAVE_SDIO 1

// Can't support MMC/SD features if mountpoints are disabled or if SDIO support is not enabled.
#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_STM32_SDIO)
#undef HAVE_SDIO
#endif

#undef  SDIO_MINOR		// Any minor number, default 0
#define SDIO_SLOTNO 0		// Only one slot

#ifdef HAVE_SDIO

int stm32_sdio_initialize(void);

#if !defined(CONFIG_NSH_MMCSDSLOTNO)
#define CONFIG_NSH_MMCSDSLOTNO SDIO_SLOTNO
#elif CONFIG_NSH_MMCSDSLOTNO != 0
#warning "Only one MMC/SD slot, slot 0"
#undef CONFIG_NSH_MMCSDSLOTNO
#define CONFIG_NSH_MMCSDSLOTNO SDIO_SLOTNO
#endif

#if defined(CONFIG_NSH_MMCSDMINOR)
#define SDIO_MINOR CONFIG_NSH_MMCSDMINOR
#else
#define SDIO_MINOR 0
#endif

// SD card bringup does not work if performed on the IDLE thread because it will cause waiting.  Use:
// CONFIG_LIB_BOARDCTL=y, OR
// CONFIG_BOARD_INITIALIZE=y && CONFIG_BOARD_INITTHREAD=y
#if defined(CONFIG_BOARD_INITIALIZE) && !defined(CONFIG_BOARD_INITTHREAD)
#warning "SDIO initialization cannot be perfomed on the IDLE thread"
#undef HAVE_SDIO
#endif

#endif				// HAVE_SDIO

// GPIO for card detect
#define GPIO_SDIO_NCD (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN13)

//-----------------------------------------------------------------------------
// LEDs

#define GPIO_LED1 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN6)
#define GPIO_LED2 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN6)

//-----------------------------------------------------------------------------

#endif				// __CONFIGS_AXOLOTI_SRC_AXOLOTI_H

//-----------------------------------------------------------------------------
