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

int stm32_sdio_initialize(void);
int stm32_bringup(void);
void stm32_usbinitialize(void);
int stm32_usbhost_initialize(void);
int rei2c_initialize(char *devname);

//-----------------------------------------------------------------------------

// procfs
#define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT

// SDIO Configuration
#define HAVE_SDIO 1
#define SDIO_MINOR CONFIG_NSH_MMCSDMINOR
#define SDIO_SLOTNO CONFIG_NSH_MMCSDSLOTNO

// GPIO for card detect
#define GPIO_SDIO_NCD (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN13)

// LEDs
#define GPIO_LED1 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN6)
#define GPIO_LED2 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN6)

// Buttons
#define GPIO_BTN_USER (GPIO_INPUT|GPIO_PULLDOWN|GPIO_EXTI|GPIO_PORTA|GPIO_PIN10)
#define MIN_IRQBUTTON BUTTON_USER
#define MAX_IRQBUTTON BUTTON_USER
#define NUM_IRQBUTTONS  1

// USB
#undef HAVE_USBDEV
#define HAVE_USBHOST 1
#undef HAVE_USBMONITOR

#define GPIO_OTGHS_PWRON (GPIO_OUTPUT|GPIO_OUTPUT_SET|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN7)
#define GPIO_OTGHS_OVER (GPIO_INPUT|GPIO_EXTI|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTG|GPIO_PIN13)
//#define GPIO_OTGHS_VBUS (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTB|GPIO_PIN13)

//-----------------------------------------------------------------------------

#endif				// __CONFIGS_AXOLOTI_SRC_AXOLOTI_H

//-----------------------------------------------------------------------------
