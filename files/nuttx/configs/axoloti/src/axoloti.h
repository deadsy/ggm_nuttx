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
// LEDs

#define GPIO_LED1 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN6)
#define GPIO_LED2 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN6)

//-----------------------------------------------------------------------------

#endif				// __CONFIGS_AXOLOTI_SRC_AXOLOTI_H

//-----------------------------------------------------------------------------
