//-----------------------------------------------------------------------------
/*

Axoloti Board LED Support

*/
//-----------------------------------------------------------------------------

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/power/pm.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

#include "stm32.h"
#include "axoloti.h"

//-----------------------------------------------------------------------------

#ifndef CONFIG_ARCH_LEDS

//-----------------------------------------------------------------------------

// This array maps an LED number to GPIO pin configuration
static uint32_t g_ledcfg[BOARD_NLEDS] = {
  GPIO_LED1, GPIO_LED2,
};

void board_userled_initialize(void)
{
  // Configure LED1-2 GPIOs for output
  stm32_configgpio(GPIO_LED1);
  stm32_configgpio(GPIO_LED2);
}

void board_userled(int led, bool ledon)
{
  if ((unsigned)led < BOARD_NLEDS)
    {
      stm32_gpiowrite(g_ledcfg[led], ledon);
    }
}

void board_userled_all(uint8_t ledset)
{
  stm32_gpiowrite(GPIO_LED1, (ledset & BOARD_LED1_BIT) == 0);
  stm32_gpiowrite(GPIO_LED2, (ledset & BOARD_LED2_BIT) == 0);
}

//-----------------------------------------------------------------------------

#endif // !CONFIG_ARCH_LEDS

//-----------------------------------------------------------------------------
