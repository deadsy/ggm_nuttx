//-----------------------------------------------------------------------------
/*

Axoloti Board Button Code

*/
//-----------------------------------------------------------------------------

#include <nuttx/config.h>

#include <stdint.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "stm32.h"
#include "axoloti.h"

//-----------------------------------------------------------------------------

static const uint32_t g_buttons[NUM_BUTTONS] = {
  GPIO_BTN_USER,
};

// board_button_initialize() must be called to initialize button resources.  After
// that, board_buttons() may be called to collect the current state of all
// buttons or board_button_irq() may be called to register button interrupt handlers.

void board_button_initialize(void)
{
  int i;
  // Configure the GPIO pins as inputs.
  // EXTI interrupts are configured for all pins.
  for (i = 0; i < NUM_BUTTONS; i++)
    {
      stm32_configgpio(g_buttons[i]);
    }
}

uint32_t board_buttons(void)
{
  uint32_t ret = 0;
  int i;
  // Check that state of each key
  for (i = 0; i < NUM_BUTTONS; i++)
    {
      // A HI value means that the key is pressed.
      bool pressed = stm32_gpioread(g_buttons[i]);
      // Accumulate the set of depressed (not released) keys
      if (pressed)
        {
          ret |= (1 << i);
        }
    }
  return ret;
}

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, FAR void *arg)
{
  int ret = -EINVAL;
  // The following should be atomic
  if (id >= MIN_IRQBUTTON && id <= MAX_IRQBUTTON)
    {
      ret =
        stm32_gpiosetevent(g_buttons[id], true, true, true, irqhandler, arg);
    }
  return ret;
}
#endif

//-----------------------------------------------------------------------------
