//-----------------------------------------------------------------------------
/*

  configs/axoloti/include/board.h

*/
//-----------------------------------------------------------------------------

#ifndef __CONFIG_AXOLOTI_INCLUDE_BOARD_H
#define __CONFIG_AXOLOTI_INCLUDE_BOARD_H

//-----------------------------------------------------------------------------

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

//-----------------------------------------------------------------------------
// If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in any
// way.  The following definitions are used to access individual LEDs.

//LED index values for use with board_userled()
#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_NLEDS       2
#define BOARD_LED_GREEN   BOARD_LED1
#define BOARD_LED_RED     BOARD_LED2

// LED bits for use with board_userled_all()
#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)

// If CONFIG_ARCH_LEDs is defined, then NuttX will control the 2 LEDs on board the
// Axoloti.  The following definitions describe how NuttX controls the LEDs:
#define LED_STARTED       0
#define LED_HEAPALLOCATE  1
#define LED_IRQSENABLED   2
#define LED_STACKCREATED  3
#define LED_INIRQ         3
#define LED_SIGNAL        3
#define LED_ASSERTION     3
#define LED_PANIC         3

//-----------------------------------------------------------------------------

#endif // __CONFIG_AXOLOTI_INCLUDE_BOARD_H

//-----------------------------------------------------------------------------
