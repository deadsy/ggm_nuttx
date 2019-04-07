//-----------------------------------------------------------------------------
/*

Axoloti Board Boot Code

*/
//-----------------------------------------------------------------------------

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "itm.h"

#include "stm32.h"
#include "axoloti.h"

//-----------------------------------------------------------------------------

// All STM32 architectures must provide the following entry point.  This
// entry point is called early in the initialization -- after all memory
// has been configured and mapped but before any devices have been initialized.
void stm32_boardinitialize(void) {
#ifdef CONFIG_SCHED_CRITMONITOR
	// Make sure the high speed cycle counter is running.  It will be started
	// automatically only if a debugger is connected.
	putreg32(0xc5acce55, ITM_LAR);
	modifyreg32(DWT_CTRL, 0, DWT_CTRL_CYCCNTENA_MASK);
#endif

#if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI2) || defined(CONFIG_STM32_SPI3)
	stm32_spidev_initialize();
#endif

#ifdef CONFIG_STM32_OTGHS
	stm32_usbinitialize();
#endif
}

//-----------------------------------------------------------------------------

// Perform board-specific initialization
#ifdef CONFIG_BOARD_INITIALIZE
void board_initialize(void) {
	(void)stm32_bringup();
}
#endif

//-----------------------------------------------------------------------------
