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
	/* Make sure the high speed cycle counter is running.  It will be started
	 * automatically only if a debugger is connected.
	 */

	putreg32(0xc5acce55, ITM_LAR);
	modifyreg32(DWT_CTRL, 0, DWT_CTRL_CYCCNTENA_MASK);
#endif

#if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI2) || defined(CONFIG_STM32_SPI3)
	/* Configure SPI chip selects if 1) SPI is not disabled, and 2) the weak
	 * function stm32_spidev_initialize() has been brought into the link.
	 */

	if (stm32_spidev_initialize) {
		stm32_spidev_initialize();
	}
#endif

#ifdef CONFIG_STM32_OTGFS
	/* Initialize USB if the 1) OTG FS controller is in the configuration and 2)
	 * disabled, and 3) the weak function stm32_usbinitialize() has been brought
	 * into the build. Presumably either CONFIG_USBDEV or CONFIG_USBHOST is also
	 * selected.
	 */

	if (stm32_usbinitialize) {
		stm32_usbinitialize();
	}
#endif

#ifdef CONFIG_ARCH_LEDS
	/* Configure on-board LEDs if LED support has been selected. */

	board_autoled_initialize();
#endif
}

//-----------------------------------------------------------------------------

// If CONFIG_BOARD_INITIALIZE is selected, then an additional
// initialization call will be performed in the boot-up sequence to a
// function called board_initialize().  board_initialize() will be
// called immediately after up_initialize() is called and just before the
// initial application is started.  This additional initialization phase
// may be used, for example, to initialize board-specific device drivers.

#ifdef CONFIG_BOARD_INITIALIZE
void board_initialize(void) {
	/* Perform board-specific initialization */

	(void)stm32_bringup();
}
#endif

//-----------------------------------------------------------------------------
