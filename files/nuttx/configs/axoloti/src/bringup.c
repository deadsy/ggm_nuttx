//-----------------------------------------------------------------------------
/*

Axoloti Board Bringup

*/
//-----------------------------------------------------------------------------

#include <nuttx/config.h>

#include <sys/mount.h>
#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#ifdef CONFIG_USBMONITOR
#include <nuttx/usb/usbmonitor.h>
#endif

#include "stm32.h"
//#include "stm32_romfs.h"

#ifdef CONFIG_STM32_OTGFS
#include "stm32_usbhost.h"
#endif

#ifdef CONFIG_BUTTONS
#include <nuttx/input/buttons.h>
#endif

#ifdef CONFIG_USERLED
#include <nuttx/leds/userled.h>
#endif

#include "axoloti.h"

//-----------------------------------------------------------------------------

// stm32_i2c_register: Register one I2C drivers for the I2C tool.
#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
static void stm32_i2c_register(int bus) {
	FAR struct i2c_master_s *i2c;
	int ret;

	i2c = stm32_i2cbus_initialize(bus);
	if (i2c == NULL) {
		syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", bus);
	} else {
		ret = i2c_register(i2c, bus);
		if (ret < 0) {
			syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n", bus, ret);
			stm32_i2cbus_uninitialize(i2c);
		}
	}
}
#endif

// stm32_i2ctool: Register I2C drivers for the I2C tool.
#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
static void stm32_i2ctool(void) {
	stm32_i2c_register(1);
#if 0
	stm32_i2c_register(1);
	stm32_i2c_register(2);
#endif
}
#else
#define stm32_i2ctool()
#endif

//-----------------------------------------------------------------------------

// stm32_bringup: Perform architecture-specific initialization
// CONFIG_BOARD_INITIALIZE=y: Called from board_initialize().
// CONFIG_BOARD_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y : Called from the NSH library
int stm32_bringup(void) {
#ifdef HAVE_RTC_DRIVER
	FAR struct rtc_lowerhalf_s *lower;
#endif
	int ret = OK;

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
	stm32_i2ctool();
#endif

#ifdef HAVE_SDIO
	/* Initialize the SDIO block driver */
	ret = stm32_sdio_initialize();
	if (ret != OK) {
		ferr("ERROR: Failed to initialize MMC/SD driver: %d\n", ret);
		return ret;
	}
#endif

#ifdef HAVE_USBHOST
	/* Initialize USB host operation.  stm32_usbhost_initialize() starts a
	 * thread will monitor for USB connection and disconnection events.
	 */
	ret = stm32_usbhost_initialize();
	if (ret != OK) {
		uerr("ERROR: Failed to initialize USB host: %d\n", ret);
		return ret;
	}
#endif

#ifdef HAVE_USBMONITOR
	/* Start the USB Monitor */
	ret = usbmonitor_start();
	if (ret != OK) {
		uerr("ERROR: Failed to start USB monitor: %d\n", ret);
		return ret;
	}
#endif

#ifdef CONFIG_BUTTONS
	/* Register the BUTTON driver */
	ret = btn_lower_initialize("/dev/buttons");
	if (ret < 0) {
		syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
	}
#endif

#ifdef CONFIG_USERLED
	/* Register the LED driver */
	ret = userled_lower_initialize("/dev/userleds");
	if (ret < 0) {
		syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
	}
#endif

#ifdef CONFIG_FS_PROCFS
	/* Mount the procfs file system */
	ret = mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
	if (ret < 0) {
		serr("ERROR: Failed to mount procfs at %s: %d\n", STM32_PROCFS_MOUNTPOINT, ret);
	}
#endif

	return ret;
}

//-----------------------------------------------------------------------------
