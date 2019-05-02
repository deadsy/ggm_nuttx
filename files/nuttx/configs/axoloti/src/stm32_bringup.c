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

#ifdef CONFIG_STM32_OTGHS
#include "stm32_usbhost.h"
#endif

#include "stm32.h"
//#include "stm32_romfs.h"
#include <nuttx/input/buttons.h>
#include <nuttx/leds/userled.h>

#include "axoloti.h"

//-----------------------------------------------------------------------------

// stm32_bringup: Perform architecture-specific initialization
// CONFIG_BOARD_INITIALIZE=y: Called from board_initialize().
// CONFIG_BOARD_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y : Called from the NSH library
int stm32_bringup(void)
{
#ifdef HAVE_RTC_DRIVER
  FAR struct rtc_lowerhalf_s *lower;
#endif
  int ret = OK;

#ifdef HAVE_SDIO
  // Initialize the SDIO block driver
  ret = stm32_sdio_initialize();
  if (ret != OK)
    {
      ferr("stm32_sdio_initialize failed %d\n", ret);
      return ret;
    }
#endif

#ifdef HAVE_USBHOST
  // Initialize USB host operation.  stm32_usbhost_initialize() starts a
  // thread will monitor for USB connection and disconnection events.
  ret = stm32_usbhost_initialize();
  if (ret != OK)
    {
      uerr("stm32_usbhost_initialize failed %d\n", ret);
      return ret;
    }
#endif

#ifdef HAVE_USBMONITOR
  // Start the USB Monitor
  ret = usbmonitor_start();
  if (ret != OK)
    {
      uerr("usbmonitor_start failed %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_BUTTONS
  // Register the BUTTON driver
  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "btn_lower_initialize failed %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT_REI2C
  // register the rei2c driver
  ret = rei2c_initialize("/dev/re0");
  if (ret < 0)
    {
      syslog(LOG_ERR, "rei2c_initialize failed %d\n", ret);
    }
#endif

#ifdef CONFIG_USERLED
  // Register the LED driver
  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "userled_lower_initialize failed %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  // Mount the procfs file system
  ret = mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      serr("failed to mount procfs at %s: %d\n", STM32_PROCFS_MOUNTPOINT, ret);
    }
#endif

  return ret;
}

//-----------------------------------------------------------------------------
