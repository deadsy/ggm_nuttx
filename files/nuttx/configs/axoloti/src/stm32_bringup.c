/****************************************************************************
 * config/axoloti/src/stm32_bringup.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Jason T. Harris <sirmanlypowers@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

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

#ifdef CONFIG_STM32_OTGHS
#include "stm32_usbhost.h"
#endif

#ifdef CONFIG_BUTTONS
#include <nuttx/input/buttons.h>
#endif

#ifdef CONFIG_USERLED
#include <nuttx/leds/userled.h>
#endif

#include "axoloti.h"

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

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

/****************************************************************************/
