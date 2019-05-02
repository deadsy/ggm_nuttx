//-----------------------------------------------------------------------------
/*

Axoloti Board USB Host Code

Note: The Axoloti can only support USB host operations on the USB port
connetced to the OTG_HS controller.

*/
//-----------------------------------------------------------------------------

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <sched.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kthread.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbdev_trace.h>

#include "up_arch.h"
#include "stm32.h"
#include "stm32_otghs.h"
#include "axoloti.h"

//-----------------------------------------------------------------------------

// Setup USB-related GPIO pins
void stm32_usbinitialize(void)
{
  // The OTG HS has an internal soft pull-up.  No GPIO configuration is required
  // Configure the OTG HS VBUS sensing GPIO, Power On, and Overcurrent GPIOs
  //stm32_configgpio(GPIO_OTGHS_VBUS);
  stm32_configgpio(GPIO_OTGHS_PWRON);
  stm32_configgpio(GPIO_OTGHS_OVER);
}

// Enable/disable driving of VBUS 5V output.
void stm32_usbhost_vbusdrive(int iface, bool enable)
{
  DEBUGASSERT(iface == 0);
  if (enable)
    {
      // Enable the Power Switch by driving the enable pin low
      stm32_gpiowrite(GPIO_OTGHS_PWRON, false);
    }
  else
    {
      // Disable the Power Switch by driving the enable pin high
      stm32_gpiowrite(GPIO_OTGHS_PWRON, true);
    }
}

// Setup to receive an interrupt-level callback if an overcurrent condition is detected.
int stm32_setup_overcurrent(xcpt_t handler, void *arg)
{
  return stm32_gpiosetevent(GPIO_OTGHS_OVER, true, true, true, handler, arg);
}

//-----------------------------------------------------------------------------

#define CONFIG_AXOLOTI_USBHOST_PRIO 100
#define CONFIG_AXOLOTI_USBHOST_STACKSIZE 1024

static struct usbhost_connection_s *g_usbconn;

// Wait for USB devices to be connected.
static int usbhost_waiter(int argc, char *argv[])
{
  struct usbhost_hubport_s *hport;
  uinfo("Running\n");
  for (;;)
    {
      // Wait for the device to change state
      DEBUGVERIFY(CONN_WAIT(g_usbconn, &hport));
      uinfo("%s\n", hport->connected ? "connected" : "disconnected");
      // Did we just become connected?
      if (hport->connected)
        {
          // Yes.. enumerate the newly connected device
          (void)CONN_ENUMERATE(g_usbconn, hport);
        }
    }
  // Keep the compiler from complaining
  return 0;
}

// This function will start a thread that will monitor for device connection/disconnection events.
int stm32_usbhost_initialize(void)
{
  int pid;
  int ret;

  // First, register all of the class drivers needed to support the drivers that we care about:
  uinfo("Register class drivers\n");

#ifdef CONFIG_USBHOST_MSC
  // Register the USB mass storage class class
  ret = usbhost_msc_initialize();
  if (ret != OK)
    {
      uerr("ERROR: Failed to register the mass storage class: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_HIDKBD
  // Initialize the HID keyboard class
  ret = usbhost_kbdinit();
  if (ret != OK)
    {
      uerr("ERROR: Failed to register the HID keyboard class\n");
    }
#endif

#ifdef CONFIG_USBHOST_HIDMOUSE
  // Initialize the HID mouse class
  ret = usbhost_mouse_init();
  if (ret != OK)
    {
      uerr("ERROR: Failed to register the HID mouse class\n");
    }
#endif

  // Then get an instance of the USB host interface
  uinfo("Initialize USB host\n");
  g_usbconn = stm32_otghshost_initialize(0);
  if (g_usbconn)
    {
      // Start a thread to handle device connection.
      uinfo("Start usbhost_waiter\n");
      pid =
        kthread_create("usbhost", CONFIG_AXOLOTI_USBHOST_PRIO,
                       CONFIG_AXOLOTI_USBHOST_STACKSIZE,
                       (main_t) usbhost_waiter, (FAR char *const *)NULL);
      return pid < 0 ? -ENOEXEC : OK;
    }
  return -ENODEV;
}

//-----------------------------------------------------------------------------
