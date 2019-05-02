//-----------------------------------------------------------------------------
/*

Axoloti Board SDIO Support

*/
//-----------------------------------------------------------------------------

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

#include "stm32.h"
#include "axoloti.h"

//-----------------------------------------------------------------------------

#ifdef HAVE_SDIO

//-----------------------------------------------------------------------------
// Card Detection

static FAR struct sdio_dev_s *g_sdio_dev;
static bool g_sd_inserted = 0xff;       /* Impossible value */

// Card detect interrupt handler.
static int stm32_ncd_interrupt(int irq, FAR void *context, FAR void *arg)
{
  bool present;
  present = !stm32_gpioread(GPIO_SDIO_NCD);
  if (present != g_sd_inserted)
    {
      sdio_mediachange(g_sdio_dev, present);
      g_sd_inserted = present;
    }
  return OK;
}

//-----------------------------------------------------------------------------
// Public

// Initialize SDIO-based MMC/SD card support
int stm32_sdio_initialize(void)
{
  bool cd_status;
  int ret;

  // Configure the card detect GPIO
  stm32_configgpio(GPIO_SDIO_NCD);
  // Register an interrupt handler for the card detect pin
  (void)stm32_gpiosetevent(GPIO_SDIO_NCD, true, true, true, stm32_ncd_interrupt,
                           NULL);

  // Mount the SDIO-based MMC/SD block driver
  // First, get an instance of the SDIO interface
  finfo("Initializing SDIO slot %d\n", SDIO_SLOTNO);
  g_sdio_dev = sdio_initialize(SDIO_SLOTNO);
  if (!g_sdio_dev)
    {
      ferr("ERROR: Failed to initialize SDIO slot %d\n", SDIO_SLOTNO);
      return -ENODEV;
    }
  // Now bind the SDIO interface to the MMC/SD driver
  finfo("Bind SDIO to the MMC/SD driver, minor=%d\n", SDIO_MINOR);
  ret = mmcsd_slotinitialize(SDIO_MINOR, g_sdio_dev);
  if (ret != OK)
    {
      ferr("ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
      return ret;
    }

  finfo("Successfully bound SDIO to the MMC/SD driver\n");

  // Use SD card detect pin to check if a card is g_sd_inserted
  cd_status = !stm32_gpioread(GPIO_SDIO_NCD);
  finfo("Card detect : %d\n", cd_status);
  sdio_mediachange(g_sdio_dev, cd_status);

  return OK;
}

//-----------------------------------------------------------------------------

#endif // HAVE_SDIO

//-----------------------------------------------------------------------------
