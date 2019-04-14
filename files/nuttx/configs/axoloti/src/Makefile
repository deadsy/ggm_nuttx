-include $(TOPDIR)/Make.defs

ASRCS =
CSRCS = boot.c bringup.c

CSRCS += userleds.c

ifeq ($(CONFIG_STM32_SDIO),y)
CSRCS += sdio.c
endif

ifeq ($(CONFIG_USBHOST),y)
CSRCS += usbhost.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS +=  buttons.c
endif

ifeq ($(CONFIG_INPUT_REI2C),y)
  CSRCS += rei2c.c
endif

include $(TOPDIR)/configs/Board.mk
