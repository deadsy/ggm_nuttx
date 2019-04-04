-include $(TOPDIR)/Make.defs

ASRCS =
CSRCS = boot.c bringup.c

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += autoleds.c
else
CSRCS += userleds.c
endif

ifeq ($(CONFIG_STM32_SDIO),y)
CSRCS += sdio.c
endif

include $(TOPDIR)/configs/Board.mk
