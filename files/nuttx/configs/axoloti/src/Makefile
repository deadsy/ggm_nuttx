
-include $(TOPDIR)/Make.defs

ASRCS =
CSRCS = boot.c 

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += autoleds.c
else
CSRCS += userleds.c
endif

include $(TOPDIR)/configs/Board.mk
