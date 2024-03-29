TOP = .

#BOARD = axoloti
#BOARD = stm32f4discovery
#BOARD = stm32f429i-disco
BOARD = imxrt1020-evk
#BOARD = imxrt1050-evk

CONFIG = nsh
#CONFIG = ggm
#CONFIG = audio

BOARD_CONFIG = $(BOARD)/$(CONFIG)

NUTTX_REPO = $(TOP)/nuttx
APPS_REPO = $(TOP)/apps

BUILD = $(TOP)/build
NUTTX_BUILD = $(BUILD)/nuttx
APPS_BUILD = $(BUILD)/apps

ELF_FILE = $(NUTTX_BUILD)/nuttx
BIN_FILE = $(NUTTX_BUILD)/nuttx.bin

XTOOLS = /opt/gcc-arm-none-eabi-8-2018-q4-major/bin/arm-none-eabi-

JLINK = /opt/SEGGER/JLink/JLinkExe
JLINK_CMD = cmd.jlink

RESET_CMD =	st-flash reset

.PHONY: all
all: .stamp_build

.PHONY: src
src: .stamp_src

.PHONY: reset
reset:
	$(RESET_CMD)

.PHONY: flash_st
flash:
	st-flash write $(BIN_FILE) 0x08000000
	$(RESET_CMD)

.PHONY: flash_nxp
flash_nxp:
	echo "r\nloadfile $(BIN_FILE) 0x60000000\nq" > $(JLINK_CMD)
	$(JLINK) -if swd -speed auto -device MIMXRT1021xxx5A -CommanderScript $(JLINK_CMD)

.PHONY: clean
clean:
	-rm $(JLINK_CMD)
	-rm -rf $(BUILD)
	-rm .stamp*

.PHONY: config
config:
	make -C $(NUTTX_BUILD) menuconfig
	make -C $(NUTTX_BUILD) savedefconfig
	cp $(NUTTX_BUILD)/defconfig $(NUTTX_REPO)/configs/$(BOARD_CONFIG)/defconfig

.PHONY: init
init: nuttx apps

.stamp_src: nuttx apps
	mkdir -p $(BUILD)
	rsync -aq $(NUTTX_REPO) $(BUILD) --exclude .git
	rsync -aq $(APPS_REPO) $(BUILD) --exclude .git
	touch $@

nuttx:
	git clone git@github.com:deadsy/nuttx.git
	git -C nuttx remote add upstream https://bitbucket.org/nuttx/nuttx.git 

apps:
	git clone git@github.com:deadsy/apps.git
	git -C apps remote add upstream https://bitbucket.org/nuttx/apps.git

.stamp_cfg: .stamp_src
	$(NUTTX_BUILD)/tools/configure.sh -l $(BOARD_CONFIG)
	touch $@

.stamp_build: .stamp_cfg
	CROSSDEV=$(XTOOLS) ARCROSSDEV=$(XTOOLS) make -C $(NUTTX_BUILD)
	touch $@
