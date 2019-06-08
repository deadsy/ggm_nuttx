TOP = .

#BOARD_CONFIG ?= axoloti/ggm
BOARD_CONFIG ?= axoloti/nsh

#BOARD_CONFIG ?= stm32f4discovery/ggm
#BOARD_CONFIG ?= stm32f4discovery/audio

#BOARD_CONFIG ?= stm32f429i-disco/nsh

NUTTX_REPO = $(TOP)/nuttx
APPS_REPO = $(TOP)/apps

BUILD = $(TOP)/build
NUTTX_BUILD = $(BUILD)/nuttx
APPS_BUILD = $(BUILD)/apps

XTOOLS = /opt/gcc-arm-none-eabi-8-2018-q4-major/bin/arm-none-eabi-

RESET_CMD =	st-flash reset

.PHONY: all
all: .stamp_build

.PHONY: src
src: .stamp_src

.PHONY: reset
reset:
	$(RESET_CMD)

.PHONY: flash
flash:
	st-flash write $(BIN_FILE) 0x08000000
	$(RESET_CMD)

.PHONY: clean
clean:
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
