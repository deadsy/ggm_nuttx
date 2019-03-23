
BOARD = stm32f4discovery
CONFIG = audio

#BOARD = axoloti
#CONFIG = ggm

XTOOLS = /opt/gcc-arm-none-eabi-7-2018-q2-update/bin/arm-none-eabi-

DL = $(PWD)/dl
SRC = $(PWD)/src

NUTTX_SITE = https://bitbucket.org/nuttx/nuttx/downloads
NUTTX_VER = 7.28

NUTTX_NAME = nuttx-$(NUTTX_VER)
NUTTX_TGZ = $(DL)/$(NUTTX_NAME).tar.gz
NUTTX_SRC = $(SRC)/nuttx

APPS_NAME = apps-$(NUTTX_VER)
APPS_TGZ = $(DL)/$(APPS_NAME).tar.gz
APPS_SRC = $(SRC)/apps

PATCHFILES := $(sort $(wildcard patches/*.patch))

PATCH_CMD = \
  for f in $(PATCHFILES); do\
      echo $$f ":"; \
      patch -d $(SRC) -p1 < $$f || exit 1; \
  done

COPY_CMD = tar cf - -C files . | tar xf - -C $(SRC)

.PHONY: all
all: .stamp_build

.PHONY: src
src: .stamp_src

.PHONY: clean
clean:
	-rm -rf $(SRC)
	-rm .stamp*

.PHONY: distclean
distclean: clean
	-rm -rf $(DL)

.stamp_src: $(NUTTX_TGZ) $(APPS_TGZ)
	mkdir -p $(NUTTX_SRC)
	tar -C $(NUTTX_SRC) --strip-components=1 -xzf $(NUTTX_TGZ)
	mkdir -p $(APPS_SRC)
	tar -C $(APPS_SRC) --strip-components=1 -xzf $(APPS_TGZ)
	$(PATCH_CMD)
	$(COPY_CMD)
	touch $@

.stamp_cfg: .stamp_src
	$(NUTTX_SRC)/tools/configure.sh -l $(BOARD)/$(CONFIG)
	touch $@

.stamp_build: .stamp_cfg
	CROSSDEV=$(XTOOLS) ARCROSSDEV=$(XTOOLS) make -C $(NUTTX_SRC)
	touch $@

$(NUTTX_TGZ):
	mkdir -p $(DL)
	wget -P $(DL) $(NUTTX_SITE)/$(NUTTX_NAME).tar.gz
	echo "92fde612a542c47d11eb0bd85dc1d53ccf236f8106ad6b216fdfbc77f3b8ce1d *$(NUTTX_TGZ)" | sha256sum --check --strict

$(APPS_TGZ):
	mkdir -p $(DL)
	wget -P $(DL) $(NUTTX_SITE)/$(APPS_NAME).tar.gz
	echo "8325b36bbe992474ddcb7bb965804ce45d7959ef18fefa35c3c9948089ec9fc5 *$(APPS_TGZ)" | sha256sum --check --strict
