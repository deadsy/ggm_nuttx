
DL_DIR = $(PWD)/dl

NUTTX_VER = 7.2.8
NUTTX_NAME = nuttx-$(NUTTX_VER)
NUTTX_TGZ = $(DL_DIR)/$(NUTTX_NAME).tar.gz
APPS_NAME = apps-$(NUTTX_VER)
APPS_TGZ = $(DL_DIR)/$(APPS_NAME).tar.gz

PATCHFILES := $(sort $(wildcard patches/*.patch ))

PATCH_CMD = \
  for f in $(PATCHFILES); do\
      echo $$f ":"; \
      patch -d $(OS_DIR) -p1 < $$f || exit 1; \
  done

COPY_CMD = tar cf - -C files . | tar xf - -C $(OS_DIR)

.PHONY: all
all: $(NUTTX_TGZ) $(APPS_TGZ)
	$(COPY_CMD)
	$(PATCH_CMD)

$(NUTTX_TGZ):
	mkdir -p $(DL_DIR)
	wget -P $(DL_DIR) --no-check-certificate https://bitbucket.org/nuttx/nuttx/downloads/$(NUTTX_NAME).tar.gz

	#echo "319a44e3914986a483f7d5e5c257f9626f9e804c *$(BR_TBZ)" | sha1sum --check --strict
	#echo "3084dafe690261c0cd9a57093f2943c9ff5d13b47a4fa8c15f49c1696a63b08a *$(BR_TBZ)" | sha256sum --check --strict

$(APPS_TGZ):
	mkdir -p $(DL_DIR)
	wget -P $(DL_DIR) "https://bitbucket.org/nuttx/nuttx/downloads/$(APPS_NAME).tar.gz"

.PHONY: clean
clean:
	-rm -rf $(OS_DIR)
