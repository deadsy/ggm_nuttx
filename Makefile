
DL = $(PWD)/dl

NUTTX_SITE = https://bitbucket.org/nuttx/nuttx/downloads
NUTTX_VER = 7.28
NUTTX_NAME = nuttx-$(NUTTX_VER)
NUTTX_TGZ = $(NUTTX_NAME).tar.gz
APPS_NAME = apps-$(NUTTX_VER)
APPS_TGZ = $(APPS_NAME).tar.gz

PATCHFILES := $(sort $(wildcard patches/*.patch ))

PATCH_CMD = \
  for f in $(PATCHFILES); do\
      echo $$f ":"; \
      patch -d $(OS_DIR) -p1 < $$f || exit 1; \
  done

COPY_CMD = tar cf - -C files . | tar xf - -C $(OS_DIR)

.PHONY: all
all: $(NUTTX_TGZ) $(APPS_TGZ)
	#$(COPY_CMD)
	#$(PATCH_CMD)

$(NUTTX_TGZ):
	mkdir -p $(DL)
	wget -P $(DL) $(NUTTX_SITE)/$(NUTTX_TGZ)
	echo "92fde612a542c47d11eb0bd85dc1d53ccf236f8106ad6b216fdfbc77f3b8ce1d *$(DL)/$(NUTTX_TGZ)" | sha256sum --check --strict

$(APPS_TGZ):
	mkdir -p $(DL)
	wget -P $(DL) $(NUTTX_SITE)/$(APPS_TGZ)
	echo "8325b36bbe992474ddcb7bb965804ce45d7959ef18fefa35c3c9948089ec9fc5 *$(DL)/$(APPS_TGZ)" | sha256sum --check --strict

.PHONY: clean
clean:
	#-rm -rf $(OS_DIR)

.PHONY: distclean
distclean:
	-rm -rf $(DL)
