TOP = ..
include $(TOP)/mk/common.mk

NUTTX_TGZ = $(PWD)/../dl/nuttx-$(NUTTX_HASH).tar.gz
APPS_TGZ = $(PWD)/../dl/apps-$(APPS_HASH).tar.gz

.PHONY: update
update: nuttx apps
	git -C nuttx pull
	git -C apps pull

nuttx:
	git clone https://bitbucket.org/nuttx/nuttx.git

apps:
	git clone https://bitbucket.org/nuttx/apps.git

archive:
	-rm $(NUTTX_TGZ)
	-rm $(APPS_TGZ)
	cd nuttx; git archive -o $(NUTTX_TGZ) $(NUTTX_HASH); cd ..
	cd apps; git archive -o $(APPS_TGZ) $(APPS_HASH); cd ..
