TOP = ..
include $(TOP)/mk/common.mk

NUTTX_TGZ = $(PWD)/../dl/nuttx-$(NUTTX_HASH).tar.gz
APPS_TGZ = $(PWD)/../dl/apps-$(APPS_HASH).tar.gz

.PHONY: update
update:
	git -C nuttx fetch upstream
	git -C nuttx merge upstream/master
	git -C apps fetch upstream
	git -C apps merge upstream/master

archive:
	mkdir -p $(TOP)/dl
	-rm $(NUTTX_TGZ)
	-rm $(APPS_TGZ)
	cd nuttx; git archive -o $(NUTTX_TGZ) $(NUTTX_HASH); cd ..
	cd apps; git archive -o $(APPS_TGZ) $(APPS_HASH); cd ..
