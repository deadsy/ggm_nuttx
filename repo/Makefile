
.PHONY: update
update: nuttx apps
	git -C nuttx pull
	git -C apps pull

nuttx:
	git clone https://bitbucket.org/nuttx/nuttx.git

apps:
	git clone https://bitbucket.org/nuttx/apps.git

