SUBDIRS = src java
MBOT_SUBDIRS = src
MAKEFLAGS += --no-print-directory

laptop-only:
	@for dir in $(MBOT_SUBDIRS); do \
	echo "[$$dir]"; $(MAKE) -C $$dir laptop-only || exit 2; done

all:
	@for dir in $(SUBDIRS); do \
	echo "[$$dir]"; $(MAKE) -C $$dir all || exit 2; done

mbot-only:
	@for dir in $(MBOT_SUBDIRS); do \
	echo "[$$dir]"; $(MAKE) -C $$dir mbot-only || exit 2; done
	
clean:
	@for dir in $(SUBDIRS); do \
	echo "clean [$$dir]"; $(MAKE) -C $$dir clean || exit 2; done
	@rm -f *~
