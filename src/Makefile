# Only add your directories to this folder if you are 100% sure that
# it will always compile without warnings.
SUBDIRS = lcmtypes common imagesource vx/math vx vx/gtk slam planning apps mbot optitrack
MBOT_SUBDIRS = lcmtypes common mbot

MAKEFLAGS += --no-print-directory

mbot-only:
	@$(MAKE) -f Makefile.lcmgen all || exit 2;
	@for dir in $(MBOT_SUBDIRS); do \
	echo "[$$dir]"; $(MAKE) -C $$dir all || exit 2; done

all:
	@$(MAKE) -f Makefile.lcmgen all || exit 2;
	@for dir in $(SUBDIRS); do \
	echo "[$$dir]"; $(MAKE) -C $$dir all || exit 2; done

clean:
	@for dir in $(SUBDIRS); do \
	echo "clean [$$dir]"; $(MAKE) -C $$dir clean || exit 2; done
	@rm -f *~
