.PHONY: thumbs thumbs_clean

thumbs: camera common

thumbs:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/camera/thumbnail_publisher -f Build.mk

thumbs_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/camera/thumbnail_publisher -f Build.mk clean

all: thumbs

clean: thumbs_clean

