.PHONY: viewer_image viewer_image_clean

viewer_image: common april_lcmtypes vx

viewer_image:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/viewer/image -f Build.mk

viewer_image_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/viewer/image -f Build.mk clean

all: viewer_image

clean: viewer_image_clean
