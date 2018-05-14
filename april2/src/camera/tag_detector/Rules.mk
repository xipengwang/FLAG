.PHONY: tag_detector tag_detector_clean

tag_detector: april_lcmtypes apriltag

tag_detector:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/camera/tag_detector -f Build.mk

tag_detector_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/camera/tag_detector -f Build.mk clean

all: tag_detector

clean: tag_detector_clean
