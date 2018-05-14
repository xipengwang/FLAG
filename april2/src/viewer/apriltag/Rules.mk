.PHONY: viewer_apriltag viewer_apriltag_clean

viewer_apriltag: apriltag camera graph imagesource april_lcmtypes vx

viewer_apriltag:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/viewer/apriltag -f Build.mk

viewer_apriltag_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/viewer/apriltag -f Build.mk clean

all: viewer_apriltag

clean: viewer_apriltag_clean
