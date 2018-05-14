export CFLAGS_CAMERA =
export LDFLAGS_CAMERA = $(LIB_PATH)/libcamera.a $(LDFLAGS_COMMON)
export DEPS_CAMERA = $(LIB_PATH)/libcamera.a $(DEPS_COMMON)

.PHONY: camera camera_clean

camera: common

camera:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/camera -f Build.mk

camera_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/camera -f Build.mk clean

all: camera

clean: camera_clean

include $(APRIL_PATH)/src/camera/lcm_camera_driver/Rules.mk
include $(APRIL_PATH)/src/camera/tag_detector/Rules.mk
include $(APRIL_PATH)/src/camera/thumbnail_publisher/Rules.mk
