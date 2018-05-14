.PHONY: lcm_camera_driver lcm_camera_driver_clean

lcm_camera_driver: april_lcmtypes camera imagesource

lcm_camera_driver:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/camera/lcm_camera_driver -f Build.mk

lcm_camera_driver_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/camera/lcm_camera_driver -f Build.mk clean

all: lcm_camera_driver

clean: lcm_camera_driver_clean
