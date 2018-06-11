.PHONY: velodyne_to_corner_clean velodyne_to_corner

velodyne_to_corner:  common vx lcmtypes velodyne

velodyne_to_corner:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/velodyne-to-corner -f Build.mk

velodyne_to_corner_clean:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/velodyne-to-corner -f Build.mk clean

all: velodyne_to_corner

clean: velodyne_to_corner_clean
