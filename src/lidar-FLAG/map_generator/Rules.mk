.PHONY: map_generator_clean map_generator

map_generator:  common vx lcmtypes graph

map_generator:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/lidar-FLAG/map_generator -f Build.mk

map_generator_clean:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/lidar-FLAG/map_generator -f Build.mk clean

all: map_generator

clean:  map_generator_clean
