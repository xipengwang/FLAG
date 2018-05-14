.PHONY: pf_clean pf

pf:  common vx lcmtypes graph

pf:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/lidar-FLAG/particle-filter-localization -f Build.mk

pf_clean:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/lidar-FLAG/particle-filter-localization -f Build.mk clean

all: pf

clean: pf_clean
