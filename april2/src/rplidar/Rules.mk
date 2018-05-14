.PHONY: rplidar rplidar_clean

rplidar: common april_lcmtypes

rplidar:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/rplidar -f Build.mk

rplidar_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/rplidar -f Build.mk clean

all: rplidar

clean: rplidar_clean
