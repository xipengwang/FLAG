export CFLAGS_APRIL_VELODYNE =
export LDFLAGS_APRIL_VELODYNE = $(LIB_PATH)/libapril_velodyne.a -lm
export DEPS_APRIL_VELODYNE = $(LIB_PATH)/libapril_velodyne.a

.PHONY: velodyne velodyne_clean

velodyne: common april_lcmtypes vx

velodyne:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/velodyne -f Build.mk

velodyne_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/velodyne -f Build.mk clean

all: velodyne

clean: velodyne_clean
