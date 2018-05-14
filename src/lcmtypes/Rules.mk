export CFLAGS_LCMTYPES  = $(CFLAGS_APRIL_LCMTYPES)
export LDFLAGS_LCMTYPES = $(LIB_PATH)/libmagiclcmtypes.a $(LDFLAGS_APRIL_LCMTYPES)
export DEPS_LCMTYPES = $(LIB_PATH)/libmagiclcmtypes.a $(DEPS_APRIL_LCMTYPES)

.PHONY: lcmtypes lcmtypes_clean

lcmtypes: april_lcmtypes

lcmtypes:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/lcmtypes -f Build.mk

lcmtypes_clean:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/lcmtypes -f Build.mk clean

all: lcmtypes

clean: lcmtypes_clean
