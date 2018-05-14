export CFLAGS_APRIL_LCMTYPES =
export LDFLAGS_APRIL_LCMTYPES = $(LIB_PATH)/libaprillcmtypes.a $(LDFLAGS_LCM)
export DEPS_APRIL_LCMTYPES = $(LIB_PATH)/libaprillcmtypes.a $(DEPS_LCM)

.PHONY: april_lcmtypes april_lcmtypes_clean

april_lcmtypes:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/lcmtypes -f Build.mk

# is this indirection necessary?
april_lcmtypes_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/lcmtypes -f Build.mk clean

all: april_lcmtypes

clean: april_lcmtypes_clean
