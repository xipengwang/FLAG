export CFLAGS_COMMON = -I$(APRIL_PATH)/src/
export LDFLAGS_COMMON = $(LIB_PATH)/libcommon.a -lm -lz -lpthread $(LDFLAGS_APRIL_LCMTYPES)
export DEPS_COMMON = $(LIB_PATH)/libcommon.a $(DEPS_APRIL_LCMTYPES)

.PHONY: common common_clean

common: april_lcmtypes

common:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/common -f Build.mk

common_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/common -f Build.mk clean

all: common

clean: common_clean

include $(APRIL_PATH)/src/common/example/Rules.mk
include $(APRIL_PATH)/src/common/test/Rules.mk
