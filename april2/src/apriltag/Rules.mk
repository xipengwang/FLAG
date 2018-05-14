export CFLAGS_APRILTAG =
export LDFLAGS_APRILTAG = $(LIB_PATH)/libapriltag.a -lm $(LDFLAGS_COMMON) -lpthread
export DEPS_APRILTAG = $(LIB_PATH)/libapriltag.a $(DEPS_COMMON)

.PHONY: apriltag apriltag_clean

apriltag: april_lcmtypes common

apriltag:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/apriltag -f Build.mk

apriltag_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/apriltag -f Build.mk clean

all: apriltag

clean: apriltag_clean

include $(APRIL_PATH)/src/apriltag/aws/Rules.mk
include $(APRIL_PATH)/src/apriltag/test/Rules.mk
