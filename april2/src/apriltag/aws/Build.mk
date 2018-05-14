CFLAGS := $(CFLAGS_STD) $(CFLAGS_COMMON) $(CFLAGS_APRILTAG) -O4 -Wno-unused-func
LDFLAGS := $(LDFLAGS_STD) $(LDFLAGS_COMMON) $(LDFLAGS_APRILTAG)
DEPS := $(DEPS_STD) $(DEPS_COMMON) $(DEPS_APRILTAG)

include $(BUILD_COMMON)

all: apriltag_aws
	@true

apriltag_aws: apriltag_aws.o $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

clean:
	@rm -rf *.o apriltag_aws
