TARGETS = $(BIN_PATH)/velodyne-driver $(BIN_PATH)/velodyne-viewer $(LIB_PATH)/libapril_velodyne.a

CFLAGS := $(CFLAGS_STD) $(CFLAGS_VX) $(CFLAGS_APRIL_LCMTYPES) $(CFLAGS_LCM)
LDFLAGS := $(LDFLAGS_STD) $(LDFLAGS_VX) $(LDFLAGS_APRIL_LCMTYPES) $(LDFLAGS_LCM)
DEPS := $(DEPS_STD) $(DEPS_VX) $(DEPS_APRIL_LCMTYPES) $(DEPS_LCM)

include $(BUILD_COMMON)


all: $(TARGETS)
	@true

$(BIN_PATH)/velodyne-driver: main.o $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

$(BIN_PATH)/velodyne-viewer: viewer.o april_velodyne.o $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

$(LIB_PATH)/libapril_velodyne.a: april_velodyne.o $(DEPS)
	@$(AR) rc $@ $^

clean:
	@rm -rf *.o $(TARGETS)
