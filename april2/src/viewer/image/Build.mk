TARGETS = $(BIN_PATH)/image-viewer

CFLAGS := $(CFLAGS_STD) $(CFLAGS_VX) $(CFLAGS_APRIL_LCMTYPES)
LDFLAGS := $(LDFLAGS_STD) $(LDFLAGS_VX) $(LDFLAGS_APRIL_LCMTYPES)
DEPS := $(DEPS_STD) $(DEPS_VX) $(DEPS_APRIL_LCMTYPES)

include $(BUILD_COMMON)


all: $(TARGETS)
	@/bin/true

$(BIN_PATH)/image-viewer: main.o $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

clean:
	@rm -rf *.o $(TARGETS)
