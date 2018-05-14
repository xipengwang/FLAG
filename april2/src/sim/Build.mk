include $(BUILD_COMMON)

DEPS = STD VX LCM GRAPH APRIL_LCMTYPES
CFLAGS := $(CFLAGS_STD) $(CFLAGS_VX) $(CFLAGS_LCM) $(CFLAGS_GRAPH) $(CFLAGS_APRIL_LCMTYPES) -O3
LDFLAGS := $(LDFLAGS_STD) $(LDFLAGS_VX) $(LDFLAGS_LCM) $(LDFLAGS_GRAPH) $(LDFLAGS_APRIL_LCMTYPES)
DEPS := $(DEPS_STD) $(DEPS_VX) $(DEPS_LCM) $(DEPS_GRAPH) $(DEPS_APRIL_LCMTYPES)

all: $(BIN_PATH)/sim-anim $(BIN_PATH)/sim-viewer $(BIN_PATH)/sim-editor $(LIB_PATH)/libsim.a
	@true

$(LIB_PATH)/libsim.a: sim.o sim_triray.o
	@$(AR) rc $@ $^

$(BIN_PATH)/sim-anim: anim.o $(DEPS)
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_PATH)/sim-viewer: viewer.o $(DEPS)
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_PATH)/sim-editor: sim_editor.o sim.o sim_triray.o  $(DEPS)
	@$(CC) -o $@ $^ $(LDFLAGS)

clean:
	@rm -rf *.o
