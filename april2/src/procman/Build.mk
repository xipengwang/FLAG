TARGETS = $(BIN_PATH)/procman-daemon $(BIN_PATH)/procman-controller $(BIN_PATH)/procman-spy $(BIN_PATH)/procman-spy-web

CFLAGS  := $(CFLAGS_STD)  $(CFLAGS_VX)  $(CFLAGS_COMMON) $(CFLAGS_GTK) $(CFLAGS_LCM)
LDFLAGS := $(LDFLAGS_STD) $(LDFLAGS_VX) $(LDFLAGS_COMMON) -lpthread $(LDFLAGS_GTK) $(LDFLAGS_LCM)
DEPS  	:= $(DEPS_STD)    $(DEPS_VX)    $(DEP $(DEPS_COMMON) $(DEPS_GTK) $(DEPS_LCM)

include $(BUILD_COMMON)


all: $(TARGETS)
	@/bin/true

$(BIN_PATH)/procman-daemon: procman_core.o procman_daemon.o $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

$(BIN_PATH)/procman-controller: procman_core.o procman_controller.o $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

$(BIN_PATH)/procman-spy: procman_core.o procman_spy.o $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

$(BIN_PATH)/procman-spy-web: procman_core.o procman_spy_web.o $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

clean:
	@rm -rf *.o $(TARGETS)
