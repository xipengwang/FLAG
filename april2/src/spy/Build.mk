TARGETS = $(BIN_PATH)/lcm-spy-console $(BIN_PATH)/lcm-spy-web

CFLAGS := $(CFLAGS_STD) $(CFLAGS_COMMON) $(CFLAGS_HTTPD) $(CFLAGS_APRIL_LCMTYPES) $(CFLAGS_LCM)
LDFLAGS := $(LDFLAGS_STD) $(LDFLAGS_COMMON) $(LDFLAGS_HTTPD) $(LDFLAGS_APRIL_LCMTYPES) $(LDFLAGS_LCM)
DEPS := $(DEPS_STD) $(DEPS_HTTPD) $(DEPS_APRIL_LCMTYPES) $(DEPS_LCM)

include $(BUILD_COMMON)


all: $(TARGETS)
	@true

$(BIN_PATH)/lcm-spy-console: lcm_spy_console.o lcmgen.o tokenize.o $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

$(BIN_PATH)/lcm-spy-web: lcm_spy_web.o lcmgen.o tokenize.o $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

clean:
	@rm -rf *.o $(TARGETS)
