OBJS = main.o
TARGETS = $(BIN_PATH)/espeak-daemon $(BIN_PATH)/espeak-send

CFLAGS = $(CFLAGS_STD) $(CFLAGS_COMMON) $(CFLAGS_HTTPD) $(CFLAGS_LCM)
LDFLAGS = $(LDFLAGS_STD) $(LDFLAGS_COMMON) $(LDFLAGS_HTTPD) -lpthread $(LDFLAGS_LCM) 
DEPS = $(DEPS_STD) $(DEPS_HTTPD) $(DEPS_LCM)

include $(BUILD_COMMON)


all: $(TARGETS)
	@true

$(BIN_PATH)/espeak-daemon:  main.o $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

$(BIN_PATH)/espeak-send:  espeak_send.o $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

clean:
	@rm -rf *.o $(TARGETS)
