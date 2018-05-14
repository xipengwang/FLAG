OBJS = main.o scip2.o

CFLAGS  = $(CFLAGS_STD) $(CFLAGS_COMMON) $(CFLAGS_VX) $(CFLAGS_LCM) $(CFLAGS_APRIL_LCMTYPES)
LDFLAGS = $(LDFLAGS_STD) $(LDFLAGS_COMMON) $(LDFLAGS_VX) $(LDFLAGS_LCM) $(LDFLAGS_APRIL_LCMTYPES) -lpthread
DEPS    = $(DEPS_STD) $(DEPS_COMMON) $(DEPS_VX) $(DEPS_LCM) $(DEPS_APRIL_LCMTYPES)

include $(BUILD_COMMON)


all: $(BIN_PATH)/hokuyo $(BIN_PATH)/hokuyo-viewer
	@true

$(BIN_PATH)/hokuyo:  $(OBJS) $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

$(BIN_PATH)/hokuyo-viewer:  viewer.o $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

clean:
	@rm -rf *.o
