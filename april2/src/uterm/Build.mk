SRCS = $(shell ls *.c)
OBJS = $(SRCS:%.c=%.o)
TARGET = $(BIN_PATH)/uterm

CFLAGS := $(CFLAGS_STD) $(CFLAGS_COMMON)
LDFLAGS := $(LDFLAGS_STD) $(LDFLAGS_COMMON)
DEPS := $(DEPS_STD) $(DEPS_COMMON)

include $(BUILD_COMMON)


all: $(TARGET)
	@true

$(TARGET): $(OBJS) $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

clean:
	@rm -rf *.o $(TARGET)
