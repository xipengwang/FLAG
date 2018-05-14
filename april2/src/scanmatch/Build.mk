SRCS = $(shell ls *.c)
OBJS = $(SRCS:%.c=%.o)
TARGET = $(LIB_PATH)/libscanmatch.a

CFLAGS := $(CFLAGS_STD) $(CFLAGS_COMMON)
LDFLAGS := $(LDFLAGS_STD) $(LDFLAGS_COMMON)
DEPS := $(DEPS_STD) $(DEPS_COMMON)

include $(BUILD_COMMON)


all: $(TARGET)
	@true

$(TARGET): $(OBJS)
	@$(AR) rc $@ $^

clean:
	@rm -rf *.o $(TARGET)
