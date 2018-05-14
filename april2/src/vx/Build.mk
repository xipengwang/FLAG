SRCS = $(shell ls *.c)
OBJS = $(SRCS:%.c=%.o)

CFLAGS := $(CFLAGS_STD) $(CFLAGS_COMMON)
LDFLAGS := $(LDFLAGS_STD) $(LDFLAGS_COMMON)

include $(BUILD_COMMON)


all: $(LIB_PATH)/libvx.a
	@true

$(LIB_PATH)/libvx.a: $(OBJS)
	@$(AR) rc $@ $^

clean:
	@rm -rf *.o $(LIB_PATH)/libvx.a
