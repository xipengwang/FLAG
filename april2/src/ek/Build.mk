SRCS = $(shell ls *.c)
OBJS = $(SRCS:%.c=%.o)

CFLAGS := $(CFLAGS_STD) $(CFLAGS_COMMON)

include $(BUILD_COMMON)


all: $(LIB_PATH)/libek.a
	@true

$(LIB_PATH)/libek.a: $(OBJS)
	@$(AR) rc $@ $^

clean:
	@rm -rf *.o $(LIB_PATH)/libek.a
