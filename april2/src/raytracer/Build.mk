SRCS = $(shell ls *.c)
OBJS = $(SRCS:%.c=%.o)
TARGET = $(LIB_PATH)/libraytracer.a

CFLAGS := $(CFLAGS_STD) $(CFLAGS_COMMON)

include $(BUILD_COMMON)


all: $(TARGET)
	@true

$(LIB_PATH)/libraytracer.a: $(OBJS)
	@$(AR) rc $@ $^

clean:
	@rm -rf *.o $(TARGET)
