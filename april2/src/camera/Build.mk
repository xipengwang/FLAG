SRCS = $(shell ls *.c)
OBJS = $(SRCS:%.c=%.o)
TARGET = $(LIB_PATH)/libcamera.a

CFLAGS := $(CFLAGS_STD) $(CFLAGS_COMMON)

include $(BUILD_COMMON)


all: $(TARGET)
	@true

$(TARGET): $(OBJS)
	@$(AR) rc $@ $^ 

clean:
	@rm -rf *.o $(TARGET)
