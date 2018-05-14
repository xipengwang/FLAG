SRCS := $(shell ls *.c)
SRCS := $(filter-out apriltag1.c, $(SRCS))
OBJS := $(SRCS:%.c=%.o)

CFLAGS := $(CFLAGS_STD) $(CFLAGS_COMMON)

include $(BUILD_COMMON)


all: $(LIB_PATH)/libapriltag.a
	@true

$(LIB_PATH)/libapriltag.a: $(OBJS)
	@$(AR) rc $@ $^

.PHONY: clean
clean:
	@rm -rf *.o $(LIB_PATH)/libapriltag.a
