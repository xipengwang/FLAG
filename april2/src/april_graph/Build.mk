SRCS = $(shell ls *.c)
OBJS = $(SRCS:%.c=%.o)

CFLAGS := $(CFLAGS_STD) $(CFLAGS_COMMON)

include $(BUILD_COMMON)


all: $(LIB_PATH)/libgraph.a
	@true

$(LIB_PATH)/libgraph.a: $(OBJS)
	@$(AR) rc $@ $^ 

clean:
	@rm -rf *.o $(LIB_PATH)/libgraph.a
