SRCS = $(shell ls *.c)
OBJS = $(SRCS:%.c=%.o)

# fno-strict-overflow necessary to avoid -O? errors in g2d_convex_hull
CFLAGS := $(CFLAGS_STD) -I..  $(CFLAGS_LCM) -O4 -fno-strict-overflow
LDFLAGS := $(LDFLAGS_STD) $(LDFLAGS_COMMON) $(LDFLAGS_LCM)
DEPS := $(DEPS_STD) $(DEPS_LCM)

include $(BUILD_COMMON)


all: $(LIB_PATH)/libcommon.a
	@true

$(LIB_PATH)/libcommon.a: $(OBJS) $(DEPS)
	@$(AR) rc $@ $^

clean:
	@rm -rf *.o $(LIB_PATH)/libcommon.a
