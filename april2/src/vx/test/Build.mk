SRCS = $(shell ls *.c)
OBJS = $(SRCS:%.c=%.o)
TARGETS = $(OBJS:%.o=$(TEST_PATH)/%)

CFLAGS := $(CFLAGS_STD) $(CFLAGS_VX) $(CFLAGS_UTEST)
LDFLAGS := $(LDFLAGS_STD) $(LDFLAGS_VX) $(LDFLAGS_UTEST)
DEPS  := $(DEPS_STD) $(DEPS_VX) $(DEPS_UTEST)

include $(BUILD_COMMON)

$(TEST_PATH)/%: %.o $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

all: $(TARGETS)
	@/bin/true

test: $(TARGETS) $(OBJS)

clean:
	@rm -rf *.o $(OBJS) $(TARGETS)
