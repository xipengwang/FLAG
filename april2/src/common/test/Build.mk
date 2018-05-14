CFLAGS := $(CFLAGS_STD) $(CFLAGS_COMMON)
LDFLAGS := $(LDFLAGS_STD) $(LDFLAGS_COMMON)
DEPS := $(DEPS_STD) $(DEPS_COMMON)

include $(BUILD_COMMON)

all: matd_svd_test
	@true

matd_svd_test: matd_svd_test.o $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

clean:
	@rm -rf *.o matd_svd_test
