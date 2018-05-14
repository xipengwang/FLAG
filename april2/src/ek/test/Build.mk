CFLAGS := $(CFLAGS_STD) $(CFLAGS_COMMON) $(CFLAGS_EK)
LDFLAGS := $(LDFLAGS_STD) $(LDFLAGS_COMMON) $(LDFLAGS_EK)
DEPS := $(DEPS_STD) $(DEPS_COMMON) $(DEPS_EK)

include $(BUILD_COMMON)


.PHONY: clean

all: ek_rsa_test ek_rsa_ssa_test ek_bigint_test ek_hash_test
	@true

ek_bigint_test: ek_bigint_test.o $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

ek_rsa_test: ek_rsa_test.o $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

ek_rsa_ssa_test: ek_rsa_ssa_test.o $(DEPS)
	@$(CC) -o $@ $^ $(LDFLAGS)

ek_hash_test: ek_hash_test.o $(DEPS)
	@$(CC) -o $@ $^ $(LDFLAGS)

clean:
	@rm -rf *.o
