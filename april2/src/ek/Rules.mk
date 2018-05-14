export CFLAGS_EK = -Dapril_ek_flag
export LDFLAGS_EK = $(LIB_PATH)/libek.a $(LDFLAGS_COMMON)
export DEPS_EK = $(LIB_PATH)/libek.a $(DEPS_COMMON)

.phony: ek ek_clean

ek: common

ek:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/ek -f Build.mk

ek_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/ek -f Build.mk clean

all: ek

clean: ek_clean


include $(APRIL_PATH)/src/ek/test/Rules.mk
