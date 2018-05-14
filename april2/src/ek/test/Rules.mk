.phony: ek_test ek_test_clean

ek_test: common ek

ek_test:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/ek/test -f Build.mk

ek_test_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/ek/test -f Build.mk clean

all: ek_test

clean: ek_test_clean
