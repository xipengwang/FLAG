.PHONY: common_test common_test_clean

common_test: common

common_test:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/common/test -f Build.mk

common_test_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/common/test -f Build.mk clean

all: common_test

clean: common_test_clean
