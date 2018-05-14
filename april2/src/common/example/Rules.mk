.PHONY: common_example common_example_clean

common_example: common

common_example:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/common/example -f Build.mk

common_example_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/common/example -f Build.mk clean

all: common_example

clean: common_example_clean
