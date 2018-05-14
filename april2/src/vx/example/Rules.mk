.PHONY: vx_example

vx_example: vx imagesource

vx_example:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/vx/example -f Build.mk

vx_example_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/vx/example -f Build.mk clean


all: vx_example

clean: vx_example_clean
