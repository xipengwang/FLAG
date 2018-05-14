.PHONY: vx_test vx_plot_test

vx_test: vx

vx_plot_test: vx utest

jsvx_test: vx

vx_test:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/vx/test -f Build.mk

vx_test_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/vx/test -f Build.mk clean

vx_plot_test:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/vx/test -f Build.mk test

vx_plot_test_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/vx/test -f Build.mk clean

all:

test: vx_plot_test

clean: vx_test_clean
