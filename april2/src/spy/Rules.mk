.PHONY: spy spy_clean

spy: common april_lcmtypes httpd

spy:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/spy -f Build.mk

spy_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/spy -f Build.mk clean

all: spy

clean: spy_clean
