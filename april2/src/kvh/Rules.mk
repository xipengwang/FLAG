.PHONY: kvh kvh_clean

kvh: april_lcmtypes common

kvh:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/kvh -f Build.mk

kvh_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/kvh -f Build.mk clean

all: kvh

clean: kvh_clean
