.PHONY: procman procman_clean

procman: vx common april_lcmtypes

procman:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/procman -f Build.mk

procman_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/procman -f Build.mk clean

all: procman

clean: procman_clean
