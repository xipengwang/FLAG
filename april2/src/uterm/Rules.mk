.PHONY: uterm uterm_clean

uterm: common april_lcmtypes

uterm:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/uterm -f Build.mk

uterm_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/uterm -f Build.mk clean

all: uterm

clean: uterm_clean
