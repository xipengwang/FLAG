.PHONY: licensify licensify_clean

licensify: common

licensify:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/licensify -f Build.mk

licensify_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/licensify -f Build.mk clean

all: licensify

clean: licensify_clean
