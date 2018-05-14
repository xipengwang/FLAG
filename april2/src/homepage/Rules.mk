.PHONY: homepage homepage_clean

homepage: common april_lcmtypes httpd

homepage:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/homepage -f Build.mk

homepage_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/homepage -f Build.mk clean

all: homepage

clean: homepage_clean
