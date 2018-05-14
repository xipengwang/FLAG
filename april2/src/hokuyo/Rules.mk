.PHONY: hokuyo hokuyo_clean

hokuyo: httpd common april_lcmtypes vx

hokuyo:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/hokuyo -f Build.mk

hokuyo_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/hokuyo -f Build.mk clean

all: hokuyo

clean: hokuyo_clean
