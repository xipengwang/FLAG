.PHONY: ublox ublox_clean

ublox: common april_lcmtypes

ublox:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/ublox -f Build.mk

ublox_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/ublox -f Build.mk clean

all: ublox

clean: ublox_clean
