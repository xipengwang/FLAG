.PHONY: alarm_monitor alarm_monitor_clean

alarm_monitor: common april_lcmtypes

alarm_monitor:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/alarm_monitor -f Build.mk

alarm_monitor_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/alarm_monitor -f Build.mk clean

all: alarm_monitor

clean: alarm_monitor_clean
