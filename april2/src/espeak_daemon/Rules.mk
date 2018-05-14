.PHONY: espeak espeak_clean

espeak: httpd common

espeak:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/espeak_daemon -f Build.mk

espeak_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/espeak_daemon -f Build.mk clean

all: espeak

clean: espeak_clean
