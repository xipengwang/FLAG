.PHONY: apriltag_test apriltag_test_clean

apriltag_test: apriltag raytracer

apriltag_test:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/apriltag/test -f Build.mk

apriltag_test_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/apriltag/test -f Build.mk clean

all: apriltag_test

clean: apriltag_test_clean
