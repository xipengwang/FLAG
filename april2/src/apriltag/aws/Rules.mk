.PHONY: apriltag_aws apriltag_aws_clean

apriltag_aws: apriltag common

apriltag_aws:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/apriltag/aws -f Build.mk

apriltag_aws_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/apriltag/aws -f Build.mk clean

all: apriltag_aws

clean: apriltag_aws_clean
