export CFLAGS_IMAGESOURCE = `pkg-config --cflags libusb-1.0 libpng`
export LDFLAGS_IMAGESOURCE = $(LIB_PATH)/libimagesource.a `pkg-config --libs libusb-1.0 libpng`
export DEPS_IMAGESOURCE = $(LIB_PATH)/libimagesource.a

.PHONY: imagesource imagesource_clean

imagesource: common vx

imagesource:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/imagesource -f Build.mk

imagesource_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/imagesource -f Build.mk clean

all: imagesource

clean: imagesource_clean
