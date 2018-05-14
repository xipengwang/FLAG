# libusb
export CFLAGS_USB := `pkg-config --cflags libusb-1.0`
export LDFLAGS_USB := `pkg-config --libs libusb-1.0`
export DEPS_USB    :=

# libpng
export CFLAGS_PNG := `pkg-config --cflags libpng`
export LDFLAGS_PNG := `pkg-config --libs libpng`

include $(APRIL_PATH)/src/Rules.mk
