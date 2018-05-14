SRCS = image_source.c image_source_islog.c image_source_null.c image_source_filedir.c image_source_tcp.c image_convert.c url_parser.c  image_source_pgusb.c image_source_v4l2.c image_source_dc1394.c
OBJS = $(SRCS:%.c=%.o)
TARGET = $(LIB_PATH)/libimagesource.a

CFLAGS := $(CFLAGS_STD) $(CFLAGS_COMMON) $(CFLAGS_GTK) $(CFLAGS_IMAGESOURCE) $(CFLAGS_LCM)
LDFLAGS := $(LDFLAGS_STD) $(LDFLAGS_VX) $(LDFLAGS_APRIL_LCMTYPES) $(LDFLAGS_USB) $(LDFLAGS_PNG) $(LDFLAGS_LCM)
DEPS := $(DEPS_STD) $(DEPS_VX) $(DEPS_APRIL_LCMTYPES) $(DEPS_LCM)

include $(BUILD_COMMON)


all: $(TARGET)
	@true

$(BIN_PATH)/istest: istest.o $(LIB_PATH)/libimagesource.a $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS) $(LDFLAGS_IMAGESOURCE)

$(TARGET): $(OBJS)
	@$(AR) rc $@ $^

clean:
	@rm -rf *.o $(TARGET)
