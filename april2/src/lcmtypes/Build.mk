APRIL_LCM_DIR   := $(realpath $(CURDIR)/../../lcmtypes)

# Just the list of type names
APRIL_LCM       := $(notdir $(shell ls $(APRIL_LCM_DIR)/*.lcm))

# The list of .c and .h files that we will generate
APRIL_LCM_C     := $(APRIL_LCM:%.lcm=%.c)
APRIL_LCM_H     := $(APRIL_LCM:%.lcm=%.h)
APRIL_LCM_O     := $(APRIL_LCM:%.lcm=%.o)

%.c %.h : $(APRIL_LCM_DIR)/%.lcm
	@lcm-gen -c $< --cinclude lcmtypes/

CFLAGS := $(CFLAGS_STD) -I.. $(CFLAGS_LCM)
LDFLAGS := $(LDFLAGS_STD) $(LDFLAGS_LCM)

include $(BUILD_COMMON)


all: $(APRIL_LCM_C) $(APRIL_LCM_H) $(LIB_PATH)/libaprillcmtypes.a
	@true

$(LIB_PATH)/libaprillcmtypes.a: $(APRIL_LCM_H) $(APRIL_LCM_O)
	@$(AR) rc $@ $(APRIL_LCM_O)

clean:
	@rm -rf *.o *.c *.h $(LIB_PATH)/libaprillcmtypes.a
