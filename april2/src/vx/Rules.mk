export CFLAGS_VX = $(CFLAGS_COMMON) $(CFLAGS_HTTPD)
export LDFLAGS_VX = $(LIB_PATH)/libvx.a $(LDFLAGS_HTTPD) -lpthread
export DEPS_VX = $(LIB_PATH)/libvx.a $(DEPS_HTTPD)

.PHONY: vx vx_clean

vx: april_lcmtypes common httpd

vx:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/vx -f Build.mk

vx_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/vx -f Build.mk clean

all: vx

clean: vx_clean

include $(APRIL_PATH)/src/vx/example/Rules.mk
include $(APRIL_PATH)/src/vx/test/Rules.mk
