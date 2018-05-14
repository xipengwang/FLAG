export CFLAGS_SCANMATCH =
export LDFLAGS_SCANMATCH = $(LIB_PATH)/libscanmatch.a $(LDFLAGS_COMMON)
export DEPS_SCANMATCH = $(LIB_PATH)/libscanmatch.a $(DEPS_COMMON)

.PHONY: scanmatch scanmatch_clean

scanmatch: common

scanmatch:
	@echo $@
	@cd $(APRIL_PATH)/src/scanmatch && $(MAKE) -f Build.mk && cd ..

scanmatch_clean:
	@echo $@
	@cd $(APRIL_PATH)/src/scanmatch && $(MAKE) -f Build.mk clean && cd ..

all: scanmatch

clean: scanmatch_clean
