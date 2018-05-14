.SUFFIXES:
.SUFFIXES:	.c .o .h

export ROOT_PATH    := $(CURDIR)
export APRIL_PATH   := $(ROOT_PATH)/april2
export BIN_PATH     := $(ROOT_PATH)/bin
export LIB_PATH     := $(ROOT_PATH)/lib
export BUILD_COMMON := $(ROOT_PATH)/Build.common

export CONFIG_DIR   := $(ROOT_PATH)/config
export SRC_PATH     := $(ROOT_PATH)/src
export LCM_PATH     := $(ROOT_PATH)/src/lcmtypes


### Build flags for all targets
#
export CFLAGS_STD := -std=gnu99 -g -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE \
		-D_REENTRANT -Wall -Wno-unused-parameter -Wno-unused-variable \
		-Wno-format-zero-length -pthread -fPIC -Werror \
		-Wredundant-decls \
		-I$(ROOT_PATH)/src -I$(ROOT_PATH)/april2/src
export LDFLAGS_STD := -lpthread
export DEPS_STD    :=

export CFLAGS_LCM  := `pkg-config --cflags lcm`
export LDFLAGS_LCM := `pkg-config --libs lcm`
export DEPS_LCM    :=

export CFLAGS_GTK  := `pkg-config --cflags gtk+-2.0`
export LDFLAGS_GTK := `pkg-config --libs gtk+-2.0`
export DEPS_GTK    :=

MAKE := $(MAKE) --no-print-directory

# Additional targets are added for "all" and "clean" by lower modules
all:

clean:
	@rm -f $(BIN_PATH)/* $(LIB_PATH)/* $(APRIL_PATH)/bin/*

.PHONY: 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19
2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 : all
	$(ROOT_PATH)/scripts/push_to_robots.sh $@

# This begins the recursive decent crawl over all the Rules.mk files.
# Add additional roots here as necessary.
include $(ROOT_PATH)/Rules.mk

SHELL=/bin/bash

.PHONY: no_targets__ list build_list

no_targets__:
list:
	@sh -c "$(MAKE) -p no_targets__ | \
    grep ^[a-zA-Z]:* | \
    grep -v = | \
    sed 's/:.*//p' | \
    sort -u"

build_list:
	@$(eval LIST := `$(MAKE) list | \
	   sed 's/no_targets__\|build_list\|all\|[a-zA-Z0-9_]\+_clean\|make\[[0-9]\]\+\|Makefile//g'`)
	@echo "Targets Found " $(LIST)
	@F_TG=""
	@for TG in $(LIST); do \
	$(MAKE) clean &> /dev/null; echo "Processing Target " $$TG; \
	$(MAKE) $$TG -j &> /dev/null; \
	if [ $$? -ne 0 ]; then echo "Failed Target " $$TG; F_TG="$$F_TG "$$TG; fi ; \
	done; \
	if [ -z "$$F_TG" ]; then echo "All targets passed!" ; \
	else echo "All failed targets " $$F_TG ; fi
