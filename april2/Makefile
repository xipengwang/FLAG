# CFLAGS_*: The usual.
# LDFLAGS_*: Should include whatever arguments are needed to link, including flags, libraries, etc.
# DEPS_*: Should include whatever targets should trigger a re-link. (Files only)

export APRIL_PATH = $(CURDIR)
export LIB_PATH = $(CURDIR)/lib
export BIN_PATH = $(CURDIR)/bin
export BUILD_COMMON = $(CURDIR)/Build.common

.SUFFIXES:	.c .o

export CFLAGS_STD := -std=gnu99 -g -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE \
		-D_REENTRANT -Wall -Wno-unused-parameter -Wno-unused-variable \
		-Wno-format-zero-length -fPIC -Wredundant-decls -Werror
export LDFLAGS_STD := -lm -lpthread
export DEPS_STD :=

export CFLAGS_LCM := `pkg-config --cflags lcm`
export LDFLAGS_LCM := `pkg-config --libs lcm`
export DEPS_LCM :=

export CFLAGS_GTK := `pkg-config --cflags gtk+-2.0`
export LDFLAGS_GTK := `pkg-config --libs gtk+-2.0`
export DEPS_GTK :=

MAKE := $(MAKE) --no-print-directory

all:

clean:
	@rm -f $(BIN_PATH)/* $(LIB_PATH)/*

test:

.PHONY: doc
doc:
	@doxygen doc/Doxyfile

doc_clean:
	@rm -rf doc/*~ doc/html/ doc/latex/

# This begins the recursive decent crawl over all the Rules.mk files.
# Add additional roots here as necessary.
include $(APRIL_PATH)/Rules.mk
