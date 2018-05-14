export CFLAGS_HTTPD =
export LDFLAGS_HTTPD = $(LIB_PATH)/libaprilhttpd.a $(LDFLAGS_EK)
export DEPS_HTTPD = $(LIB_PATH)/libaprilhttpd.a $(DEPS_EK)

.phony: httpd httpd_clean

httpd: ek common

httpd:
	@echo $@
	@cd $(APRIL_PATH)/src/httpd && $(MAKE) -f Build.mk && cd ..

httpd_clean:
	@echo $@
	@cd $(APRIL_PATH)/src/httpd && $(MAKE) -f Build.mk clean && cd ..

all: httpd

clean: httpd_clean
