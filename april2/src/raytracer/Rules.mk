export CFLAGS_RAYTRACER =
export LDFLAGS_RAYTRACER = $(LIB_PATH)/libraytracer.a $(LDFLAGS_COMMON) -lpthread
export DEPS_RAYTRACER = $(LIB_PATH)/libraytracer.a $(DEPS_COMMON)

.PHONY: raytracer raytracer_clean

raytracer: common

raytracer:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/raytracer -f Build.mk

raytracer_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/raytracer -f Build.mk clean

all: raytracer

clean: raytracer_clean
