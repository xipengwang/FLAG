CFLAGS := $(CFLAGS_STD) $(CFLAGS_COMMON) $(CFLAGS_APRILTAG) $(CFLAGS_RAYTRACER)
LDFLAGS := $(LDFLAGS_STD) $(LDFLAGS_COMMON) $(LDFLAGS_APRILTAG) $(LDFLAGS_RAYTRACER)
DEPS := $(DEPS_STD) $(DEPS_COMMON) $(DEPS_APRILTAG) $(DEPS_RAYTRACER)

include $(BUILD_COMMON)

TARGETS = apriltag-eval

all: $(TARGETS)
	@/bin/true

apriltag-eval: apriltag_eval.o $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

apriltag-cpp-test: apriltag_cpp_test.cc
	g++ -o $@ $(CFLAGS) $^ $(LDFLAGS)

clean:
	@rm -rf *.o $(TARGETS)
