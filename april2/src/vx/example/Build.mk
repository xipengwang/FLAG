CFLAGS := $(CFLAGS_STD) $(CFLAGS_VX) $(CFLAGS_IMAGESOURCE) $(CFLAGS_COMMON)
LDFLAGS := $(LDFLAGS_STD) $(LDFLAGS_VX) $(LDFLAGS_IMAGESOURCE) $(LDFLAGS_COMMON)
DEPS  := $(DEPS_STD) $(DEPS_VX) $(DEPS_IMAGESOURCE) $(DEPS_COMMON)

include $(BUILD_COMMON)


all: mesh_model_test jsvx_isview vx_plot_example jsvx_test
	@true

jsvx_isview: jsvx_isview.o $(DEPS)
	@$(LD) $(CFLAGS) -o $@ $< $(LDFLAGS)

mesh_model_test: mesh_model_test.o $(DEPS)
	@$(LD) $(CFLAGS) -o $@ $< $(LDFLAGS)

vx_plot_example: vx_plot_example.o $(DEPS)
	@$(LD) $(CFLAGS) -Dfoobar -o $@ $< $(LDFLAGS)

jsvx_test: jsvx_test.o $(DEPS)
	@$(LD) $(CFLAGS) -Dfoobar -o $@ $< $(LDFLAGS)

clean:
	@rm -rf *.o
