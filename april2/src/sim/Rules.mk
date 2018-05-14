export CFLAGS_SIM = $(CFLAGS_COMMON)
export LDFLAGS_SIM = $(LIB_PATH)/libsim.a
export DEPS_SIM = $(LIB_PATH)/libsim.a

.phony: sim sim_clean

sim: common vx april_lcmtypes graph

sim:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/sim -f Build.mk

sim_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/sim -f Build.mk clean

all: sim


clean: sim_clean
