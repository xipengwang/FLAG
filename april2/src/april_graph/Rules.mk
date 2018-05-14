export CFLAGS_GRAPH =
export LDFLAGS_GRAPH = $(LIB_PATH)/libgraph.a $(LDFLAGS_COMMON)
export DEPS_GRAPH = $(LIB_PATH)/libgraph.a $(DEPS_COMMON)

.PHONY: graph graph_clean

graph: common

graph:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/april_graph -f Build.mk

graph_clean:
	@echo $@
	@$(MAKE) -C $(APRIL_PATH)/src/april_graph -f Build.mk clean

all: graph

clean: graph_clean
