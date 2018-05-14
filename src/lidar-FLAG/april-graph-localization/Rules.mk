.PHONY: graph_flag_clean graph_flag

graph_flag:  common vx lcmtypes graph

graph_flag:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/lidar-FLAG/april-graph-localization/ -f Build.mk

graph_flag_clean:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/lidar-FLAG/april-graph-localization/ -f Build.mk clean

all: graph_flag

clean: graph_flag_clean
