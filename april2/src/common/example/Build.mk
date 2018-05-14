CFLAGS := $(CFLAGS_STD) $(CFLAGS_COMMON)
LDFLAGS := $(LDFLAGS_STD) $(LDFLAGS_COMMON)
DEPS := $(DEPS_STD) $(DEPS_COMMON)

TARGET = dynamixel_util json_parse \
		$(BIN_PATH)/lcm_bridge_send $(BIN_PATH)/lcm_bridge_recv \
		udpr_sender udpr_receiver \
		$(BIN_PATH)/lcm-tcp-server $(BIN_PATH)/lcm-tcp-client \
		pjpeg_demo pam_merge \
		$(BIN_PATH)/lcm-rename

include $(BUILD_COMMON)

all: $(TARGET)
	@true

pam_merge: pam_merge.o $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

pjpeg_demo: pjpeg_demo.o $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

udpr_sender: udpr_sender.o $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

udpr_receiver: udpr_receiver.o $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

dynamixel_util: dynamixel_util.o $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

json_parse: json_parse.o $(DEPS)
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_PATH)/lcm_bridge_send: lcm_bridge_send.o $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

$(BIN_PATH)/lcm_bridge_recv: lcm_bridge_recv.o $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

$(BIN_PATH)/lcm-tcp-server: lcm_tcp_server.o $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

$(BIN_PATH)/lcm-tcp-client: lcm_tcp_client.o $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

$(BIN_PATH)/lcm-rename: lcm_rename.o $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)


clean:
	@rm -rf *.o $(TARGET)
