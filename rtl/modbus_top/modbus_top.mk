COMPILER  := iverilog
SIMULATOR := vvp
WAVE      := surfer
SRC_DIR   := src
TB_DIR    := tb
BUILD_DIR := build
VVP_DIR   := $(BUILD_DIR)/vvp
VCD_DIR   := $(BUILD_DIR)/vcd
SELF      := $(lastword $(MAKEFILE_LIST))
DEPS := \
  ../uart/src/uart_rx.v \
  ../uart/src/uart_tx.v \
  ../crc16/src/crc16.v \
  ../frame_detector/src/frame_detector.v \
  ../register_bank/src/register_bank.v \
  ../modbus_slave_fsm/src/modbus_slave_fsm.v \
  ../modbus_master_fsm/src/modbus_master_fsm.v
SOURCES := $(wildcard $(SRC_DIR)/*.v) $(DEPS)
TB_FILES    := $(wildcard $(TB_DIR)/*.v)
TB_STEMS    := $(notdir $(TB_FILES:.v=))
ALL_TARGETS := $(patsubst %,$(VVP_DIR)/%.vvp,$(TB_STEMS))

.PHONY: all clean run directories

ifdef name
  TARGET_STEM    := $(basename $(notdir $(name)))
  DEFAULT_TARGET := $(VVP_DIR)/$(TARGET_STEM).vvp
else
  DEFAULT_TARGET := $(ALL_TARGETS)
endif

all: directories $(DEFAULT_TARGET)

directories:
	@mkdir -p $(VVP_DIR) $(VCD_DIR)

$(VVP_DIR)/%.vvp: $(TB_DIR)/%.v $(SOURCES)
	$(COMPILER) -o $@ $(SOURCES) $<

ifdef name
  RUN_STEM := $(basename $(notdir $(name)))
else ifdef TOP
  RUN_STEM := $(basename $(notdir $(TOP)))
endif

test: directories
ifndef RUN_STEM
	$(error Specify: make test name=<tb_file>)
endif
	@$(MAKE) --no-print-directory -f $(SELF) name=$(RUN_STEM)
	@echo "--- Running $(RUN_STEM) ---"
	@$(SIMULATOR) $(VVP_DIR)/$(RUN_STEM).vvp

run: directories
ifndef RUN_STEM
	$(error Specify: make run name=<tb_file>)
endif
	@$(MAKE) --no-print-directory -f $(SELF) name=$(RUN_STEM)
	@echo "--- Running $(RUN_STEM) ---"
	@$(SIMULATOR) $(VVP_DIR)/$(RUN_STEM).vvp
	@echo "--- Opening waveform ---"
	@$(WAVE) $(VCD_DIR)/$(RUN_STEM).vcd

clean:
	@rm -rf $(BUILD_DIR)
	@echo "modbus_top clean."
