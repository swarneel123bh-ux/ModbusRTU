# sim.mk
#
# Two harnesses, two executables (separate obj dirs so they don't collide):
#   src/cosim_main.cpp  -> build/modbus_cosim   (two-DUT byte-interface loopback)
#   src/serial_main.cpp -> build/modbus_serial  (one DUT, serial line on a PTY)
#
# Layout:
#   sim/src/*.cpp     harness sources
#   sim/sim.mk        this file
#   sim/build/        obj dirs, executables, *.vcd  (all generated)
#
# Standalone (CWD = sim/):
#   make -f sim.mk cosim         make -f sim.mk cosim-run     make -f sim.mk cosim-trace
#   make -f sim.mk serial        make -f sim.mk serial-run    make -f sim.mk serial-trace
#   make -f sim.mk simtb         make -f sim.mk simtb-run     make -f sim.mk simtb-trace
#   make -f sim.mk clean
#
# sim_tb build (the card): set SIM_TB_V to sim_tb.v's path if not src/sim_tb.v,
#   e.g.  make -f sim.mk simtb SIM_TB_V=../modbus_tb/sim_tb.v

RTL       := ../rtl
TOP       := modbus_top
TOP_SRC   := $(RTL)/modbus_top/src/modbus_top.v
BUILD     := build

# openpty lives in libutil on macOS/BSD; in libc on Linux (no flag needed)
UNAME_S   := $(shell uname -s)
ifneq ($(UNAME_S),Linux)
  LDFLAGS := -lutil
endif

INCDIRS   := \
  -y $(RTL)/uart/src \
  -y $(RTL)/crc16/src \
  -y $(RTL)/frame_detector/src \
  -y $(RTL)/register_bank/src \
  -y $(RTL)/modbus_slave_fsm/src \
  -y $(RTL)/modbus_master_fsm/src \
  -y $(RTL)/modbus_top/src

VFLAGS    := --cc --exe --build -j 0 \
             --top-module $(TOP) \
             -Wno-fatal \
             $(INCDIRS)

# sim_tb is its own Verilog top (master + slave + internal bus). Point this
# at wherever sim_tb.v lives; modbus_top + submodules resolve via INCDIRS.
SIM_TB_V  ?= src/sim_tb.v
VFLAGS_TB := --cc --exe --build -j 0 \
             --top-module sim_tb \
             -Wno-fatal \
             -y src \
             $(INCDIRS)

# ----- cosim (existing two-DUT loopback) -----
cosim: $(BUILD)
	verilator $(VFLAGS) --Mdir $(BUILD)/obj_cosim -LDFLAGS "$(LDFLAGS)" $(TOP_SRC) src/cosim_main.cpp
	@cp $(BUILD)/obj_cosim/V$(TOP) $(BUILD)/modbus_cosim
	@echo "built $(BUILD)/modbus_cosim"

cosim-trace: $(BUILD)
	verilator $(VFLAGS) --trace --Mdir $(BUILD)/obj_cosim -LDFLAGS "$(LDFLAGS)" $(TOP_SRC) src/cosim_main.cpp
	@cp $(BUILD)/obj_cosim/V$(TOP) $(BUILD)/modbus_cosim
	@echo "built $(BUILD)/modbus_cosim (trace -> $(BUILD)/sim.vcd)"

cosim-run: cosim
	./$(BUILD)/modbus_cosim

# ----- serial (new single-DUT PTY bridge) -----
serial: $(BUILD)
	verilator $(VFLAGS) --Mdir $(BUILD)/obj_serial -LDFLAGS "$(LDFLAGS)" $(TOP_SRC) src/serial_main.cpp
	@cp $(BUILD)/obj_serial/V$(TOP) $(BUILD)/modbus_serial
	@echo "built $(BUILD)/modbus_serial"

serial-trace: $(BUILD)
	verilator $(VFLAGS) --trace --Mdir $(BUILD)/obj_serial -LDFLAGS "$(LDFLAGS)" $(TOP_SRC) src/serial_main.cpp
	@cp $(BUILD)/obj_serial/V$(TOP) $(BUILD)/modbus_serial
	@echo "built $(BUILD)/modbus_serial (trace -> $(BUILD)/serial.vcd)"

serial-run: serial
	./$(BUILD)/modbus_serial

# ----- sim_tb (the card: sim_tb.v top + harness + pure-C master driver) -----
simtb: $(BUILD)
	verilator $(VFLAGS_TB) --Mdir $(BUILD)/obj_simtb -LDFLAGS "$(LDFLAGS)" \
	  $(SIM_TB_V) src/sim_tb_main.cpp src/master_driver.c
	@cp $(BUILD)/obj_simtb/Vsim_tb $(BUILD)/sim_tb
	@echo "built $(BUILD)/sim_tb"

simtb-trace: $(BUILD)
	verilator $(VFLAGS_TB) --trace --Mdir $(BUILD)/obj_simtb -LDFLAGS "$(LDFLAGS)" \
	  $(SIM_TB_V) src/sim_tb_main.cpp src/master_driver.c
	@cp $(BUILD)/obj_simtb/Vsim_tb $(BUILD)/sim_tb
	@echo "built $(BUILD)/sim_tb (trace -> $(BUILD)/sim_tb.vcd)"

simtb-run: simtb
	./$(BUILD)/sim_tb

$(BUILD):
	@mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)/obj_cosim $(BUILD)/obj_serial $(BUILD)/obj_simtb \
	       $(BUILD)/modbus_cosim $(BUILD)/modbus_serial $(BUILD)/sim_tb \
	       $(BUILD)/sim.vcd $(BUILD)/serial.vcd $(BUILD)/sim_tb.vcd

.PHONY: cosim cosim-trace cosim-run serial serial-trace serial-run \
        simtb simtb-trace simtb-run clean
