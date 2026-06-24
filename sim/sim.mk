# sim.mk -- Verilator co-simulation build for the Modbus RTU card.
#
# Layout:
#   sim/src/sim_main.cpp   harness source
#   sim/sim.mk             this file
#   sim/build/             obj_dir, executable, sim.vcd  (all generated)
#
# Standalone:  cd sim && make -f sim.mk            (build)
#              cd sim && make -f sim.mk run        (build + launch)
#              cd sim && make -f sim.mk trace      (build with VCD)
#              cd sim && make -f sim.mk clean
# From root:   make sim          (see root Makefile hook below)
#
# All relative paths assume CWD = sim/ (true under both `cd sim` and
# `make -C sim -f sim.mk`).

RTL       := ../rtl
TOP       := modbus_top
TOP_SRC   := $(RTL)/modbus_top/src/modbus_top.v
MAIN      := src/sim_main.cpp

BUILD     := build
OBJDIR    := $(BUILD)/obj_dir
EXE       := $(BUILD)/modbus_cosim

# openpty lives in libutil on macOS/BSD; in libc on Linux (no flag needed)
UNAME_S   := $(shell uname -s)
ifneq ($(UNAME_S),Linux)
  LDFLAGS := -lutil
endif

# -y search paths: every module's src/ that modbus_top pulls in
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
             --Mdir $(OBJDIR) \
             -Wno-fatal \
             $(INCDIRS)

# default: build, no waveform
all: $(BUILD)
	verilator $(VFLAGS) -LDFLAGS "$(LDFLAGS)" $(TOP_SRC) $(MAIN)
	@cp $(OBJDIR)/V$(TOP) $(EXE)
	@echo "built $(EXE)"

# build with tracing (defines VM_TRACE in Verilator-land, dumps build/sim.vcd)
trace: $(BUILD)
	verilator $(VFLAGS) --trace -LDFLAGS "$(LDFLAGS)" $(TOP_SRC) $(MAIN)
	@cp $(OBJDIR)/V$(TOP) $(EXE)
	@echo "built $(EXE) (tracing on -> $(BUILD)/sim.vcd)"

run: all
	./$(EXE)

$(BUILD):
	@mkdir -p $(BUILD)

clean:
	rm -rf $(OBJDIR) $(EXE) $(BUILD)/sim.vcd

.PHONY: all trace run clean
