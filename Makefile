.PHONY: all rtl rtl-clean sim sim-run sim-trace sim-clean sw clean

RTL_MODULES := $(patsubst rtl/%/,%,$(wildcard rtl/*/))

# -------------------------------------------------------
# Per-module rule generator
# $(1) = module name (e.g. uart)
# -------------------------------------------------------
define rtl_module_rules
.PHONY: rtl-$(1) rtl-$(1)-run rtl-$(1)-test rtl-$(1)-clean
rtl-$(1):
	@$$(MAKE) -C rtl/$(1) -f $(1).mk
rtl-$(1)-run:
	@$$(MAKE) -C rtl/$(1) -f $(1).mk run name=$$(name)
rtl-$(1)-test:
	@$$(MAKE) -C rtl/$(1) -f $(1).mk test name=$$(name)
rtl-$(1)-clean:
	@$$(MAKE) -C rtl/$(1) -f $(1).mk clean
endef
$(foreach mod,$(RTL_MODULES),$(eval $(call rtl_module_rules,$(mod))))

# -------------------------------------------------------
# Aggregate RTL targets
# -------------------------------------------------------
rtl: $(addprefix rtl-,$(RTL_MODULES))
rtl-clean: $(addsuffix -clean,$(addprefix rtl-,$(RTL_MODULES)))

# -------------------------------------------------------
# Sim (Verilator co-simulation, delegated to sim/sim.mk)
# -------------------------------------------------------
sim:
	@$(MAKE) -C sim -f sim.mk
sim-run:
	@$(MAKE) -C sim -f sim.mk run
sim-trace:
	@$(MAKE) -C sim -f sim.mk trace
sim-clean:
	@$(MAKE) -C sim -f sim.mk clean

# -------------------------------------------------------
# SW stub
# -------------------------------------------------------
sw:
	@echo "sw: not yet implemented"

# -------------------------------------------------------
# Global
# -------------------------------------------------------
all: rtl
clean: rtl-clean sim-clean
	@echo "All clean."
