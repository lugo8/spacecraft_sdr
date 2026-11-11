TOP     = gps_top
SRCS    = gps_ca_gen.v gps_channel.v gps_correlator.v gps_spi_slave.v gps_top.v
DEVICE  = --45k
PACKAGE = --package CABGA285

# Resource estimation (synthesis only, no PnR)
synth: $(SRCS)
	yosys -p "read_verilog $(SRCS); synth_ecp5 -top $(TOP) -json $(TOP).json" 2>&1 | tee synth.log
	@echo "--- Resource summary ---"
	@grep -A20 "Printing statistics" synth.log

# Full place-and-route (needs nextpnr-ecp5)
pnr: synth
	nextpnr-ecp5 $(DEVICE) $(PACKAGE) --json $(TOP).json --textcfg $(TOP).config 2>&1 | tee pnr.log

# Install tools (Debian/Ubuntu)
install-tools:
	sudo apt-get install -y yosys nextpnr-ecp5

clean:
	rm -f *.json *.config *.log

.PHONY: synth pnr install-tools clean
