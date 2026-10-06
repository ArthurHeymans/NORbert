# Makefile for Tang Primer 25K SPI Flash Emulator
# Uses Gowin IDE (Education Edition) via gw_sh for CLI synthesis.
# build-oss is an experimental open-source flow (Yosys, nextpnr-himbaechel,
# Apicula). It builds, but its GW5A timing model is not trusted yet.

# GW5A-LV25MG121NC1/I0 - Tang Primer 25K FPGA (MBGA121N package)
DEVICE = GW5A-LV25MG121NC1/I0
FAMILY = GW5A-25B

# Source files
VERILOG_FILES = \
	src/top.v \
	src/spi_trx.v \
	src/spi_prefetch.v \
	src/sdram.v \
	src/glue.v \
	src/uart.v \
	src/ft245.v \
	src/fifo.v \
	src/logger.v \
	src/util.v \
	src/pll.v

# Included by the sources above
VERILOG_HEADERS = \
	src/spi_flash_cmds.vh \
	src/host_protocol.vh

# Constraints
CST_FILE = tangprimer25k.cst

# Gowin IDE output
BITSTREAM = impl/pnr/spi_flash.fs

# Open-source flow output. Apicula's GW5A-25A database covers the Tang
# Primer 25K part.
OSS_DIR = impl/oss
OSS_BITSTREAM = $(OSS_DIR)/spi_flash.fs
OSS_SDC = tangprimer25k.oss.sdc

# Open-source lint tools use Yosys' GW5A primitive declarations so the
# generated PLL wrapper can be elaborated without the proprietary simulator.
GOWIN_CELLS = $(shell yosys-config --datdir)/gowin/cells_xtra_gw5a.v

.PHONY: all build build-oss lint lint-yosys lint-verilator test test-gate prog prog-oss flash clean tool webui webui-serve ftdi-setup help

all: build

# Full build: synthesis + PnR via Gowin CLI
build: $(BITSTREAM)

$(BITSTREAM): $(VERILOG_FILES) $(VERILOG_HEADERS) $(CST_FILE) tangprimer25k.sdc build.tcl
	gw_sh build.tcl

# Open-source build. Keep shadow SRAM disabled (-nolutram) until the
# experimental database and RAM16 support have fresh hardware validation.
# Keep plain LUT4s (-nowidelut) for comparability with earlier experimental
# builds; the fork fixes wide-LUT costing but broader benchmarks remain.
# The dual-purpose pins are released as GPIO like in build.tcl: without
# i2c_as_gpio, IO_sdram_dq[12] on the I2C SDA pin always reads 1.
# Timing failures are reported but not fatal: the GW5A delays are largely
# borrowed from GW2A, so the Gowin build stays the reference.
build-oss: $(OSS_BITSTREAM)

$(OSS_BITSTREAM): $(VERILOG_FILES) $(VERILOG_HEADERS) $(CST_FILE) $(OSS_SDC)
	mkdir -p $(OSS_DIR)
	yosys -q -l $(OSS_DIR)/yosys.log \
		-p "read_verilog -Isrc $(VERILOG_FILES); synth_gowin -family gw5a -nolutram -nowidelut -top top -json $(OSS_DIR)/top.json"
	nextpnr-himbaechel --json $(OSS_DIR)/top.json --write $(OSS_DIR)/pnr.json \
		--device $(DEVICE) --vopt family=GW5A-25A --vopt cst=$(CST_FILE) \
		--vopt i2c_as_gpio --vopt sspi_as_gpio \
		--sdc $(OSS_SDC) --timing-allow-fail -l $(OSS_DIR)/nextpnr.log
	gowin_pack -d GW5A-25A --i2c_as_gpio --sspi_as_gpio --mspi_as_gpio \
		--ready_as_gpio --done_as_gpio --cpu_as_gpio -o $@ $(OSS_DIR)/pnr.json

# Open-source syntax, elaboration, and synthesis checks.
lint: lint-yosys lint-verilator

# The pinned ABC9 backend uses the experimental XAIG writer. Acknowledge
# that feature explicitly; genuine design warnings still fail lint.
lint-yosys:
	yosys -Q -q -x write_xaiger2 \
		-w "define gw1n not used.*" \
		-w "Yosys has only limited support for tri-state logic.*" \
		-e ".*" \
		-p "read_verilog -lib $(GOWIN_CELLS); read_verilog -Isrc $(VERILOG_FILES); synth_gowin -family gw5a -top top -noflatten; check"

# Width warnings are on deliberately: the address arithmetic here is 23-bit
# burst addresses wrapped by a mask, so a silent truncation is the most
# likely class of bug. Where Verilog-2001 has no cast to size a parameter
# expression, the site carries a lint_off with the reason inline.
lint-verilator:
	verilator --lint-only --top-module top -Isrc \
		-Wno-CASEINCOMPLETE -Wno-DEFPARAM -Wno-PINMISSING \
		$(GOWIN_CELLS) $(VERILOG_FILES)

# Behavioral tests use a clock-only PLL stub, not proprietary primitives.
# Keep generated C++ and binaries outside the working tree.
#
# Width warnings are not suppressed here either: the testbenches must see
# the same width discipline as the lint build, so a testbench cannot hide a
# width bug in the RTL it is exercising.
test:
	@set -eu; build=$$(mktemp -d); trap 'rm -rf "$$build"' EXIT; \
	for test in spi_flash sdram_controller refresh write_completion prefetch_health toctou quad_fast fifo logger uart ft245; do \
		echo "Testing $$test"; \
		verilator --binary --timing -j 2 --top-module $${test}_tb -Isrc \
			-Wno-CASEINCOMPLETE -Wno-PINMISSING -Wno-TIMESCALEMOD \
			--Mdir "$$build/$$test" tests/$${test}_tb.sv tests/pll_stub.v \
			$(filter-out src/pll.v,$(VERILOG_FILES)) >"$$build/$$test.log" 2>&1 \
			|| { cat "$$build/$$test.log"; exit 1; }; \
		"$$build/$$test/V$${test}_tb"; \
	done

# The same testbenches against the Yosys GW5A netlist.
test-gate:
	tests/gate.sh

# Program the device (volatile - lost on power cycle)
prog: $(BITSTREAM)
	openFPGALoader -b tangprimer25k $<

prog-oss: $(OSS_BITSTREAM)
	openFPGALoader -b tangprimer25k $<

# Program to flash (persistent)
flash: $(BITSTREAM)
	openFPGALoader -b tangprimer25k -f $<

clean:
	rm -rf impl

# Program FT2232H EEPROM for async 245 FIFO mode on Channel A
ftdi-setup:
	ftdi_eeprom --flash-eeprom ft2232h.conf
	@echo ""
	@echo "EEPROM programmed. Unplug and replug the FT2232H now."

# Build the spi-flash-tool (default: ftdi-nusb backend, pure Rust)
tool:
	cargo build --release --manifest-path tool/Cargo.toml
	@echo "Tool built: tool/target/release/spi-flash-tool"

# Build and serve the browser UI with Trunk.
webui:
	trunk build --release

webui-serve:
	trunk serve

help:
	@echo "Tang Primer 25K SPI Flash Emulator"
	@echo ""
	@echo "  make build   - Synthesize + PnR (default, requires gw_sh)"
	@echo "  make build-oss - Experimental open-source build (yosys/nextpnr/apicula)"
	@echo "  make prog-oss  - Program the open-source bitstream (volatile)"
	@echo "  make prog    - Program FPGA (volatile)"
	@echo "  make flash   - Program to flash (persistent)"
	@echo "  make lint    - Check Verilog with Yosys and Verilator"
	@echo "  make test    - Simulate SPI, SDRAM, TOCTOU, FIFO, logger, UART and FT245"
	@echo "  make test-gate - Run the testbenches on the Yosys GW5A netlist"
	@echo "  make tool    - Build spi-flash-tool (ftdi-nusb backend, default)"
	@echo "  make webui  - Build the WebUSB/Web Serial browser UI"
	@echo "  make webui-serve - Build and serve the UI at http://localhost:8081"
	@echo "  make ftdi-setup - Program FT2232H EEPROM for 245 FIFO"
	@echo "  make clean   - Clean build artifacts"
