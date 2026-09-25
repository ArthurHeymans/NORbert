# Makefile for Tang Primer 25K SPI Flash Emulator
# Uses Gowin IDE (Education Edition) via gw_sh for CLI synthesis.
# GW5A is NOT supported by open-source tools (apicula/nextpnr-gowin).

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

# Open-source lint tools use Yosys' GW5A primitive declarations so the
# generated PLL wrapper can be elaborated without the proprietary simulator.
GOWIN_CELLS = $(shell yosys-config --datdir)/gowin/cells_xtra_gw5a.v

.PHONY: all build lint lint-yosys lint-verilator test prog flash clean tool webui webui-serve ftdi-setup help

all: build

# Full build: synthesis + PnR via Gowin CLI
build: $(BITSTREAM)

$(BITSTREAM): $(VERILOG_FILES) $(VERILOG_HEADERS) $(CST_FILE) tangprimer25k.sdc build.tcl
	gw_sh build.tcl

# Open-source syntax, elaboration, and synthesis checks.
lint: lint-yosys lint-verilator

lint-yosys:
	yosys -Q -q \
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
	for test in spi_flash sdram_controller toctou quad_fast fifo logger uart ft245; do \
		echo "Testing $$test"; \
		verilator --binary --timing -j 2 --top-module $${test}_tb -Isrc \
			-Wno-CASEINCOMPLETE -Wno-PINMISSING -Wno-TIMESCALEMOD \
			--Mdir "$$build/$$test" tests/$${test}_tb.sv tests/pll_stub.v \
			$(filter-out src/pll.v,$(VERILOG_FILES)) >"$$build/$$test.log" 2>&1 \
			|| { cat "$$build/$$test.log"; exit 1; }; \
		"$$build/$$test/V$${test}_tb"; \
	done

# Program the device (volatile - lost on power cycle)
prog: $(BITSTREAM)
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
	@echo "  make prog    - Program FPGA (volatile)"
	@echo "  make flash   - Program to flash (persistent)"
	@echo "  make lint    - Check Verilog with Yosys and Verilator"
	@echo "  make test    - Simulate SPI, SDRAM, TOCTOU, FIFO, logger, UART and FT245"
	@echo "  make tool    - Build spi-flash-tool (ftdi-nusb backend, default)"
	@echo "  make webui  - Build the WebUSB/Web Serial browser UI"
	@echo "  make webui-serve - Build and serve the UI at http://localhost:8081"
	@echo "  make ftdi-setup - Program FT2232H EEPROM for 245 FIFO"
	@echo "  make clean   - Clean build artifacts"
