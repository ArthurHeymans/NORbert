# shellcheck shell=sh disable=SC2034
# Makefile for Tang Primer 25K SPI Flash Emulator
# Uses Gowin IDE (Education Edition) via gw_sh for CLI synthesis.
# GW5A is NOT supported by open-source tools (apicula/nextpnr-gowin).

# GW5A-LV25MG121NC1/I0 - Tang Primer 25K FPGA (MBGA121N package)
DEVICE=GW5A-LV25MG121NC1/I0
FAMILY=GW5A-25B

# Source files
VERILOG_FILES=\
	src/top.v \
	src/spi_trx.v \
	src/sd_card_1bit.v \
	src/sdram.v \
	src/glue.v \
	src/uart.v \
	src/ft245.v \
	src/fifo.v \
	src/logger.v \
	src/util.v \
	src/pll.v

# Constraints
CST_FILE=tangprimer25k.cst

# Gowin IDE output
BITSTREAM=impl/pnr/spi_flash.fs

.PHONY: all build lint test prog flash clean tool tool-d2xx ftdi-setup help

all: build

# Full build: synthesis + PnR via Gowin CLI
build: $(BITSTREAM)

$(BITSTREAM): $(VERILOG_FILES) $(CST_FILE) build.tcl
	gw_sh build.tcl

# Lint with yosys (quick syntax/logic check)
lint:
	yosys -p "read_verilog $(VERILOG_FILES); synth_gowin -top top -noflatten"

# Simulation: SD card emulator end-to-end against the real SDRAM controller
# plus a minimal SDRAM chip model.  Note: use iverilog; verilator 5.044
# segfaults on the task constructs in sd_card_1bit.v (V3Life bug).
test: tb/sd_card_tb.v src/sd_card_1bit.v src/sdram.v
	iverilog -g2012 -o impl/sd_card_tb tb/sd_card_tb.v src/sd_card_1bit.v src/sdram.v
	vvp impl/sd_card_tb | tee impl/sd_card_tb.log
	grep -q "TEST PASSED" impl/sd_card_tb.log

# Program the device (volatile - lost on power cycle)
prog: $(BITSTREAM)
	openFPGALoader -b tangprimer25k $(BITSTREAM)

# Program to flash (persistent)
flash: $(BITSTREAM)
	openFPGALoader -b tangprimer25k -f $(BITSTREAM)

clean:
	rm -rf impl

# Program FT2232H EEPROM for async 245 FIFO mode on Channel A
ftdi-setup:
	ftdi_eeprom --flash-eeprom ft2232h.conf
	@echo ""
	@echo "EEPROM programmed. Unplug and replug the FT2232H now."

# Build the norbert-tool (default: rs-ftdi backend, pure Rust)
tool:
	cargo build --release --manifest-path tool/Cargo.toml
	@echo "Tool built: tool/target/release/norbert-tool"

# Build with D2XX backend (requires ftdi_sio unbind, proprietary C library)
tool-d2xx:
	cargo build --release --manifest-path tool/Cargo.toml --no-default-features --features d2xx
	@echo "Tool built (D2XX backend): tool/target/release/norbert-tool"

help:
	@echo "Tang Primer 25K SPI Flash Emulator"
	@echo ""
	@echo "  make build   - Synthesize + PnR (default, requires gw_sh)"
	@echo "  make prog    - Program FPGA (volatile)"
	@echo "  make flash   - Program to flash (persistent)"
	@echo "  make lint    - Lint Verilog with yosys"
	@echo "  make test    - Run SD card emulation testbench (iverilog)"
	@echo "  make tool    - Build norbert-tool (rs-ftdi backend, default)"
	@echo "  make tool-d2xx - Build norbert-tool (D2XX backend)"
	@echo "  make ftdi-setup - Program FT2232H EEPROM for 245 FIFO"
	@echo "  make clean   - Clean build artifacts"
