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
	src/sdram.v \
	src/glue.v \
	src/uart.v \
	src/ft245.v \
	src/fifo.v \
	src/util.v \
	src/pll.v

# Constraints
CST_FILE = tangprimer25k.cst

# Gowin IDE output
BITSTREAM = impl/pnr/spi_flash.fs

.PHONY: all build lint prog flash clean tool help

all: build

# Full build: synthesis + PnR via Gowin CLI
build: $(BITSTREAM)

$(BITSTREAM): $(VERILOG_FILES) $(CST_FILE) build.tcl
	gw_sh build.tcl

# Lint with yosys (quick syntax/logic check)
lint:
	yosys -p "read_verilog $(VERILOG_FILES); synth_gowin -top top -noflatten"

# Program the device (volatile - lost on power cycle)
prog: $(BITSTREAM)
	openFPGALoader -b tangprimer25k $<

# Program to flash (persistent)
flash: $(BITSTREAM)
	openFPGALoader -b tangprimer25k -f $<

clean:
	rm -rf impl

# Build the spi-flash-tool
tool:
	cargo build --release --manifest-path tool/Cargo.toml
	@echo "Tool built: tool/target/release/spi-flash-tool"

help:
	@echo "Tang Primer 25K SPI Flash Emulator"
	@echo ""
	@echo "  make build   - Synthesize + PnR (default, requires gw_sh)"
	@echo "  make prog    - Program FPGA (volatile)"
	@echo "  make flash   - Program to flash (persistent)"
	@echo "  make lint    - Lint Verilog with yosys"
	@echo "  make tool    - Build spi-flash-tool"
	@echo "  make clean   - Clean build artifacts"
