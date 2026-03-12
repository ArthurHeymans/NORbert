# NORbert

A friendly FPGA that pretends to be your NOR flash.

NORbert uses a [Sipeed Tang Primer 25K](https://wiki.sipeed.com/hardware/en/tang/tang-primer-25k/primer-25k.html) to emulate a SPI NOR flash chip, backed by 64MB of SDRAM. Load a firmware image over UART, and NORbert will serve it to your target system as if it were a real flash chip -- supporting single, dual, and quad SPI read modes with pipelined SDRAM prefetch for real-time streaming.

## Features

- **SPI NOR flash emulation** with full command support: read, fast read, page program, sector/block/chip erase, JEDEC ID, status registers
- **Multi-I/O modes**: 1-1-1, 1-1-2, 1-2-2, 1-1-4, and 1-4-4 SPI read modes
- **64MB backing store** using two SDRAM chips with bit-interleaved storage for zero-overhead serial output
- **Pipelined prefetch**: SDRAM reads are issued during SPI address/dummy phases so data is ready on the first clock edge
- **2 Mbaud UART** interface for loading and dumping images from a host PC
- **Two emulated flash identities** (compile-time selectable):
  - Winbond W25Q64FV (8MB, 3-byte addressing)
  - Micron N25Q256A (32MB, 3/4-byte addressing)

## Hardware

- [Sipeed Tang Primer 25K](https://wiki.sipeed.com/hardware/en/tang/tang-primer-25k/primer-25k.html) (Gowin GW5A-LV25MG121)
- Tang Primer 25K Dock ext-board (provides SDRAM and USB-UART)
- SPI signals exposed on PMOD connector J5

## Building

### Prerequisites

If you have [Nix](https://nixos.org/) with flakes enabled, just enter the dev shell:

```
nix develop    # or let direnv handle it
```

This provides the Gowin IDE (Education Edition), yosys, openFPGALoader, verilator, and the Rust toolchain.

Without Nix, you'll need:
- [Gowin IDE Education Edition](https://www.gowinsemi.com/en/support/home/) v1.9.11.03 (`gw_sh` on PATH)
- [openFPGALoader](https://github.com/trabucayre/openFPGALoader)
- [yosys](https://github.com/YosysHQ/yosys) (optional, for linting)
- Rust toolchain (for the host tool)

### FPGA bitstream

```
make build                  # synthesize + place & route
make prog                   # program FPGA (volatile, lost on power cycle)
make flash                  # program to flash (persistent)
```

To target the Micron N25Q256A instead of the default Winbond W25Q64FV:

```
make build FLASH_CHIP=1
```

### Host tool

```
make tool
```

This builds `spi-flash-tool` at `tool/target/release/spi-flash-tool`.

## Usage

Load a firmware image into NORbert's SDRAM, then let your target SPI master read it back as if it were a real flash chip.

```
# Check connection
spi-flash-tool version

# Load a firmware image
spi-flash-tool load firmware.bin

# Load with verification
spi-flash-tool load firmware.bin --verify

# Dump contents to a file
spi-flash-tool dump output.bin --length 0x100000

# Read a range (hex dump)
spi-flash-tool read 0x0 0x100

# List available serial ports
spi-flash-tool ports
```

Use `-p /dev/ttyUSBx` if your device isn't on the default `/dev/ttyUSB0`.

## Project structure

```
src/
  top.v        Top-level module, clock/reset, bus wiring
  spi_trx.v    SPI flash transceiver (command decoder + data path)
  sdram.v      Dual-chip SDRAM controller with interleaved storage
  glue.v       UART protocol handler, SPI write engine, LED control
  uart.v       UART TX/RX (2 Mbaud)
  fifo.v       Synchronous FIFO (first-word-fall-through)
  util.v       Clock divider, PWM, synchronizers
  pll.v        PLL: 50MHz -> 120MHz with phase-shifted outputs
tool/
  src/main.rs  Host-side CLI (Rust) for loading/dumping over UART
```
