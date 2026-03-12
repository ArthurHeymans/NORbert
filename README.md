# NORbert

A friendly FPGA that pretends to be your NOR flash.

NORbert uses a [Sipeed Tang Primer 25K](https://wiki.sipeed.com/hardware/en/tang/tang-primer-25k/primer-25k.html) to emulate a SPI NOR flash chip, backed by 64MB of SDRAM. Load a firmware image over UART, and NORbert will serve it to your target system as if it were a real flash chip -- supporting single, dual, and quad SPI read modes with pipelined SDRAM prefetch for real-time streaming.

## Features

- **SPI NOR flash emulation** with full command support: read, fast read, page program, sector/block/chip erase, JEDEC ID, status registers, SFDP
- **Configurable chip identity** at runtime -- load any chip definition from [rflasher](https://github.com/benpye/rflasher)'s RON database to set JEDEC ID, size, and SFDP parameters (defaults to Winbond W25Q64FV)
- **Multi-I/O modes**: 1-1-1, 1-1-2, 1-2-2, 1-1-4, and 1-4-4 SPI read modes
- **64MB backing store** using two SDRAM chips with bit-interleaved storage for zero-overhead serial output
- **Pipelined prefetch**: SDRAM reads are issued during SPI address/dummy phases so data is ready on the first clock edge
- **2 Mbaud UART** interface for loading and dumping images from a host PC

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

# Configure chip identity (uses rflasher chip database)
spi-flash-tool configure W25Q128JV --chips-dir ~/src/rflasher/chips/vendors

# List available serial ports
spi-flash-tool ports
```

The `configure` command loads a chip definition from [rflasher](https://github.com/benpye/rflasher)'s RON database and sends the JEDEC ID, size, and a generated SFDP table to the FPGA. The chip name is matched by substring, so `W25Q128` is enough if it's unambiguous. Without configuration, NORbert defaults to Winbond W25Q64FV.

Use `-p /dev/ttyUSBx` if your device isn't on the default `/dev/ttyUSB0`.

## SPI read performance

Sustained SPI clock limits are set by the SDRAM prefetch pipeline in `spi_trx.v`.
Each 8-byte SDRAM burst takes ~14 system clock cycles (132 MHz) from activate to
data valid. The maximum SPI clock is determined by how many SPI clocks the pipeline
has between triggering the next SDRAM read and needing the data (`fresh_read`):

**f_spi_max = 132 MHz x N_spi / 14**

### Sustained streaming (continuous sequential read)

| Command | Mode | Bits/clk | N_spi | Max SPI clock | Throughput |
|---------|------|----------|-------|---------------|------------|
| 0x03 Read | 1-1-1 | 1 | 4 | ~37 MHz | ~4.7 MB/s |
| 0x0B Fast Read | 1-1-1 | 1 | 4 | ~37 MHz | ~4.7 MB/s |
| 0x3B Dual Output | 1-1-2 | 2 | 8 | ~75 MHz | ~18.9 MB/s |
| 0xBB Dual I/O | 1-2-2 | 2 | 8 | ~75 MHz | ~18.9 MB/s |
| 0x6B Quad Output | 1-1-4 | 4 | 4 | ~37 MHz | ~18.9 MB/s |
| 0xEB Quad I/O | 1-4-4 | 4 | 4 | ~37 MHz | ~18.9 MB/s |

The single-width data pipeline (STA_READ) triggers the next SDRAM burst during byte
7 of the current burst (4 SPI clocks of headroom). The dual pipeline (STA_READ_DUAL)
triggers at byte 6 (8 SPI clocks), while quad (STA_READ_QUAD) also triggers at byte
6 but with only 4 SPI clocks due to 2 clocks/byte. This means dual and quad modes
hit the same ~19 MB/s throughput ceiling, but dual can clock 2x faster.

### First-burst limits

The initial SDRAM read is pipelined during the address/dummy phase. Commands with
dummy clocks get more headroom for the first burst:

| Command | N_spi (initial) | Max SPI (initial) |
|---------|-----------------|-------------------|
| 0x03 / 0x13 (no dummy) | 4 | ~37 MHz |
| 0x0B / 0x3B / 0x6B (8 dummy clocks) | 12 | ~113 MHz |
| 0xBB (dual addr + 4 mode/dummy) | 5 | ~47 MHz |
| 0xEB (quad addr + 6 mode/dummy) | 6 | ~56 MHz |

After the first burst, sustained rates apply. Commands with single-bit address
phases (0x0B, 0x3B, 0x6B) benefit most from dummy clocks. The multi-bit address
commands (0xBB, 0xEB) use fewer SPI clocks for the address, leaving less headroom.

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
  src/chip.rs  Chip definition loading from rflasher RON database
  src/sfdp.rs  SFDP/BFPT table generation from chip definitions
```
