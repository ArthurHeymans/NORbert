# NORbert

A friendly FPGA that pretends to be your SPI NOR flash.

NORbert uses a [Sipeed Tang Primer 25K](https://wiki.sipeed.com/hardware/en/tang/tang-primer-25k/primer-25k.html) to emulate a SPI NOR flash chip, backed by 64MB of SDRAM. Load a firmware image over UART, and NORbert will serve it to your target system as if it were a real flash chip -- supporting single, dual, and quad SPI read modes with pipelined SDRAM prefetch for real-time streaming.

## Features

- **SPI NOR flash emulation** with full command support: read, fast read, page program, sector/block/chip erase, JEDEC ID, status registers, SFDP
- **Configurable chip identity** at runtime -- load any chip definition from [rflasher](https://github.com/benpye/rflasher)'s RON database to set JEDEC ID, size, and SFDP parameters (defaults to Winbond W25Q64FV)
- **Multi-I/O modes**: 1-1-1, 1-1-2, 1-2-2, 1-1-4, and 1-4-4 SPI read modes
- **64MB backing store** using two SDRAM chips with bit-interleaved storage for zero-overhead serial output
- **Pipelined prefetch**: SDRAM reads are issued during SPI address/dummy phases so data is ready on the first clock edge
- **2 Mbaud UART** interface for loading and dumping images from a host PC
- **FT245 asynchronous FIFO** via FT2232H for faster bulk transfers (requires one-time EEPROM configuration)
- **SPI bus logging**: real-time command/address/byte-count packets drained over either transport via a 512-byte ring FIFO (`monitor` subcommand)
- **TOCTOU traps**: four independent address-match entries that transparently redirect reads to a different SDRAM location on the second access, for exercising verify-then-use flows
- **Target flash #HOLD control**: drive IO3 low to silence a real flash chip sharing the SPI bus

## Hardware

- [Sipeed Tang Primer 25K](https://wiki.sipeed.com/hardware/en/tang/tang-primer-25k/primer-25k.html) (Gowin GW5A-LV25MG121)
- Tang Primer 25K Dock ext-board (provides SDRAM and USB-UART)
- SPI signals exposed on PMOD connector J5
- **Optional:** FT2232H breakout board for FT245 high-speed transport (e.g., CJMCU-FT2232H or any FT2232H module)

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
- FTDI D2XX driver/library (only if using FT245 transport)

### FPGA bitstream

```
make build                  # synthesize + place & route (default Pico FT245 timing)
make build-pico             # force-build the Raspberry Pi Pico timing variant
make build-ft2232h          # force-build the real FT2232H timing variant
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
spi-flash-tool status       # running | stopped

# Load a firmware image (auto stops + starts emulation around the load)
spi-flash-tool load firmware.bin

# Load with verification
spi-flash-tool load firmware.bin --verify

# Dump contents to a file
spi-flash-tool dump output.bin --length 0x100000

# Read a range (hex dump)
spi-flash-tool read 0x0 0x100

# Configure chip identity (uses rflasher chip database)
spi-flash-tool configure W25Q128JV --chips-dir ~/src/rflasher/chips/vendors

# Gate SPI emulation explicitly (the dance above does this automatically)
spi-flash-tool start
spi-flash-tool stop

# List available serial ports
spi-flash-tool ports
```

The `configure` command loads a chip definition from [rflasher](https://github.com/benpye/rflasher)'s RON database and sends the JEDEC ID, size, and a generated SFDP table to the FPGA. The chip name is matched by substring, so `W25Q128` is enough if it's unambiguous. Without configuration, NORbert defaults to Winbond W25Q64FV.

At power-on the FPGA boots in the STOPPED state -- the SPI pins are held in reset and the host can always reach the tool, regardless of what the target board is doing. `load` automatically stops emulation, writes the image, and starts it again. Use `start`/`stop`/`status` for manual control.

Use `-p /dev/ttyUSBx` if your device isn't on the default `/dev/ttyUSB0`.

### SPI bus monitoring

`monitor` streams decoded SPI activity from NORbert in real time. It works over either UART or FT245 and is safe to run while the target is actively reading:

```
spi-flash-tool monitor
```

Example output while flashprog reads a 4 KB region:

```
TXN#   COMMAND            ADDRESS    INFO
------------------------------------------------------------
1      0x9F READ_JEDEC_ID
2      0x05 READ_STATUS
3      0x05 READ_STATUS
4      0xBB DUAL_IO_READ  0x001000
       end: 4097 bytes from 0x001000
```

The monitor also tracks double-reads of the same (opcode, address) pair and flags them as TOCTOU candidates. Press Ctrl+C to stop. The underlying protocol is a poll-based ring-buffer drain (`CMD_LOGPOLL` = 0x3A); packet types are `0xA1` (command), `0xA2` (address), `0xA3` (end + byte count) and `0xA4` (TOCTOU trap fired), with `0xA0` as the per-poll terminator.

### TOCTOU traps

Four independent trap entries redirect matching reads to a different SDRAM location on the second (and subsequent) access. The first matching read is let through unchanged -- it arms the trap. The first 8 bytes of the redirected read come from the original address because the SDRAM prefetch pipeline fires before the trap check completes; bytes 8+ come from the replacement.

```
# Configure: any read in 0x001000-0x001FFF gets redirected to 0x101000-0x101FFF
spi-flash-tool toctou set 1 0x001000 0xFFF000 0x101000
spi-flash-tool toctou arm 1

# First target read of 0x001000 returns the original data.
# Second target read of 0x001000 returns the replacement data.

# Clear the triggered flag so the next read is "first" again:
spi-flash-tool toctou reset 1

# Tear everything down:
spi-flash-tool toctou reset-all
```

Arguments to `toctou set` are `<index 0..3> <start-address> <match-mask> <replace-base>`, all as byte addresses. 1-bits in the mask must match exactly; 0-bits are don't-care. The replacement preserves the "don't-care" bits of the original address.

### Target flash #HOLD

When NORbert shares a SPI bus with a real flash chip, `hold on` drives IO3 low continuously, asserting `#HOLD` on the target flash so it tristates and ignores all commands. `hold off` releases it. Mutually exclusive with quad I/O because IO3 is shared.

### FT245 transport (FT2232H)

For much faster bulk transfers (~5 MB/s vs ~200 KB/s over UART), connect an FT2232H module and use `--ft245`:

```
# List connected FT2232H devices
spi-flash-tool ft-list

# Use FT245 instead of UART for any command
spi-flash-tool --ft245 version
spi-flash-tool --ft245 load firmware.bin --verify
spi-flash-tool --ft245 dump output.bin --length 0x100000
spi-flash-tool --ft245 configure W25Q128JV --chips-dir ~/src/rflasher/chips/vendors

# Select a specific FT2232H by serial number (when multiple are connected)
spi-flash-tool --ft245 --ft-serial FT6XXXXX load firmware.bin
```

The FT2232H is used in asynchronous 245 FIFO mode. This requires a **one-time EEPROM configuration** to set Channel A to "245 FIFO" mode using [FT_PROG](https://ftdichip.com/utilities/#ft_prog) (Windows) or `ftd2xx_eeprom` (Linux). No special BitMode is set at runtime -- the host tool just opens the device and reads/writes normally.

For maximum direct-FT2232H throughput, build the FPGA image with the shorter real-FTDI FIFO timing profile:

```
make build-ft2232h
```

The default `make build`/`make build-pico` profile uses longer strobes needed by the Raspberry Pi Pico bridge and should still work with a real FT2232H, but more slowly.

**Wiring:** Connect the FT2232H Channel A pins to the FPGA dock as follows:

| FT2232H Pin | Signal   | FPGA Pin | Dock Location    |
|-------------|----------|----------|------------------|
| AD0-AD7     | D[0:7]   | H5, H8, G7, F5, H7, G8, G5, F3 | PMOD J7 |
| RXF#        | ft_rxf_n | D10      | PMOD J6 top      |
| TXE#        | ft_txe_n | G10      | PMOD J6 top      |
| RD#         | ft_rd_n  | B10      | PMOD J6 top      |
| WR#         | ft_wr_n  | H11      | Button S0 (core board) |

Note: H11 is a core board button pin, repurposed for FT245 (buttons are unused by NORbert). CLKOUT and OE# are not used in async mode. All signals are 3.3V LVCMOS.

## SPI read performance

Sustained SPI clock limits are set by the SDRAM prefetch pipeline in `spi_trx.v`.
Each 8-byte SDRAM burst takes ~12 system clock cycles (120 MHz) from activate to
data valid (tRCD=2 + tREAD=8 + 2-FF sync). The maximum SPI clock is determined by
how many SPI clocks the pipeline has between triggering the next SDRAM read and
needing the data (`fresh_read`):

**f_spi_max = 120 MHz x N_spi / 12**

### Sustained streaming (continuous sequential read)

| Command | Mode | Bits/clk | N_spi | Max SPI clock | Throughput |
|---------|------|----------|-------|---------------|------------|
| 0x03 Read | 1-1-1 | 1 | 4 | ~40 MHz | ~5.0 MB/s |
| 0x0B Fast Read | 1-1-1 | 1 | 4 | ~40 MHz | ~5.0 MB/s |
| 0x3B Dual Output | 1-1-2 | 2 | 8 | ~80 MHz | ~20 MB/s |
| 0xBB Dual I/O | 1-2-2 | 2 | 8 | ~80 MHz | ~20 MB/s |
| 0x6B Quad Output | 1-1-4 | 4 | 4 | ~40 MHz | ~20 MB/s |
| 0xEB Quad I/O | 1-4-4 | 4 | 4 | ~40 MHz | ~20 MB/s |

The single-width data pipeline (STA_READ) triggers the next SDRAM burst during byte
7 of the current burst (4 SPI clocks of headroom). The dual pipeline (STA_READ_DUAL)
triggers at byte 6 (8 SPI clocks), while quad (STA_READ_QUAD) also triggers at byte
6 but with only 4 SPI clocks due to 2 clocks/byte. This means dual and quad modes
hit the same ~20 MB/s throughput ceiling, but dual can clock 2x faster.

### First-burst limits

The initial SDRAM read is pipelined during the address/dummy phase. Commands with
dummy clocks get more headroom for the first burst:

| Command | N_spi (initial) | Max SPI (initial) |
|---------|-----------------|-------------------|
| 0x03 / 0x13 (no dummy) | 4 | ~40 MHz |
| 0x0B / 0x3B / 0x6B (8 dummy clocks) | 12 | ~120 MHz |
| 0xBB (dual addr + 4 mode/dummy) | 5 | ~50 MHz |
| 0xEB (quad addr + 6 mode/dummy) | 6 | ~60 MHz |

After the first burst, sustained rates apply. Commands with single-bit address
phases (0x0B, 0x3B, 0x6B) benefit most from dummy clocks. The multi-bit address
commands (0xBB, 0xEB) use fewer SPI clocks for the address, leaving less headroom.

## Project structure

```
src/
  top.v        Top-level module, clock/reset, bus wiring, TOCTOU address mux
  spi_trx.v    SPI flash transceiver (command decoder + data path)
  sdram.v      Dual-chip SDRAM controller with interleaved storage
  glue.v       Protocol handler, UART/FT245 I/O, SPI write engine, TOCTOU trap engine, LOGPOLL state machine, LED control
  logger.v     SPI event capture into a 512-byte ring FIFO drained by CMD_LOGPOLL
  uart.v       UART TX/RX (2 Mbaud)
  ft245.v      FT2232H async 245 FIFO interface
  fifo.v       Synchronous FIFO (first-word-fall-through)
  util.v       Clock divider, PWM, synchronizers
  pll.v        PLL: 50MHz -> 120MHz with phase-shifted outputs
pico-firmware/
  protocol/    no_std postcard RPC wire types for the Pico bridge
  firmware/    Embassy RP2040 USB + W5500 bridge firmware
tool/
  src/main.rs  Host-side CLI (Rust) for loading/dumping/monitoring/TOCTOU over UART or FT245
  src/chip.rs  Chip definition loading from rflasher RON database
  src/sfdp.rs  SFDP/BFPT table generation from chip definitions
```

## Serial protocol

All opcodes reply with a single `0x01` ACK unless otherwise noted. The FPGA
accepts command bytes from whichever port (UART or FT245) first delivers one
while the parser is idle, and routes the response back to the same port.

| Opcode | Name       | Args                                                | Reply                            |
|--------|------------|-----------------------------------------------------|----------------------------------|
| `0x30` | VERSION    | none                                                | 1 byte (current: `0x04`)         |
| `0x31` | RAMREAD    | 3-byte burst addr + 2-byte burst count              | `count*8` data bytes             |
| `0x32` | RAMWRITE   | 3-byte burst addr + 2-byte burst count + data       | `0x01`                           |
| `0x33` | CHIPCONFIG | JEDEC(3) + flags + erase_bursts(3) + sfdp_len + sfdp| `0x01`                           |
| `0x34` | START      | none -- enable SPI emulation                        | `0x01`                           |
| `0x35` | STOP       | none -- hold spi_trx in reset                       | `0x01`                           |
| `0x36` | STATUS     | none                                                | `0x01` running / `0x02` stopped  |
| `0x37` | HOLDCTL    | 1 byte: `0x01` assert, `0x00` release               | `0x01`                           |
| `0x38` | LOGCTL     | 1 byte: `0x01` start capture, `0x00` stop capture   | `0x01`                           |
| `0x39` | TOCTOU     | sub-command + args (see below)                      | `0x01`                           |
| `0x3A` | LOGPOLL    | none                                                | log bytes terminated by `0xA0`   |

TOCTOU sub-commands (all prefixed with opcode `0x39`):

| Sub    | Name      | Args                                                 |
|--------|-----------|------------------------------------------------------|
| `0x01` | SET       | index + start(3) + mask(3) + replace(3), all big-endian |
| `0x02` | ARM       | index                                                |
| `0x03` | DISARM    | index                                                |
| `0x04` | RESET     | index -- clear triggered flag                        |
| `0x05` | RESET_ALL | none -- disarm + clear all four                      |

RAMREAD/RAMWRITE/CHIPCONFIG are only accepted while emulation is stopped, to
avoid racing the SPI fast path on SDRAM. The other commands are always safe to
issue and bypass the SPI-idle gate so the host can reach the tool even while a
target is hammering the bus.

## Acknowledgments

- [Arisotura/spi_flash](https://github.com/Arisotura/spi_flash) -- SPI flash emulation logic adapted from this project
- [ArthurHeymans/tang_20k_spi_flash](https://github.com/ArthurHeymans/tang_20k_spi_flash) -- my first attempt at porting Arisotura's project, targeting different Tang hardware with 8MB of embedded DRAM
- [Trammel Hudson's SPISpy](https://trmm.net/SPISpy) -- the original FPGA-based SPI flash emulator that inspired this project
