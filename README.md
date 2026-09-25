# NORbert

A friendly FPGA that pretends to be your SPI NOR flash.

NORbert uses a [Sipeed Tang Primer 25K](https://wiki.sipeed.com/hardware/en/tang/tang-primer-25k/primer-25k.html) to emulate a SPI NOR flash chip, backed by 64MB of SDRAM. Load a firmware image over UART, and NORbert will serve it to your target system as if it were a real flash chip -- supporting single, dual, and quad SPI read modes with pipelined SDRAM prefetch for real-time streaming.

## Features

- **SPI NOR flash emulation** with full command support: read, fast read, page program, sector/block/chip erase, JEDEC ID, status registers, SFDP
- **Configurable chip identity** at runtime -- load any chip definition from [rflasher](https://github.com/benpye/rflasher)'s RON database to set JEDEC ID, size, and SFDP parameters (defaults to Winbond W25Q64FV)
- **Multi-I/O modes**: 1-1-1, 1-1-2, 1-2-2, 1-1-4, and 1-4-4 SPI read modes
- **Fast SPI reads**: one-burst lookahead with ping-pong SDRAM buffers sustains 50 MHz single/dual/quad reads and 60-70 MHz quad reads (see limits below)
- **64MB backing store** using two SDRAM chips with byte-serial burst layout for minimal first-byte latency
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

## SPI Flash Pin Mapping

NORbert exposes the SPI flash interface on the **PMOD J5** connector of the Tang Primer 25K Dock. The table below maps the standard SPI flash signals to the corresponding FPGA I/O pins:

| SPI Signal | FPGA Pin | PMOD J5 Pin | Notes                              |
|------------|----------|-------------|------------------------------------|
| `/CS#`     | `T9`     | 1           | Chip Select (active-low)           |
| `SCK`      | `T8`     | 2           | SPI Clock                          |
| `D0/DO`    | `R9`     | 3           | Data Out / IO0                     |
| `D1/DI`    | `R8`     | 4           | Data In / IO1                      |
| `D2`       | `L8`     | 5           | IO2 (used for Dual/Quad reads)     |
| `D3`       | `L9`     | 6           | IO3 / `#HOLD#` (shared function)   |
| `GND`      | —        | 10          | Ground                             |
| `VCC`      | —        | 9           | 3.3V Power                         |

*Note: D3 and `#HOLD#` share the physical IO3 pin. Asserting `#HOLD` drives it low to silence a real flash on a shared bus. Consult `tangprimer25k.cst` for exact pin assignments.*

## Releases

Tagged releases publish a Gowin `.fs` bitstream for the Tang Primer 25K. Download
`spi_flash.fs` from the [latest release](https://github.com/ArthurHeymans/NORbert/releases/latest)
and load it with openFPGALoader:

```sh
# Volatile: runs immediately, lost on power-off
openFPGALoader -b tangprimer25k spi_flash.fs

# Persistent: boots automatically after power-on
openFPGALoader -b tangprimer25k -f spi_flash.fs
```

Each release carries one bitstream, named `spi_flash.fs` so that the
`releases/latest/download/...` link in the web UI keeps working across
releases, plus a `SHA256SUMS` file to check it against:

```sh
sha256sum --check SHA256SUMS
```

Release tags exactly match the Rust package version (for example, tag `0.1.0`
uses `version = "0.1.0"` in `tool/Cargo.toml`). The release workflow rejects a
mismatch.

## Building

### Prerequisites

If you have [Nix](https://nixos.org/) with flakes enabled, just enter the dev shell:

```sh
nix develop    # or let direnv handle it
```

This provides the Gowin IDE (Education Edition), yosys, openFPGALoader, verilator, the Rust toolchain with the WebAssembly target, `wasm-bindgen-cli`, and Trunk for building and serving the Web UI.

Without Nix, you'll need:

- [Gowin IDE Education Edition](https://www.gowinsemi.com/en/support/home/) v1.9.11.03 (`gw_sh` on PATH)
- [openFPGALoader](https://github.com/trabucayre/openFPGALoader)
- [yosys](https://github.com/YosysHQ/yosys) (optional, for linting)
- [Verilator](https://verilator.org/) and a C++ toolchain (for linting/simulation)
- Rust toolchain (for the host tool)

### FPGA bitstream

```sh
make build                  # synthesize + place & route
make prog                   # program FPGA (volatile, lost on power cycle)
make flash                  # program to flash (persistent)
```

### RTL checks

```sh
make lint                   # Yosys synthesis checks and Verilator lint
make test                   # SPI, SDRAM, TOCTOU, FIFO, logger, UART, FT245
```

The tests cover SFDP startup/reconfiguration, page-program and AAI semantics,
program latency/refresh, accepted SDRAM addresses during redirected reads, and
three- versus four-byte addressing: every four-byte read command at all start
offsets, the 0xB7/0xE9 address-mode pair, the four-byte program and erase
forms, and a chip that ignores the mode commands entirely.
The transport and buffer blocks are covered by their own benches: the FIFO
against a queue model (`tests/fifo_tb.sv`), the logger's byte stream against
the format the host decoder expects (`tests/logger_tb.sv`), the UART over all
256 byte values in both transmitter configurations (`tests/uart_tb.sv`), and
the FT245 handshake against a time-accurate FT2232H model (`tests/ft245_tb.sv`).
They use a burst-level memory model or a clock-only PLL stub; they do not replace
Gowin timing analysis or hardware validation of SDRAM pin timing.

### Host tool

```sh
make tool
```

This builds `spi-flash-tool` at `tool/target/release/spi-flash-tool`.

The tool also exposes a WebAssembly library for browser frontends.
`WebFlashDevice.requestUsb()` opens the FT2232H FT245 interface through WebUSB,
while `WebFlashDevice.requestSerial()` opens the dock's 2 Mbaud UART through Web
Serial. The returned device provides the same protocol operations for either
transport, automatically stops and restores SPI emulation around RAM and
configuration changes, and applies I/O timeouts. Browser
frontends should prefer `read_chunks` and `write_file` for bounded-memory
transfers with progress and cancellation callbacks. Check the WASM build with:

```sh
rustup target add wasm32-unknown-unknown
cargo check --manifest-path tool/Cargo.toml \
  --target wasm32-unknown-unknown --no-default-features --features wasm --lib
```

`WebFlashDevice.requestUsb()` and `WebFlashDevice.requestSerial()` must be called
from a browser user gesture so the browser can display its device
permission picker. Web Serial currently requires a Chromium-based browser and,
like WebUSB, a secure context (HTTPS or localhost).

A ready-to-use frontend is in `web/`. From the Nix development shell, build and
serve it with Trunk:

```sh
make webui-serve
```

Without Nix, first install the WebAssembly target, Trunk, and the
`wasm-bindgen-cli` version recorded in `tool/Cargo.lock`:

```sh
rustup target add wasm32-unknown-unknown
cargo install trunk
cargo install wasm-bindgen-cli --version 0.2.127
make webui-serve
```

Open <http://localhost:8081> and choose either **Connect FT245 (WebUSB)** or
**Connect UART (Web Serial)**. The UI uses the compiled-in rflasher database to
search for and configure the emulated chip, and provides emulation control,
verified SDRAM uploads and downloads, target-flash `#HOLD`, decoded live SPI
activity monitoring, and full TOCTOU trap configuration.

The **Bitstream** tab can program a release or locally built Gowin `.fs` file
straight into volatile SRAM or persistent configuration flash. This is a Rust/
WebAssembly port of the FT2232H MPSSE/JTAG and Gowin GW5A paths used by
openFPGALoader, built on the existing `ftdi-nusb`, `nusb`, and WebUSB support.
Disconnect an active FT245 connection and close native JTAG tools before
programming because they also claim FT2232H interface A. On Linux, unbind the
`ftdi_sio` driver from interface A if Chromium reports that it cannot claim the
interface. On Windows, interface A must use the WinUSB driver (for example,
configured with Zadig); leave interface B on its normal driver.

#### Browser JTAG wiring

The Dock's onboard debugger is a BL616, not an FT2232H. Browser programming
therefore needs an external FT2232H connected to the Dock's external-JTAG
header. On Dock schematic revision 60033 this connector is J7:

| FT2232H channel A | J7 pin | Signal |
|-------------------|--------|--------|
| AD3 / CS           | 1      | TMS    |
| AD1 / DO           | 2      | TDI    |
| AD0 / SK           | 3      | TCK    |
| AD2 / DI           | 5      | TDO    |
| GND                | 6      | GND    |
| VIO reference      | 7      | 3.3 V  |
| GND                | 8      | BL616_EN |

Leave J7 pin 4 (`+5V`) unconnected. Grounding `BL616_EN` disables the onboard
debugger while the external adapter is attached. Pin 7 is a logic-voltage
reference; do not use it to power the Dock or target from the adapter.

Flash programming requires a readable JEDEC SFDP Basic Flash Parameter Table
and checks the write and sector-erase range against the reported capacity
(and the programmer's 24-bit address limit) before changing protection or
issuing an erase. Chips without supported SFDP data are rejected rather than
assuming a capacity. Incomplete `.fs` configuration data is rejected before
opening the JTAG adapter.

Browser device APIs are unavailable when opening `web/index.html` directly as
a `file://` URL. WebUSB requires Chromium and a secure context (HTTPS or
localhost).

## Usage

Load a firmware image into NORbert's SDRAM, then let your target SPI master read it back as if it were a real flash chip.

```sh
# Check connection
spi-flash-tool version
spi-flash-tool status       # running | stopped

# Check whether the SPI fast read path ever fell behind (clears the flags)
spi-flash-tool prefetch

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

```sh
spi-flash-tool monitor
```

Example output while flashprog reads a 4 KB region:

```text
TXN#   COMMAND            ADDRESS    INFO
------------------------------------------------------------
1      0x9F READ_JEDEC_ID
2      0x05 READ_STATUS
3      0x05 READ_STATUS
4      0xBB DUAL_IO_READ  0x001000
       end: 4097 bytes from 0x001000
```

The monitor also tracks double-reads of the same (opcode, address) pair and flags them as TOCTOU candidates. Press Ctrl+C to stop. The underlying protocol is a poll-based ring-buffer drain (`CMD_LOGPOLL` = 0x3A); packet types are `0xA1` (command), `0xA2` (address), `0xA3` (end + byte count) and `0xA4` (TOCTOU trap fired), with `0xA0` as the per-poll terminator. Log bytes equal to `0xA0` or `0xA5` are sent as `0xA5 0x00` and `0xA5 0x05`.

The ring holds 503 bytes (512 entries less FIFO headroom), with one event
pending per packet type behind it. A host that polls slower than the target
reads will therefore see gaps: the FPGA drops the events it has no pending
slot for rather than overwriting the ring, so a transaction can lose some of
its packets (its address, say) and keep others. Packets that do arrive are
always whole and in order, never torn, corrupted or reordered.
`tests/logger_tb.sv` checks both the loss and the integrity.

### TOCTOU traps

Four independent trap entries redirect matching reads to a different SDRAM location on the second (and subsequent) access. The first matching read is let through unchanged -- it arms the trap. The first SDRAM burst (up to 8 bytes, depending on starting alignment) always comes from the original address; subsequent bursts come from the replacement. All matching entries become triggered, and the highest-index already-triggered match selects the replacement when traps overlap.

```sh
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

```sh
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

The FT2232H is used in asynchronous 245 FIFO mode. This requires a **one-time EEPROM configuration** to set Channel A to "245 FIFO" mode using [FT_PROG](https://ftdichip.com/utilities/#ft_prog) (Windows) or `ftdi_eeprom` (Linux). No special BitMode is set at runtime -- the host tool just opens the device and reads/writes normally.

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

Sustained SPI clock limits are set by the SDRAM prefetch pipeline
(`spi_prefetch.v` and `sdram.v`). Each 8-byte SDRAM burst takes ~12 system clock cycles (120 MHz)
from post to data valid. Instead of posting just-in-time, the SPI engine keeps
one burst in flight at all times (one-burst lookahead with ping-pong buffers),
so the limit is throughput (one burst per ~12 sysclks), not single-burst
latency. SDRAM runs at CAS latency 2 (in spec to 133 MHz for the W9825G6KH-6)
with a byte-serial burst layout, so the first beat already completes bytes 0-1.

Validated by simulation (`make test`, `tests/quad_fast_tb.sv`: all start
offsets 0-7, row/bank crossings, refresh coexistence). 60-70 MHz operation
also needs timing closure past the default 30 MHz `spi_clk` constraint plus
signal-integrity validation on the PMOD leads -- simulated margins below do
not replace that.

| Command | Mode | Simulated max (all offsets) | Notes |
|---------|------|-----------------------------|-------|
| 0x03 Read | 1-1-1 | 50 MHz | No dummy clocks; hardest first burst |
| 0x0B Fast Read | 1-1-1 | 70 MHz | 8 dummy clocks |
| 0x3B Dual Output | 1-1-2 | 50 MHz (spot) | 8 dummy clocks |
| 0xBB Dual I/O | 1-2-2 | 60 MHz | 4 mode clocks |
| 0x6B Quad Output | 1-1-4 | 70 MHz | 8 dummy clocks |
| 0xEB Quad I/O | 1-4-4 | 50 MHz | 6 mode clocks; offset 7 flags thin |

Known corners (all flagged by the FPGA's prefetch fault flags -- see
`spi-flash-tool prefetch` below, never silent):

- **Quad-I/O reads starting at offset 7 above 50 MHz** need the second burst
  ~30 ns before a single SDRAM controller can physically produce it. Use
  offset <= 6 or <= 50 MHz for 0xEB.
- **Slow reads above ~60 MHz** run out of first-burst window (3.5 clocks, no
  dummy). Real NOR flashes cap 0x03 the same way (f_R < f_C).
- **70 MHz dual-I/O offset 7** passes with thin margin flagged.

If you need more: the levers are open-row (skip re-ACTIVATE on same-row
bursts, saves ~2 sysclks per burst) and master-configured extra dummy clocks
for 0xEB/0xBB, not a faster SDRAM clock.

### Checking the prefetch path

The corners above are detected in the FPGA, not just in simulation. Run a
target read, then ask the FPGA what happened:

```sh
spi-flash-tool prefetch
```

The flags are latched in the FPGA and cleared by the read that reports them,
so the check covers everything since the previous one:

- **underrun** (`0x01`) -- a burst was shifted out of the SPI data path
  before the SDRAM controller filled it. The target read stale bytes. This
  is a correctness failure, not a margin warning.
- **thin margin** (`0x02`) -- the next burst had not landed when its first
  byte was needed. Not necessarily corruption, but the margin is gone and a
  slightly longer SDRAM access would turn it into an underrun.

The command is safe to run while the target is reading, and the web UI has
the same check as **Check prefetch** on the Device panel. An underrun means
the frequency, start offset or SDRAM latency of that read is outside the
supported envelope in the table above -- the data the target received cannot
be trusted.

## Project structure

```text
src/
  top.v        Top-level module, clock/reset, bus wiring, TOCTOU address mux
  spi_trx.v    SPI flash transceiver (command decoder + data path)
  spi_prefetch.v  SDRAM burst requests, ping-pong buffer selection and the prefetch fault flags
  sdram.v      Dual-chip SDRAM controller, byte-serial bursts, ping-pong prefetch
  spi_flash_cmds.vh  Emulated SPI flash opcodes and read wait states
  host_protocol.vh   Host serial protocol opcodes and log packet types
  glue.v       Protocol handler, UART/FT245 I/O, SPI write engine, TOCTOU trap engine, LOGPOLL state machine, prefetch fault latch, LED control
  logger.v     SPI event capture into a 512-byte ring FIFO drained by CMD_LOGPOLL
  uart.v       UART TX/RX (2 Mbaud)
  ft245.v      FT2232H async 245 FIFO interface
  fifo.v       Synchronous FIFO (first-word-fall-through)
  util.v       Clock divider, PWM, synchronizers
  pll.v        PLL: 50MHz -> 120MHz with phase-shifted outputs
tool/
  src/main.rs  Host-side CLI (Rust) for loading/dumping/monitoring/TOCTOU over UART or FT245
  src/chip.rs  Chip definition loading from rflasher RON database
  src/sfdp.rs  SFDP/BFPT table generation from chip definitions
```

## Serial protocol

`src/host_protocol.vh` defines these values for the FPGA; `cargo test` checks
that `tool/src/protocol.rs` matches it.

All opcodes reply with a single `0x01` ACK unless otherwise noted. The FPGA
accepts command bytes from whichever port (UART or FT245) first delivers one
while the parser is idle, and routes the response back to the same port.

`VERSION` reports the protocol version, currently 6. The host tool talks to
any version from 3 up to its own and enables commands by version, but it
refuses a newer bitstream rather than guess at its protocol: a version 6
bitstream needs a tool from the same release or later.

| Opcode | Name       | Args                                                | Reply                            |
|--------|------------|-----------------------------------------------------|----------------------------------|
| `0x30` | VERSION    | none                                                | 1 byte (current: `0x06`)         |
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
| `0x3B` | PREFETCH   | none                                                | 1 byte: `0x80` valid + `0x01` underrun + `0x02` thin |

A clean PREFETCH reply is `0x80` (not `0x00`, which the host discards as
transport noise); `0x81`, `0x82` and `0x83` report faults. PREFETCH is a
single byte with no argument, like STATUS: over FT245 the FPGA only takes
the always-safe opcodes while the target holds CS low, so a stray argument
byte would block every command queued behind it. It is safe to issue in any
emulation state, and reading it clears the flags it reports, so a fault is
reported to exactly one reader.

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
