# 1-bit SD Card Emulation Plan

> Status: implemented. Datapath prefetches one 8-byte burst ahead (gap-free
> at any practical SD clock), CMD18+CMD12 multi-block reads work, SCR
> advertises 1-bit only, CSD capacity derives from `image_sectors`, card is
> read-only (PERM_WRITE_PROTECT). `MODE` host command takes a mandatory
> argument byte (`0xFF` = query, `0x00`/`0x01` = set NOR/SD). Testbench:
> `make test` (iverilog; verilator 5.044 segfaults on the RTL task
> constructs, V3Life bug). Original plan below.

Goal: keep NORbert's current SPI NOR flash workflow, and add a selectable 1-bit native SD card mode backed by the same SDRAM and host transports.

Non-goals for the first version:

- SPI-mode SD card emulation
- 4-bit SD mode
- UHS / 1.8 V switching
- native SD writes, unless a target proves it needs them
- full Linux performance

## User experience

Keep the same shape as today's tool:

```sh
# Load an image into SDRAM, then expose it as a 1-bit SDHC card
norbert-tool mode sd
norbert-tool load sunxi.img
norbert-tool start

# Switch back to SPI NOR behavior
norbert-tool mode nor
norbert-tool configure W25Q128JV --chips-dir ~/src/rflasher/chips/vendors
norbert-tool load firmware.bin
norbert-tool start

# Common operations stay common
norbert-tool stop
norbert-tool status
norbert-tool dump out.img --length 64M
norbert-tool monitor
```

Internally this means adding a persistent runtime mode bit:

```text
MODE_NOR = current spi_trx path
MODE_SD  = new 1-bit SD card path
```

`STOP` disables whichever emulator is active. `START` enables the selected emulator.

## Pin reuse

The existing SPI NOR PMOD pins become SD pins in SD mode:

| SD card signal | Existing top-level pin |
|---|---|
| CLK | `spi_clk_pin` |
| CMD | `spi_mosi_pin` |
| DAT0 | `spi_miso_pin` |
| DAT1 | `spi_io2_pin` held high / high-Z |
| DAT2 | `spi_io3_pin` held high / high-Z |
| DAT3/CD | `spi_cs_pin` held high / card-detect behavior |

In NOR mode, keep the current assignments unchanged.

Add a simple top-level mux:

```text
NOR mode: pins connected to spi_trx
SD mode:  pins connected to sd_card_1bit
STOP:     all target-facing outputs high-Z except deliberate pull behavior
```

## RTL modules

Add one new fast-path module first:

```text
src/sd_card_1bit.v
```

Responsibilities:

- sample `sd_clk`
- receive and validate 48-bit commands on CMD
- generate SD responses on CMD
- stream read data on DAT0
- expose SDRAM burst read requests
- emit monitor events

Reuse existing modules:

```text
src/top.v      pin mux + mode mux
src/sdram.v    backing store
src/glue.v     host protocol, load/dump/start/stop/status
src/logger.v   monitor FIFO, with small packet additions if needed
```

Avoid touching `sdram.v` unless timing proves it necessary.

## Minimum SD protocol

Implement SDHC-style block addressing only. One SD block = 512 bytes.

Required init / identification commands:

| Command | Purpose | First behavior |
|---|---|---|
| `CMD0` | reset to idle | R1 idle |
| `CMD8` | voltage check | R7 echo, 2.7-3.6 V accepted |
| `CMD55` | prefix app command | R1 |
| `ACMD41` | init complete | return busy for a short count, then ready + HCS |
| `CMD58` | OCR | R3-like OCR if host sends it; useful compatibility |
| `CMD2` | CID | R2 long response |
| `CMD3` | assign RCA | fixed RCA, e.g. `0x0001` |
| `CMD9` | CSD | R2 long response |
| `CMD7` | select card | R1b/R1 enough for first pass |
| `CMD13` | status | R1/status |
| `ACMD51` | SCR | 8-byte data block; advertise 1-bit only |

Required read commands:

| Command | Purpose | First behavior |
|---|---|---|
| `CMD16` | block length | accept only 512 |
| `CMD17` | single block read | required |
| `CMD18` | multi-block read | likely required for Allwinner boot speed/flow |
| `CMD12` | stop transmission | ends `CMD18` |

Write commands can wait:

| Command | Purpose | First behavior |
|---|---|---|
| `CMD24` | single block write | reject/log until needed |
| `CMD25` | multi-block write | reject/log until needed |

## Card metadata

Use fixed, boring card registers generated in RTL or loaded from `glue.v` later.

First version constants:

- SD version: 2.0+
- card type: SDHC/SDXC block-addressed
- OCR: 3.3 V range, CCS set, no 1.8 V switch
- RCA: `0x0001`
- block size: 512 bytes
- capacity: derived from configured image size, rounded down to 512-byte sectors
- SCR: `SD_BUS_WIDTHS = 1-bit only`

If the host sends `ACMD6` requesting 4-bit mode, return an error/illegal command and log it. Do not silently switch.

## Data path

For reads:

```text
CMD17/CMD18 arg = sector number
sector byte address = arg << 9
SDRAM burst address = byte_address >> 3
```

DAT0 frame for each block:

```text
start bit
512 data bytes, MSB first
CRC16
end bit
```

Use the current 64-bit SDRAM burst interface:

- prefetch 8-byte bursts into a small local shift buffer
- issue the next SDRAM read before the current burst drains
- start conservative: support modest SD clocks first

For Allwinner boot testing, correctness beats speed. If timing is tight, document the max SD clock and add performance later.

## CRCs

Need real CRCs:

- CMD: CRC7 for incoming validation optional at first, outgoing responses must be valid
- DAT0: CRC16 required for read data

Implement tiny serial CRC modules inside `sd_card_1bit.v` or as local helper modules in the same file. No lookup tables.

## Host protocol additions

Add the smallest serial protocol changes:

| Opcode | Name | Args | Reply |
|---|---|---|---|
| new | `MODE` | `0x00` NOR, `0x01` SD | ACK |
| new | `SDCONFIG` | sector count / optional CID/CSD fields | ACK |

Current commands keep working:

- `RAMREAD` / `RAMWRITE`: image load/dump
- `START` / `STOP` / `STATUS`: gate current mode
- `LOGCTL` / `LOGPOLL`: monitor current mode

Tool naming can stay `norbert-tool` for now; rename later only if it gets annoying.

## Monitor output

Extend monitor packets enough to debug boot:

```text
SD CMD0 arg=00000000 -> R1 idle
SD CMD8 arg=000001aa -> R7 000001aa
SD ACMD41 arg=40300000 -> ready
SD CMD17 sector=8192
SD CMD18 sector=8192 count=...
SD CMD12 stop
SD ACMD6 4-bit rejected
```

This is the main bring-up tool. Do not build a fancy analyzer first.

## Bring-up order

1. Add mode bit and pin mux; verify NOR mode still builds.
2. Add `sd_card_1bit.v` command receiver and static responses for init commands.
3. Add CMD-line monitor logging.
4. Add `CMD17` single-block read from SDRAM.
5. Add `CMD18` + `CMD12` multi-block read.
6. Add SCR via `ACMD51`, advertising 1-bit only.
7. Test against an Allwinner board boot ROM/SPL.
8. Add writes only if the board actually issues writes.
9. Add 4-bit mode only if a real target refuses 1-bit.

## Validation

Small checks only:

- Verilator testbench for command frames and CRCs
- one fake host sequence: init + CMD17 + CMD18/CMD12
- hardware monitor trace from an Allwinner boot attempt

Success criteria:

- existing SPI NOR mode still works
- tool can load/dump the same SDRAM image
- Allwinner target sees a valid SDHC card
- target can read boot sectors over 1-bit DAT0

## Risks

- Some Allwinner boot stages may force 4-bit despite SCR. If so, add 4-bit later.
- SD clock timing is host-controlled; first version may need a conservative max clock.
- DAT3/card-detect wiring varies by board/socket. The breakout should expose DAT3 and any CD switch clearly.
- GPL code from ZipCPU should be used as a reference only unless this project adopts GPL-compatible licensing.
