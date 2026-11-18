# SPI-NOR controller compatibility

Protocol 9 separates JEDEC identity and transaction framing from chip internals.
The CLI and browser share `ChipProfile::build`; there is no protection-policy
whitelist. Conventional SDR SPI-NOR definitions are accepted when their geometry
fits 64 MiB SDRAM, power-of-two pages of 1–256 bytes, four addressed erase
commands and five contiguous regions per command. Invalid definitions fail
before any transport I/O.

This is a **permissive protocol emulator**, not a faithful chip model or a
promise that every SPI controller/chip combination works. Commands are
single-lane; address/data can use one, two or four lanes. QPI/command-lane modes,
DTR/octal, NAND/DataFlash, OTP/security storage and pages above 256 bytes need
additional engines or hardware. Unsupported framing must not be advertised.

## What remains accurate

- JEDEC response, address byte count, lane mapping and dummy/mode clocks.
- NOR-AND programming, selected page wrapping and byte-only/AAI framing.
- Regional erase sizes, alignment, capacity aliases and array contents.
- Real WEL/WIP, completion polling and safe STOP cancellation/draining.
- Bounded SFDP, with JESD216 encodings matching the supported default framing.

BP range arithmetic, WP#/hard locks, Atmel sector bitmaps, SST26 BPR/read locks
and protection enforcement are removed. Initialization writes are accepted,
protection/error indications read clear, and mutations never depend on those
bits. Status/configuration values useful to controllers are small shadows:

| Controller convention | Compatibility behavior |
|---|---|
| SR1 `01/05` | Config bits echo; BP/error/SRWD indications clear; WEL/WIP remain real |
| SR2 `31/35`, combined `01` | Config/QE echo; CMP/SRL bits `41` read clear; no QE gate |
| SR3 `11/15` | Shadow readback; legacy At25F `15` remains ID |
| Macronix C2 `01/15` | Second WRSR byte is Config; Config is not masked as SR2 protection; bit5 reflects active four-byte mode |
| Micron `B1/B5`, `81/85`, `61/65` | Little-endian 16-bit NVCR and byte VCR/EVCR shadows; NVCR survives soft reset and applies address/latency framing |
| Micron `70` | Ready `80`, busy `00`; no fabricated operation error |
| Spansion 01 `71/65` | Addressed config shadows; SR1 polling, CR1/QE and CR2 address-mode aliases; register reads repeat their byte, supporting zero/whole-byte dummy phases |
| Atmel `3C/36/39` | Addressed sector indication zero; protect/unprotect ignored |
| SST26 `72/42/98` | BPR reads zero; protection writes ignored; array reads unmasked |

Native `13/0C/3C/6C/BC/EC`, ordinary `03/0B/3B/6B/BB/EB`, program
`02/12/32/34` and Macronix `38` share the engine. `B7/E9/B8` and EAR/bank
registers control addressing. `9E/9F`, legacy `90/AB`, stable eight-byte `4B`
UID and `66/99` initialization are supported with their documented framing.
Known erase descriptors take priority over vendor aliases: AT26DF041's `81`
is a 256-byte eraser, not Micron VCR write; `50` can be an addressed eraser.
Core read/program/status opcode collisions are still rejected.

Configured read latency changes framing, unlike protection bits. Micron VCR
zero/F selects defaults: fast/output 8 clocks, dual-I/O 8, quad-I/O 10; other
nibbles select their count. NVCR bits 15:12 seed VCR and active-low bit0 seeds
four-byte mode on soft reset. Macronix Config[7:6] selects standard 4/6/8-clock
waits. Spansion CR2 changes address length and supported latency, including
code B/24 clocks. Multi-IO countdowns accommodate these longer phases.
Spansion page-size bit4 cannot advertise a buffer larger than 256 bytes.
This is sparse controller setup support, not a complete vendor register map.

## Host state and lifecycle

Every catalog selection starts unprotected. `--protected=true` is rejected;
`--protected=false` is a legacy spelling for the default. `--uid` supplies the
stable eight-byte response. Optional harmless SR/config shadows are accepted;
protected/error initial indications are rejected rather than pretending they
will be enforced. Macronix's Config alias is treated separately from SR2.

STOP/START retains shadows but clears volatile WEL/AAI/address state. Loading a
new profile reseeds state using a 32-bit epoch. STOP cancels uncommitted work,
drains accepted array work and scrubs the page buffer before acknowledging or
allowing host mutation. Soft reset never changes array data. Register writes
complete immediately; array WIP follows SDRAM work, not a timer. Busy operation
polling includes Spansion's addressed SR1, and cannot be hidden by raw masks.
No register state is persistent across FPGA power loss.

SFDP is conservative: only validated availability/uniform geometry is emitted,
QER is zero because reads are not QE-gated, and QPI/DTR/octal are not advertised.
Micron default mode/dummy fields differ from generic ones and match the engine.
Unsupported SFDP tables are blank, not fabricated. Profile reads past the
bounded 128-byte table return FF rather than aliasing its header. Low-level
CHIPCONFIG remains an explicit legacy/synthetic interface.

## Wire format and implementation

CHIPCONFIG `33` is unchanged. CHIPPROFILE `3C` accepts exactly 128 bytes while
stopped, now with **format 2** and **protocol 9**. Old format 1 is rejected. ACK
is `01` for success, `02` for invalid configuration; partial/timed-out/invalid
configuration blocks START. The host validates before I/O, and FPGA independently
validates structure/geometry before publishing, incrementing epoch or ACKing.

| Byte | Format2 field |
|---|---|
| 0 | 2 |
| 1 | Protocol alias: generic0, At2x2, SST26 3/4/5; no BP model1 |
| 2, 7 | SR1 BP/error indication masks; bits1:0 must be zero |
| 3–5 | Legacy writable/status hint slots, builder FF; no write gating |
| 6 | Zero, no SR1 lock |
| 8–9 | Legacy QE/native hints; engine does not gate those capabilities |
| 10 | Address-mode/EAR command-pair hints; EAR7 applies only to the indicated pair |
| 11 | Common commands, AAI, byte-only PP and legacy15 ID framing hints |
| 12–14 | Initial SR1/SR2/SR3 shadows; C2 byte13 is Config |
| 15 | Zero, no protection-sector shift |
| 16–23 | Stable UID in response order |
| 24 | Catalog addressless erase hints C7/60/62 |
| 25–27 | Zero, no BP policy, locks or initial protection |
| 28 | Actual page exponent 0–8 |
| 29–31 | Reserved zero |
| 32–127 | Four 24-byte erase descriptors |

Each descriptor is regular opcode, optional native opcode, region count,
reserved zero, then five records of size exponent plus 24-bit little-endian
block count. Active regions must align globally and exactly cover capacity.

A BRAM opcode ROM contains paired descriptors for the two possible final
command bits. It is read synchronously on the penultimate bit, then selected
without adding a master clock. Geometry is compiled through three arithmetic
stages while stopped into 32×50-bit system-write/SCK-read RAM. SPI scans five
region ends during the final address byte with one comparator. The retained
460-bit internal port contains only 332 control bits and 128 constant zeros;
there is no sector map. Small protocol shadows correctly remain registers.

## Verification

`make test` covers framing, all erase slots, synchronous lookup, paused SCK,
quad program variants and configured multi-lane latency. `make test-rflasher`
runs real pinned rflasher probing and mutations against RTL and byte-accurate
backing SDRAM, including broad vendor initialization, malformed profiles and
STOP ownership. `make test-gate` checks mapped primitive behavior. These tests
are interoperability evidence, not a universal controller guarantee or PMOD
physical timing sign-off. See [timing](rtl-timing.md) and
[utilization](rtl-utilization.md) for the measured hardware status.
