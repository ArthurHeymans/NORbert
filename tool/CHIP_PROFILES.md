# Controller-compatible chip configuration (protocol 9)

CLI `configure` and browser `configure_chip` share one validating builder.
They send CHIPCONFIG followed by the 128-byte format-2 CHIPPROFILE while stopped.
Success restores the prior running state. All chip/state validation precedes
any VERSION/STATUS query (CLI also validates before opening a connection).
Failures after reconfiguration begins leave emulation stopped. Protocol 8
cannot receive format-2 profiles; there is no automatic generic fallback.
The low-level `configure(jedec_id, size, supports_4byte, sfdp)` API remains an
explicit generic compatibility path for earlier FPGA protocols.

## Compatibility, not chip fidelity

Conventional 1/2/4-data-lane SDR SPI NOR parts are accepted whenever transaction
framing and data geometry fit the hardware. Unlock metadata is not an acceptance
whitelist: Unknown, Unsupported, BP3, BP4, N25q and unresolved protection flags
are accepted. Host tests cover Micron/N25Q, Macronix, GigaDevice, Spansion, ISSI,
Eon, Winbond, Atmel and SST. Optional QPI capability does not exclude a part's
ordinary SPI mode, but QPI command-lane, DTR, octal, NAND and DataFlash operation
are explicitly **not** implemented or advertised.

All array operations are unprotected. BP, sector locks, SST26 BPR and WP/QE
policies do not restrict erase/program/read. Common read/address/native,
status and reset commands are accepted regardless of catalog capability bits.
Register writes are shadows, not a simulation of nonvolatile protection.
SR1 reports ready/WEL dynamically and clears configured BP/error bits and SRWD;
SR2 normally clears CMP/SRL indications (`&BE`). Macronix manufacturer C2 uses
that storage as a full, unmasked Config shadow instead: WRSR01's second byte
writes Config, and READ15 returns it. C2 therefore accepts the full initial SR2
Config byte, including bits normally labeled CMP/SRL in other families.
Since the catalog has no explicit SR1 QE flag,
`QUAD_IO` without `QE_SR2` uses a compatibility fallback preserving SR1 bit6
configuration/QE shadow readback. Bit6 is excluded from both BP and error
indication masks, including contradictory custom metadata; remaining indication
bits are unchanged. BP/error masks must never contain WIP/WEL bits1:0;
the shared initial-state validator rejects inconsistent masks before I/O.
These masks are protocol hints, not protection policies or
claims about a physical chip's exact QE layout.
Atmel 3C aliases and SST26 BPR widths remain
transaction hints; BPR reads are zero and BPR writes have no protection effect.
Without an explicit erase descriptor, Micron 70 reports ready80 when idle and
zero when busy; B1/B5 (two-byte little-endian NVCR), 81/85 (VCR) and 61/65
(EVCR) are RTL shadow seams, not host-specified initial registers. Manufacturer
01 instead uses addressed 65 register reads and 71 WriteAny. Explicit addressed
erase geometry takes precedence over these vendor aliases; it must not be
rejected simply because another vendor uses the same opcode for configuration.

## Explicit hardware and framing limits

- Capacity: power of two, 256 bytes through 64 MiB inclusive.
- Page size: power of two, 1 through 256 bytes inclusive. Larger pages are
  rejected, never truncated. Byte28 carries the actual page-wrap exponent.
- Up to four addressed erasers, each with 1..5 sequential regions. Region sizes
  must be powers of two, 256 bytes through 64 MiB; counts are 1..0xffffff.
  Boundaries must align and each eraser must cover capacity exactly.
- Addressless C7/60/62 must be one whole-capacity region and have no native
  address variant. Native addressed erase opcodes and nonuniform boot-sector
  geometry are preserved exactly.
- Zero/FF, duplicate and core read/program/status-command-colliding erase
  opcodes are rejected (including 11/32/34/38). Explicit erase descriptors
  override vendor config/flag/lock aliases 61/65/70/81/85/B1/B5/E0/E2 and
  EWSR50, independent of unlock policy. AT26DF041 is accepted with its actual
  81 256-byte, 50 2-KiB and 20 4-KiB erase geometries intact; no hardware or
  wire-format change is needed to resolve those aliases.
- Explicit At2x 3C/native dual-output ambiguity and EAR7 without an explicit
  bank-register command pair are rejected rather than guessed.

## Format-2 wire defaults

The layout stays 128 bytes: 32-byte header, four 24-byte erase descriptors.
Descriptor byte0/1 are regular/native opcodes, byte2 region count, byte3 zero;
up to five records contain size exponent and a little-endian 24-bit count.
Unused records/descriptors are zero.

| Byte | Meaning/default |
| --- | --- |
| 0 | Format 2 |
| 1 | Generic 0; At2x alias 2; SST26 BPR read widths 3/4/5 |
| 2 | BP indication hint only; bits1:0 must be zero; exclude bit6 for QUAD_IO without QE_SR2; never a range policy |
| 3/4/5 | FF shadow-write masks/common status commands |
| 6 | 0; locks unused |
| 7 | Error indication hint, forced success; bits1:0 must be zero; same bit6 config/QE exclusion as byte2 |
| 8 | 2 for explicit SR2 QE metadata, otherwise 0; never gates quad |
| 9 | 7F common native addressing/read/program superset |
| 10 | 1B normally; EAR7 parts use 07 plus their actual C5/C8 bit3 and/or 17/16 bit4 |
| 11 | A7 common fast/dual/quad, UID4B and reset; bit3 AAI, bit4 byte-only02 from Byte write granularity, WRITE_BYTE or AAI, bit6 legacy ID15 |
| 12/13/14 | Initial SR1/SR2/SR3 shadows, default 0/0/0; byte13 is full Config for manufacturer C2 |
| 15 | 0 |
| 16..23 | Stable eight-byte UID in SPI response order, default zero |
| 24 | Catalog addressless chip-erase opcode bits C7/60/62 |
| 25/26/27 | 0: no range policy, locks or protected initial flags |
| 28 | Actual page-size exponent, 0..8 |
| 29..31 | 0 |

## Initial state and SFDP

Default state is always unprotected. `--protected` / JavaScript `protected=true`
reject clearly **before I/O** because protection cannot be enforced;
`--protected=false` remains a compatibility spelling of the default. The browser
has no protected/default-protected choices. Rust `ChipState` may initialize
harmless configuration shadows; SR1 BP/error/SRWD/busy/WEL bits reject, rather
than imply protection. Other families reject initial SR2 CMP/SRL bits41, but
Macronix C2 accepts the entire Config byte (for example47), with unmasked
readback. Other SR2/QE and SR3 shadows have no array-protection effect.

`--uid 0123456789abcdef` sets eight stable response bytes in exactly that SPI
order for any accepted part. Only opcode4B with its common dummy framing is
promised; alternate vendor UID framing is not inferred.

SFDP presence remains conservative: explicit SFDP metadata or known modern
Winbond identities, uniform addressed geometry only. Legacy Atmel/SST25 and
nonuniform erasers have no signature. Native-only addressing above 16 MiB is
blanked because this host emits BFPT, not 4BAIT. BFPT describes SDR address,
mode/dummy clocks, erase sizes and actual page size; QER=0 reflects the ungated
quad engine. QPI/DTR/octal support is never advertised.

SFDP encodes **static default** SDR framing, not dynamically configured latency:

- Micron manufacturer20 defaults (VCR/NVCR latency0/F): BB DualIO has mode0,
  dummy8; EB QuadIO has mode0, dummy10. Fast/output reads remain8 clocks.
- Generic defaults, including Macronix C2 Config0: BB has mode4, dummy0
  (total4); EB has mode2, dummy4 (total6). Fast/output reads remain8 clocks.
- Runtime C2 Config[7:6] code1 selects fast6/DualIO6/QuadIO4; code2 selects
  DualIO8/QuadIO8; otherwise defaults are fast8/DualIO4/QuadIO6. These runtime
  overrides do not alter the synthesized static-default BFPT.

Fixtures are independent wire examples, not physical-chip SFDP captures.
WASM checks require WebUSB's
`RUSTFLAGS="--cfg=web_sys_unstable_apis"` with the pinned web-sys dependency.
