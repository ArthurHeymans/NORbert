# RTL correctness changes: synthesis utilization

Counts are the sum of mapped `LUT1`–`LUT4` cells in the flattened top-level
netlist. The historical correctness table below was measured with the flake's
Yosys 0.69+norbert and:

```
read_verilog -Isrc <Makefile VERILOG_FILES>
synth_gowin -family gw5a -nolutram -nowidelut -top top
stat
```

Each cell uses one physical LUT. ALUs/carry cells, flip-flops, and BSRAM are
not included. These are reproducible synthesis counts, not Gowin post-route
utilization or timing sign-off. Deltas compare against the preceding row.

| Change | LUTs | Delta |
|---|---:|---:|
| Baseline before correctness fixes | 3134 | — |
| Bound refresh deferral and release refresh during host TX stalls | 3169 | +35 |
| Preserve write completion while deselected | 3190 | +21 |
| Backpressure FT245 reads with RX FIFO capacity | 3178 | −12 |
| Check consumed beats and preserve progressive-fill ownership | 3202 | +24 |
| Queue logger frames chronologically and report dropped events | 3494 | +292 |
| Hold SDRAM requests until explicit acceptance | 3532 | +38 |
| Isolate page-buffer and NOR program/RMW engine | 3342 | −190 |
| Separate host protocol and explicit SDRAM client ownership | 3435 | +93 |
| Declare PLL phases and bound SPI/system crossings; document I/O contracts | 3435 | 0 |
| Align strict lint mapping and acknowledge its experimental XAIG backend | 3435 | 0 |
| Retain deferred refresh debt and register dispatch deadlines | 3398 | −37 |
| Close SDRAM rows left open by withdrawn SPI reads; SPI/serial row interlock | 3448 | +50 |
| Refresh only in acknowledged SPI lookahead windows | 3429 | −19 |
| Ignore commands while busy; hold program commands; abort unaligned programs | 3442 | +13 |
| Buffer UART input; gate only SDRAM-owning host commands; payload timeout | 3563 | +121 |
| Restrict TOCTOU traps to array reads; redirect before lookahead | 3528 | −35 |
| Continuous read mode for dual and quad I/O reads | 3615 | +87 |
| Honor the power-detect bypass for output enables | 3627 | +12 |

Upstream pre-profile total: **3627 LUTs**, **+493** versus the 3134-LUT
baseline. Most of the last block is the 16-entry UART receive FIFO, which
`-nolutram` maps to flip-flops and LUT multiplexers, and continuous-read decode.
The measurements below predate integration of these upstream changes unless
explicitly identified as rebased.

The logger queue also adds four `SDPX9B` BSRAM cells (the original four
`DPX9B` cells remain). It trades those blocks and LUTs for capture-order
preservation, atomic event payloads, and explicit loss accounting.

## Rebased protocol 9 controller compatibility

After integrating `master@origin` at `feb4f6a5`, upstream Yosys 0.69+post
(`v0.69`) maps the flattened top to **6,137 LUT1–LUT4 cells**, **1,014
ALU/carry cells**, **3,875 flip-flops**, four `DPX9B`, six `SDPX9B` and one
`SPX9`. `check` reports zero problems. The synthesis log is
`/tmp/norbert-pr-integration/synthesis.log`; the counts include upstream UART,
refresh/row, partial-command and continuous-read behavior.

Gowin Education 1.9.11.03 placed/routed the source-hashed snapshot
`/tmp/norbert-gowin-pr-rebase/` with the unchanged 120/40 MHz constraints:

| Resource | Used / available | Reported utilization |
|---|---:|---:|
| Logic | 5,872 / 23,040 | 26% |
| Registers | 3,190 / 23,280 | 14% |
| CLS | 4,161 / 11,520 | 37% |
| BSRAM | 10 / 56 | 18% |
| I/O | 62 / 86 | 73% |

Native composition is 4,688 LUTs, 704 ALUs and 80 RAM16s, with no DSP.
SPI Fmax is **68.799 MHz** and system Fmax **121.281 MHz**. Same-clock
setup/hold counts are zero, but system margin is approximately 0.088 ns and
**full timing sign-off still fails**: 210 setup and 79 hold inter-clock
violations remain. Resource fit and a generated bitstream are not hardware
sign-off. See [timing](rtl-timing.md) for the remaining CDC/I/O limitations.

The snapshot's `source.sha256` matches the live RTL and constraints. Native
reports are `impl/pnr/spi_flash.rpt.txt` and `spi_flash_tr_content.html`.
Historical counts below use earlier sources/tool revisions; they are not
a substitute for this rebased measurement.

### Protocol 9 before upstream integration (historical)

After local system-timing fixes, the permissive implementation maps to
**5,810 LUT1–LUT4 cells**, **989 ALU/carry cells**, **3,713 flip-flops**, four `DPX9B`, six `SDPX9B` and one
`SPX9` cells, using the same upstream Yosys 0.69 flow below. `check` reports
zero problems. The opcode ROM maps to `SPX9`; the dual-clock erase table still
uses two added `SDPX9B` blocks. Small protocol/configuration shadows remain
registers, not RAM bitmaps.

| Same-toolchain design | LUTs | ALU/carry | Flip-flops |
|---|---:|---:|---:|
| Pre-profile baseline | 3,996 | 674 | 2,742 |
| Initial protection-heavy implementation | 12,540 | 3,893 | 4,498 |
| BRAM geometry, retained protection | 8,286 | 1,358 | 3,997 |
| Permissive controller compatibility before timing repair | 5,801 | 1,008 | 3,585 |
| Local system-timing repair | 5,810 | 989 | 3,713 |

This removes **2,476 LUTs (30%)** from the previous optimized design and
**6,730 (54%)** from the initial implementation, while broadening supported
catalog definitions instead of keeping a protection whitelist. Added cost
above the baseline falls from 4,290 to 1,814 LUTs. Protection circuitry is
intentionally deleted; this is not a fidelity-preserving optimization.

The timing repair adds only **9 LUTs and 128 flip-flops**, removes 19 ALU/carry
cells and eliminates the native validator DSP. Logger capture staging, a
predicted SDRAM wait flag and pipelined validation preserve protocol and
geometry support; they do not simplify the design by dropping chips.

Native Gowin Education 1.9.11.03 place/route with unchanged 40 MHz SPI and
120 MHz system constraints generated a bitstream from the source-hashed
snapshot `/tmp/norbert-gowin-timing-p0/`:

| Resource | Used / available | Reported utilization |
|---|---:|---:|
| Logic | 5,748 / 23,040 | 25% |
| Registers | 3,157 / 23,280 | 14% |
| CLS | 4,135 / 11,520 | 36% |
| BSRAM | 10 / 56 | 18% |
| DSP | 0 / 28 | 0% |
| I/O | 62 / 86 | 73% |

Native composition is 4,600 LUTs, 680 ALUs and 78 RAM16s; vendor logic accounting
is not the plain-LUT Yosys count. **Resource fit and same-domain timing pass,
but full timing sign-off does not.** SPI Fmax is **65.383 MHz** (40 required);
system reaches **121.239 MHz** (120 required), with zero same-clock setup/hold
violations but only approximately 0.085 ns system margin. Overall inter-clock
violations remain: 141 setup and 122 hold endpoints. No clock was lowered or
crossing false-pathed to claim success. See [timing](rtl-timing.md); configuration
CDC, PMOD timing and board behavior remain unverified.

Final Yosys evidence is `/tmp/norbert-timing-p0/synthesis.log`. Native resource
and STA reports are respectively `impl/pnr/spi_flash.rpt.txt` and
`impl/pnr/spi_flash_tr_content.html` under the hashed snapshot. These measurements
apply to RTL, not subsequent documentation or test-only edits.

## Protocol 8 chip profiles (historical)

The previous optimized implementation, including STOP drain/cancellation and bounded
SFDP reads, maps to **8,286 LUT1–LUT4 cells**, **1,358 ALU/carry cells**,
**3,997 flip-flops**, four `DPX9B` and six `SDPX9B` cells. `check` reports no
problems. These measurements use upstream Yosys 0.69, not the experimental
XAIG backend of the historical table. Do not compare them directly against
3,398 or add carry cells to a physical utilization estimate.

| Same-toolchain design | LUTs | ALU/carry | Flip-flops | DPX9B | SDPX9B |
|---|---:|---:|---:|---:|---:|
| Pre-profile baseline | 3,996 | 674 | 2,742 | 4 | 4 |
| Initial chip-profile implementation | 12,540 | 3,893 | 4,498 | 4 | 4 |
| Compiled tables and shared protection query | 8,286 | 1,358 | 3,997 | 4 | 6 |

The reduction is **4,254 LUTs (34%)**, without removing supported chips or
changing the 128-byte wire format. The added LUT count above the same-tool
baseline falls from 8,544 to 4,290, roughly half; it remains substantial.

Reproduce a flattened measurement with:

```sh
yosys -l synthesis.log -p "
read_verilog -lib $(yosys-config --datdir)/gowin/cells_xtra_gw5a.v;
read_verilog -Isrc src/top.v src/spi_trx.v src/spi_prefetch.v src/sdram.v \
  src/glue.v src/spi_program.v src/host_protocol.v src/chip_profile_store.v \
  src/sdram_client_mux.v \
  src/uart.v src/ft245.v src/fifo.v src/logger.v src/util.v src/pll.v;
synth_gowin -family gw5a -nolutram -nowidelut -top top;
stat; check
"
```

`chip_profile_store` compiles geometry while stopped into a 32×50-bit
system-write/SCK-read RAM. Yosys maps `spi_trx_i.erase_table` to two additional
`SDPX9B` cells. A 460-bit control bundle replaces the raw 1024-bit register
vector. The SPI side scans five region ends with one comparator during the
final address byte, retaining only the chosen burst mask. Mutation paths
share one protection query; Atmel uses direct indexing for programs, a
reduction for whole-chip erase and eight masked 16-sector words for ranges.
Small status/protection state remains in registers; it is not a BRAM bitmap.

Hierarchical, unparameterized unit-netlist counts are not comparable to this
flattened top-level count. Functional gate tests use the
GW5A simulation models pinned by `flake.lock`; stock models may omit the RAM
primitives. `CELLS_SIM=/path/to/cells_sim.v make test-gate` overrides their path.

## Historical protocol 8 Gowin fit verification

Gowin Education 1.9.11.03 placed/routed the optimized design at revision
`f16dccc1` for `GW5A-LV25MG121NC1/I0`, using the checked-in `build.tcl`, CST
and SDC. The build produced a bitstream; all source/constraint hashes matched
the tested snapshot afterward. Native resource usage is:

| Resource | Used / available | Reported utilization |
|---|---:|---:|
| Logic | 8,108 / 23,040 | 36% |
| Registers | 3,444 / 23,280 | 15% |
| CLS | 5,572 / 11,520 | 49% |
| BSRAM | 9 / 56 | 17% |
| I/O ports | 62 / 86 | 73% |

Logic includes 6,631 LUTs, 1,009 ALUs and 78 RAM16 instances; use the tool's
8,108 logic total rather than adding unlike resource counts. Vendor mapping
allows LUT RAM and differs from the plain-LUT Yosys flow above. The vendor
synthesis hierarchy assigns two BSRAM blocks to `spi_trx_i` for the erase table.

**Fit passes, timing fails.** Reported Fmax is 35.377 MHz for SPI (40 MHz
required) and 77.453 MHz for system (120 MHz required). See
`docs/rtl-timing.md`; bitstream generation is not timing sign-off. No board
test was performed.

Verification artifacts are in `/tmp/norbert-gowin-area-verify/`: source hashes
in `source.sha256`, resource report in `impl/pnr/spi_flash.rpt.txt`, and timing
report in `impl/pnr/spi_flash_tr_content.html`. The complete mapped functional
suite also passed, alongside the full RTL suite and 18 rflasher integration
tests. Those zero-delay tests do not override routed timing failures.
