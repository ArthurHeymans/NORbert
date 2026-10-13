# Timing and clock-domain contracts

These are design requirements, not a hardware timing certificate. The RTL
and zero-delay mapped tests check functional ordering and deadlines, not
metastability, routing skew, external setup/hold, or signal integrity.

## Clocks

The 50 MHz oscillator drives a 1200 MHz PLL VCO. All three enabled outputs
have divider 10 and zero fine phase:

| Clock | Frequency | Coarse steps | Rising-edge offset from main |
|---|---:|---:|---:|
| Main/system (`CLKOUT0`) | 120 MHz | 0 | 0 ns |
| SDRAM pin (`CLKOUT1`) | 120 MHz | 9 | +3.750 ns / 162 degrees |
| DQ capture (`CLKOUT2`) | 120 MHz | 5 | +2.083333 ns / 90 degrees |

`tangprimer25k.sdc` declares these as related generated clocks. In particular,
aux-to-main transfers are **not asynchronous exceptions**. The SDRAM pin
clock is constrained at `O_sdram_clk`; its internal alias is optimized away
by Gowin. Keep constraints and `src/pll.v` defparams in sync.

The external SPI clock is unrelated and may stop indefinitely. The default
constraint is 40 MHz: the strict all-opcode/all-offset **simulation** envelope
for legacy/default framing. Profile-selected waits change the first-burst
budget and need their own per-mode margins; configured framing checks are
not a blanket 40 MHz guarantee for every latency setting. Faster modes in
the fast-read bench are exploratory, not hardware promises.
Mode-0 inputs are sampled on rising SCK and read outputs change on falling
SCK. CS directly frames transactions and releases output enables; it is not
debounced. Supply CS/SCK setup, hold, duty-cycle and reset-recovery margins
for the actual master. START/STOP and reconfiguration require a quiescent
master; configuration must not change during a transaction. STOP nevertheless
fails closed with held CS/stopped SCK: it cancels uncommitted mutations and
waits for accepted array work and page-buffer scrubbing before acknowledging.
That guard is not permission to START in the middle of an active master frame.

The nextpnr file is only a frequency-based placement target. That flow does
not model these PLL phase relationships or board I/O timing. Its independent
clock declarations must not be used as a phase-aware sign-off model.

## Crossings and ownership

| Crossing | Contract |
|---|---|
| SPI requests/address to `sdram` | Synchronized control with a bundled, held address/post selector. A new post identifies a new descriptor; the in-flight fill and next posted fill have independent ownership. The source must retain its payload through destination capture. |
| SPI bytes/command to `spi_program` | Two-stage control sampling; no logic reads the first stage. A byte is committed from the held offset/value once the second stage shows its strobe; they stay stable until the next completed SPI byte. The write command level, type/address/length and the partial-byte flag are held until completion clears WIP, so the engine may start long after later (status-only) transactions began. It starts only after the last byte of its transaction is committed and cannot preempt an outstanding host SDRAM operation. |
| `spi_program.done` to SPI WIP | Completion toggle synchronized into SCK and consumed even when deselected. With stopped SCK the toggle remains held until clocks resume; no pulse must survive an arbitrary stopped-clock interval. |
| SDRAM buffers/readiness to SPI output | Deliberately low-latency progressive data, **not a conventional fully handshaked async FIFO**. Readiness is tracked per beat and checked for each consumed sample. Both data and qualifying metadata need bounded routing and worst-phase analysis; fault detection alone does not establish CDC safety. |
| Logger CMD/ADDR/END to system | Three-stage event sampling with held payloads. Pulses must span a system sampling opportunity and events/payloads must not be overwritten before capture. Frames serialize in capture order; overflow drops new frames and reports a saturated event count. |
| Configuration/SFDP to SPI | Configure only while stopped, after the STOP drain acknowledgement. The 460-bit decoded control bundle and 32-bit completion epoch are held configuration, not a per-bit synchronized stream. Geometry is compiled into dual-clock erase RAM; its final system-clock write and complete validation precede publication. Keep controls and both RAMs unchanged through START and while running. The SPI side detects the epoch and reseeds before accepting a new command. SFDP and erase RAM read ports belong to SCK. START acknowledgement also waits for page-buffer initialization. |
| UART/FT245 flags to system | Scalar input synchronizers; FT245 data is sampled only after the RD setup interval. Output strobes and bus turnaround must meet the FT2232H timing specification plus actual board skew. |
| Host/program requests to `sdram` | Same clock domain. Hold command/address/write snapshot until `access_accept`; wait for access completion before issuing the next command. Ownership cannot switch across an outstanding request or open-row pair. |

The SDC bounds both SPI/system directions to one system period (8.333 ns)
and sets their minimum path delay to zero, replacing meaningless arbitrary
phase setup/hold checks **only for that unrelated clock pair**. It does not
false-path all CDC logic, suppress same-domain synchronizer settling paths,
or exempt related PLL clocks. This routing bound is necessary but not a
proof that every bundle/qualifier has adequate settling or skew margin.
Audit the actual destination-capture interval and tighten individual path
budgets if needed. Never replace these bounds with blanket asynchronous
clock groups merely to obtain a clean report.

## Erase-table lookup

Configuration verifies that region ends are 256-byte aligned. An addressed
single-lane erase therefore has its complete region-selection address before
the last eight address bits arrive, including EAR or native four-byte prefixes.
The SPI side then reads the five descriptor rows synchronously, using one
comparator and retaining only the first matching region's burst mask. The scan
uses six of the final eight SCK edges, leaving the mask stable before dispatch.
No free-running SCK is assumed: a pause holds the scan state; CS/reset aborts it.
This is an ordering budget, not a routed RAM-to-comparator timing guarantee.

## Refresh scheduling

At 120 MHz the nominal per-chip period is 468 clocks (a conservative
32 ms / 8192 schedule). Refresh events alternate between the chips, one
chip per event, so an event is due every 234 clocks and stalls dispatch
for a single tRC instead of a back-to-back pair. Deferral may reach twice
the per-chip period, plus bounded safe-row-closure/access service time.
Each event pays only one event period of elapsed time; it does not discard
overdue time. This preserves the nominal average rate even when SCK and CS
stall indefinitely. The saturating counter has service-latency headroom
above the dispatch deadline. Registered, increment-predicted deadline flags
remove arithmetic/comparison from the command/data dispatch critical path
without postponing the deadline. The refresh bench checks both maximum
gaps and sustained refresh rate.

SPI reads inhibit refresh from the first clock of every transaction until
the opcode turns out not to be an array read, so a refresh cannot start
late enough to delay the first burst. Throughout an array read the
controller only starts a refresh in a window: after it dispatched a
lookahead (continuation) READ and until the SPI side re-arms that post. The
next post is then at least the re-arm-to-post gap away, so a 9-clock
refresh event overlaps it by only part of that gap, well inside the burst's
margin. A refresh in the gap before a post would instead delay the post by
the whole event. Stopped SCK outside a window falls back to the deadline
(hard) refresh. `quad_fast_tb` sweeps the refresh due time across one full
event period for every documented read limit.

## Required external I/O profile

No universal SPI-master/board timing profile is available. The checked-in
SDC intentionally does **not** fabricate input/output delays. Before hardware
sign-off, add min **and** max constraints for:

- SPI IO0–IO3 input clock-to-data plus board data-vs-SCK flight-time skew,
  relative to the master's falling launch edge. Include CS framing and
  output-enable/release requirements separately.
- SPI IO0–IO3 output: master setup plus data-vs-SCK skew as maximum output
  delay, and negative master hold plus minimum skew as minimum output delay,
  relative to the following rising sample edge.
- SDRAM DQ input: datasheet clock-to-Q min/max plus clock-out and return-data
  board delays, relative to the generated SDRAM pin clock. Check capture at
  the intended following aux edge, not just equal clock frequencies.
- SDRAM address/bank/CS/RAS/CAS/WE/DQM and output DQ: SDRAM setup/hold and
  board clock/data skew, relative to the SDRAM pin clock.
- FT245 data, RD/WR and bus turnaround: FT2232H access/setup/hold limits plus
  board delay. Its handshake-derived sampling is not a synchronous external
  clock interface; constrain that access window explicitly.

Example syntax (replace symbols with measured/datasheet-derived numbers;
this is a template, not a usable default profile):

```tcl
set_input_delay -clock spi_clk -clock_fall -max SPI_TCO_MAX_PLUS_SKEW [get_ports {spi_mosi_pin spi_miso_pin spi_io2_pin spi_io3_pin}]
set_input_delay -clock spi_clk -clock_fall -min SPI_TCO_MIN_PLUS_SKEW [get_ports {spi_mosi_pin spi_miso_pin spi_io2_pin spi_io3_pin}]
set_output_delay -clock spi_clk -max SPI_SETUP_PLUS_SKEW [get_ports {spi_mosi_pin spi_miso_pin spi_io2_pin spi_io3_pin}]
set_output_delay -clock spi_clk -min NEG_SPI_HOLD_PLUS_SKEW [get_ports {spi_mosi_pin spi_miso_pin spi_io2_pin spi_io3_pin}]
set_input_delay -clock sdram_clk -max SDRAM_TCO_MAX_PLUS_FLIGHT [get_ports {IO_sdram_dq[*]}]
set_input_delay -clock sdram_clk -min SDRAM_TCO_MIN_PLUS_FLIGHT [get_ports {IO_sdram_dq[*]}]
```

## Validation and remaining sign-off

### Rebased protocol9 controller compatibility

The PR integrates the upstream refresh/row interlocks, UART buffering, busy
and partial-command handling, continuous reads and power-detect bypass from
`feb4f6a5`. Continuous reads retain their accepted address/latency framing;
Micron profile BB/EB dummy clocks are not interpreted as mode bytes. The
refresh classifier shares profile-aware read admission with the decoder,
and every issued mutation initializes its partial-byte flag.

Gowin Education 1.9.11.03 placed/routed the source-hashed snapshot
`/tmp/norbert-gowin-pr-rebase/` with unchanged constraints:

| Clock | Required | Reported Fmax | Same-domain setup-violated endpoints |
|---|---:|---:|---:|
| SPI | 40 MHz | 68.799 MHz | 0 |
| System | 120 MHz | 121.281 MHz | 0 |

Same-clock setup/hold TNS and violated-endpoint counts are zero, but system
margin is still only approximately **0.088 ns**. Overall STA reports **210
setup and 79 hold violations on inter-clock paths**. The worst reported
setup path is configuration epoch to SPI address state: 11.635 ns data delay
and −2.139 ns slack. **Full timing sign-off still fails.** Configuration CDC
snapshotting, progressive-data crossing analysis, PR1014 clock routing and
board/master I/O constraints remain follow-up work; no clocks or timing
exceptions were relaxed and no board was tested.

Native resource and timing reports are `impl/pnr/spi_flash.rpt.txt` and
`impl/pnr/spi_flash_tr_content.html` under that snapshot. Its `source.sha256`
matches all live RTL, headers, build script and constraints. Yosys sequential
equivalence proves **380/380 matched SDRAM signals** against upstream
`feb4f6a5` at 120 MHz / burst length4, including the new row/refresh interlocks.
The proof uses `equiv_simple` and `equiv_induct`; artifacts are
`/tmp/norbert-pr-integration/sdram-equiv.ys` and `sdram-equiv.log`. This is
RTL equivalence, not proof of physical PLL or I/O margins. Strict lint and
the complete RTL suite pass, including upstream host/SDRAM/output-enable
checks, partial-abort-to-valid-erase cases and profile-configured continuous
reads. All 63 host tests, native Clippy, WASM checks and 15 real-rflasher tests
using production-generated profiles also pass. Earlier timing and equivalence
measurements below do not certify this rebased RTL.

### Protocol9 local system-timing fixes before upstream integration (historical)

The system-clock repair changes only four RTL modules, without changing
protocol9/profile2, clock constraints, SPI framing or mutation ownership:

- Logger events and payloads are captured on their original edge, then queued
  one cycle later. Registered event/loss presence drives the wide FIFO enable.
- SDRAM predicts `cmdcount < cmdtarget` into a register whenever either changes.
  Command dispatch, write beats and refresh cycles remain unchanged.
- Host descriptor validation selects constant slices, avoiding the dynamic
  three-bit count index that inferred a multiplier on the publication path.
- Geometry validation registers individual checks before reducing them into
  `bad`; capacity is sampled at profile start. RAM writes retain their original
  stage, and completion waits for the final validation reduction as well.

Gowin Education 1.9.11.03 placed/routed the source-hashed snapshot
`/tmp/norbert-gowin-timing-p0/` with the unchanged SDC:

| Clock | Required | Reported Fmax | Same-domain setup-violated endpoints |
|---|---:|---:|---:|
| SPI | 40 MHz | 65.383 MHz | 0 |
| System | 120 MHz | 121.239 MHz | 0 |

The report's same-clock setup/hold TNS and violated-endpoint counts are all
zero. System margin is only approximately **0.085 ns**, below the desired
0.3 ns engineering margin; placement changes can erase it. Overall STA still
reports **141 setup and 122 hold violations on inter-clock paths**. The worst
reported setup path is from a compiled erase opcode to SPI output control:
11.776 ns data delay and −2.532 ns slack under the system-to-SPI bound.
These remaining crossings are not waived; **full timing sign-off still fails**.
Configuration snapshotting, progressive-data CDC auditing, PR1014 clock routing,
and external board/master delay constraints remain outstanding. No clocks or
exceptions were relaxed, and no board was tested.

Strict lint, the complete behavioral and mapped suites, and all 15
real-rflasher tests using production-generated profiles pass. Independent source review found no
blockers. Yosys sequential equivalence proves all 377 matched SDRAM signals against revision `40c88ded`
at 120 MHz / burst length4 (`equiv_simple` followed by `equiv_induct`), including
DQ driver registers, command timing and read/refresh ownership. That proof
compares digital RTL; it does not validate physical PLL phases or I/O margins.
Verification artifacts are `/tmp/norbert-timing-p0/sdram-equiv.ys` and
`sdram-equiv.log`; the baseline is retrieved with
`jj file show -r 40c88ded src/sdram.v`. Native resource/STA reports remain
`impl/pnr/spi_flash.rpt.txt` and `spi_flash_tr_content.html` under the snapshot.

### Protocol9 before local system-timing fixes (historical)

The format2/protocol9 redesign uses a BRAM opcode ROM, a shared command engine
and compiled geometry, without protection maps or falling-edge read-lock
qualification. Gowin Education 1.9.11.03 generated a placed/routed bitstream
with unchanged constraints from `/tmp/norbert-gowin-compat-verify/`.
All RTL/constraint hashes matched the tested snapshot afterward.

| Clock | Required | Reported Fmax | Same-domain setup-violated endpoints |
|---|---:|---:|---:|
| SPI | 40 MHz | 65.517 MHz | 0 |
| System | 120 MHz | 105.104 MHz | 128 |

SPI same-domain timing now passes the declared requirement, but **system
same-domain timing still fails**. Overall STA reports 351 setup-violated and
124 hold-violated endpoints across 19,458 paths, including constrained
crossings. PLL offsets remain 0, 3.750 and 2.083 ns. Resource fit/bitstream
creation is not timing sign-off; the design is not a timing-closed release.

The three-stage stopped-state geometry compiler and serial descriptor
validation reduce control depth, but do not establish 120 MHz closure.
Remaining system paths and the quiescent configuration CDC contract need
explicit inspection. No declared clock was lowered and no crossing blanket
false path was added. Generic PMOD clock routing warning PR1014 remains;
external input/output delays still require a board/master profile.

Native evidence is in `impl/pnr/spi_flash_tr_content.html` and
`impl/pnr/spi_flash.rpt.txt` under the source-hashed snapshot. Functional RTL,
mapped-ROM tests and real rflasher operations check protocol/data behavior,
not routed margins. No board test was performed; USB enumeration contained
no FPGA/FTDI device. See [utilization](rtl-utilization.md) for resource results.

### Historical optimized protocol8 chip-profile design

Gowin Education 1.9.11.03 successfully synthesized and placed/routed revision
`f16dccc1` with the checked-in constraints, but **internal timing fails**:

| Clock | Required | Reported Fmax | Same-domain setup-violated endpoints |
|---|---:|---:|---:|
| SPI | 40 MHz | 35.377 MHz | 7 |
| System | 120 MHz | 77.453 MHz | 186 |

The report confirms the intended 120 MHz PLL clocks and offsets of 0, 3.750
and 2.083 ns. Across all analyzed paths it reports 831 setup-violated and
131 hold-violated endpoints, including the explicitly constrained crossings.
The worst reported setup path is from
`glue_i/host_i/cfg_chip_erase_bursts_9_s0/Q` to
`spi_trx_i/write_addr_17_s0/CE`: 27.189 ns data delay, −17.960 ns slack under
the 8.333 ns system-to-SPI bound. Worst reported hold slack is −0.673 ns,
from the compiled initial-status header to an SPI status-register bit.

The configuration bundle is held while running; its crossing paths need an
explicit quiescent-configuration timing audit. That does not excuse the
independent same-domain failures. Do not lower declared clocks or blanket
false-path the crossings to label this build closed. Protection decoding also
qualifies falling-edge SPI output paths, and external paths still lack a
board/master delay profile. PR1014 still warns about generic routing of the
PMOD SPI clock.

Reports and the generated bitstream are under
`/tmp/norbert-gowin-area-verify/impl/pnr/`; `spi_flash_tr_content.html` contains
the STA details and `spi_flash.rpt.txt` the resource usage. Source and constraint
hashes match the tested snapshot. Full RTL/mapped functional suites and all
18 rflasher integration tests pass, but **this is not a timing-closed release**.
No board was tested; no FPGA/FTDI device was present in the USB enumeration.

### Historical pre-profile design

The following results apply only to the smaller pre-profile implementation.
Gowin Education 1.9.11.03 successfully parsed and placed/routed the pre-profile
design with the generated clocks and inter-clock bounds. Its clock report confirmed
120 MHz offsets of 0, 3.750 and 2.083 ns; the 40 MHz SPI clock and 120 MHz
system clock had no negative reported setup/hold slack in the final build
(internal reported Fmax: SPI 74.356 MHz, system 127.979 MHz). Those are STA
results for the declared coverage, not usable pin-frequency guarantees.
**External paths remain unconstrained**, so those results are not full timing
closure. Gowin also reports generic routing for the PMOD SPI clock (PR1014):
inspect clock insertion delay/skew and minimum SCK pulse width explicitly.

For release: supply the external profile, audit constraint endpoint coverage
and exception application, examine related-clock and CDC path/skew reports,
and re-run the SDRAM phase sweep on the actual board after placement changes.
Exercise worst SPI phase/duty cycle, stopped clocks, simultaneous host/target
traffic, long FT245 stalls and bus release on hardware. Previous PLL sweep
results are not a certificate for a newly placed netlist.
