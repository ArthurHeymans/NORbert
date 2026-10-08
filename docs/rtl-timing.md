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
constraint is 40 MHz: the strict all-opcode/all-offset **simulation** envelope.
Faster modes in the fast-read bench are exploratory, not hardware promises.
Mode-0 inputs are sampled on rising SCK and read outputs change on falling
SCK. CS directly frames transactions and releases output enables; it is not
debounced. Supply CS/SCK setup, hold, duty-cycle and reset-recovery margins
for the actual master. START/STOP and reconfiguration require a quiescent
master; configuration must not change during a transaction.

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
| Configuration/SFDP to SPI | Configure only while stopped; the log-only mode flag follows the same rule (SNIFFCTL is refused while running). The dual-clock SFDP RAM read port belongs to SCK; table/configuration values stay immutable while emulation runs. START acknowledgement also waits for page-buffer initialization. |
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

## Refresh scheduling

At 120 MHz the nominal period is 468 clocks (a conservative 32 ms / 8192
schedule for each chip). Deferral may reach twice that period, plus bounded
safe-row-closure/access service time. Each refresh pair pays only one period
of elapsed time; it does not discard overdue time. This preserves the nominal
average rate even when SCK and CS stall indefinitely. The saturating counter
has service-latency headroom above the dispatch deadline. Registered,
increment-predicted deadline flags remove arithmetic/comparison from the
command/data dispatch critical path without postponing the deadline.
The refresh bench checks both maximum gaps and sustained refresh rate.

SPI reads inhibit refresh from the first clock of every transaction until
the opcode turns out not to be an array read, so a refresh pair cannot
start late enough to delay the first burst. Throughout an array read the
controller only starts a refresh in a window: after it dispatched a
lookahead (continuation) READ and until the SPI side re-arms that post. The
next post is then at least the re-arm-to-post gap away, so a 17-clock
refresh pair overlaps it by only part of that gap, well inside the burst's
margin. A refresh in the gap before a post would instead delay the post by
the whole pair. Stopped SCK outside a window falls back to the deadline
(hard) refresh. `quad_fast_tb` sweeps the refresh due time across one full
period for every documented read limit.

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

Gowin Education 1.9.11.03 successfully parsed and placed/routed this design
with the generated clocks and inter-clock bounds. Its clock report confirmed
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
