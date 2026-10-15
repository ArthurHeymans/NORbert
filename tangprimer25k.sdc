// Gowin clock/CDC constraints. See docs/rtl-timing.md before signing off.
// PLL: 50 MHz -> 120 MHz, VCO 1200 MHz, coarse phase step 0.416667 ns.
// CLKOUT0: 0 steps; CLKOUT1: 9 steps (3.75 ns); CLKOUT2: 5 steps (2.083333 ns).
create_clock -name osc_clk -period 20.000 [get_ports {clk_50mhz}]
create_generated_clock -name sys_clk -source [get_ports {clk_50mhz}] -multiply_by 12 -divide_by 5 [get_nets {clk_133}]
create_generated_clock -name sdram_clk -source [get_ports {clk_50mhz}] -multiply_by 12 -divide_by 5 -phase 162 [get_ports {O_sdram_clk}]
create_generated_clock -name aux_clk -source [get_ports {clk_50mhz}] -multiply_by 12 -divide_by 5 -phase 90 [get_nets {aux_clk}]

// Fastest documented read mode (70 MHz, see README). 0x03 is limited to
// 40 MHz by its first-burst budget, not by this clock domain.
create_clock -name spi_clk -period 14.286 [get_ports {spi_clk_pin}]

// SPI and system clocks are unrelated. Bound their crossing datapaths
// explicitly instead of false-pathing whole domains (which would hide
// progressive buffers and bundled addresses). Override arbitrary phase
// setup/hold analysis only for this pair; keep same-domain synchronizer
// settling paths and all related PLL-domain paths timed normally.
set_max_delay -from [get_clocks {spi_clk}] -to [get_clocks {sys_clk}] 8.333
set_min_delay -from [get_clocks {spi_clk}] -to [get_clocks {sys_clk}] 0.000
set_max_delay -from [get_clocks {sys_clk}] -to [get_clocks {spi_clk}] 8.333
set_min_delay -from [get_clocks {sys_clk}] -to [get_clocks {spi_clk}] 0.000

// Exceptions, each backed by a contract in docs/rtl-timing.md. Only paths
// into the SPI clock are released here; system-side paths stay timed.
// Configuration is written only while emulation is stopped and held while
// it runs, so it is static whenever the SPI side samples it.
set_false_path -from [get_regs {glue_i/host_i/cfg_*}] -to [get_clocks {spi_clk}]
set_false_path -from [get_regs {glue_i/host_i/profile_store/*}] -to [get_clocks {spi_clk}]
set_false_path -from [get_regs {glue_i/host_i/sfdp_*}] -to [get_clocks {spi_clk}]
// Run control: START and STOP require a quiescent master, so the SPI reset
// and its terms only change while no SCK edge can use them.
set_false_path -from [get_regs {spi_reset*}] -to [get_clocks {spi_clk}]
set_false_path -from [get_regs {glue_i/host_i/spi_run_requested*}] -to [get_clocks {spi_clk}]
set_false_path -from [get_regs {glue_i/program_i/init_done*}] -to [get_clocks {spi_clk}]
// The address payload is consumed only on its synchronized event, two
// system clocks after the event's first stage captured it.
set_max_delay -from [get_regs {spi_trx_i/log_addr_out*}] -to [get_clocks {sys_clk}] 16.666
// Hold has no meaning into the first stage of a synchronizer, nor into the
// sticky first-beat margin diagnostic, which samples readiness asynchronously.
set_false_path -hold -to [get_regs {spi_trx_i/write_done_sync_0*}]
set_false_path -hold -to [get_regs {spi_trx_i/prefetch/prefetch_thin*}]

// External setup/hold and board flight times are not known universally.
// Add BOTH min and max input/output delays from a measured board/master
// profile using docs/rtl-timing.md; these clock constraints alone are NOT
// complete external timing closure. No blanket asynchronous clock groups.
