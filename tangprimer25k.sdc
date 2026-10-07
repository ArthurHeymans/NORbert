// Gowin clock/CDC constraints. See docs/rtl-timing.md before signing off.
// PLL: 50 MHz -> 120 MHz, VCO 1200 MHz, coarse phase step 0.416667 ns.
// CLKOUT0: 0 steps; CLKOUT1: 9 steps (3.75 ns); CLKOUT2: 5 steps (2.083333 ns).
create_clock -name osc_clk -period 20.000 [get_ports {clk_50mhz}]
create_generated_clock -name sys_clk -source [get_ports {clk_50mhz}] -multiply_by 12 -divide_by 5 [get_nets {clk_133}]
create_generated_clock -name sdram_clk -source [get_ports {clk_50mhz}] -multiply_by 12 -divide_by 5 -phase 162 [get_ports {O_sdram_clk}]
create_generated_clock -name aux_clk -source [get_ports {clk_50mhz}] -multiply_by 12 -divide_by 5 -phase 90 [get_nets {aux_clk}]

// All-command functional test envelope; faster modes are exploratory.
create_clock -name spi_clk -period 25.000 [get_ports {spi_clk_pin}]

// SPI and system clocks are unrelated. Bound their crossing datapaths
// explicitly instead of false-pathing whole domains (which would hide
// progressive buffers and bundled addresses). Override arbitrary phase
// setup/hold analysis only for this pair; keep same-domain synchronizer
// settling paths and all related PLL-domain paths timed normally.
set_max_delay -from [get_clocks {spi_clk}] -to [get_clocks {sys_clk}] 8.333
set_min_delay -from [get_clocks {spi_clk}] -to [get_clocks {sys_clk}] 0.000
set_max_delay -from [get_clocks {sys_clk}] -to [get_clocks {spi_clk}] 8.333
set_min_delay -from [get_clocks {sys_clk}] -to [get_clocks {spi_clk}] 0.000

// External setup/hold and board flight times are not known universally.
// Add BOTH min and max input/output delays from a measured board/master
// profile using docs/rtl-timing.md; these clock constraints alone are NOT
// complete external timing closure. No blanket asynchronous clock groups.
