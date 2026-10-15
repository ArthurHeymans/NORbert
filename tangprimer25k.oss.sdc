# Clock constraints for the open-source flow (nextpnr-himbaechel).
#
# The pinned nextpnr preserves input-port clocks through IO packing.
# Bind SPI by its input port, not the optimized-away spi_clk_in alias.
# PLL outputs are explicit frequency targets; there is no phase STA or IO
# timing, so the SDRAM capture and SPI pin timing are not analysed.
# This is a frequency-only placement target, NOT the Gowin sign-off model;
# never copy these independent clocks into a phase-aware STA flow.
# Ideal PLL offsets are SDRAM +7.5 ns, aux +4.166667 ns from clk;
# these are not measured board/pin arrival times.
# See docs/rtl-timing.md for CDC contracts and missing board I/O budgets.
create_clock -name clk -period 8.333 [get_nets {clk}]
create_clock -name aux_clk -period 8.333 [get_nets {aux_clk}]
create_clock -name sdram_clk -period 8.333 [get_nets {clk_133_sdram}]
create_clock -name spi_clk -period 14.286 [get_ports {spi_clk_pin}]
