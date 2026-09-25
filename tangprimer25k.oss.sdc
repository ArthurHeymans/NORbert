# Clock constraints for the open-source flow (nextpnr-himbaechel).
#
# nextpnr neither keeps constraints on top-level ports through IO packing
# nor derives PLL output clocks, so constrain the internal nets from top.v.
# It also has no phase relationship between the PLL outputs, and no IO
# timing, so the SDRAM capture and SPI pin timing are not analysed.
create_clock -name clk -period 8.333 [get_nets {clk}]
create_clock -name aux_clk -period 8.333 [get_nets {aux_clk}]
create_clock -name sdram_clk -period 8.333 [get_nets {clk_133_sdram}]
create_clock -name spi_clk -period 33.333 [get_nets {spi_clk_in}]
