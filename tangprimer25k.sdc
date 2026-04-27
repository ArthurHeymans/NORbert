// Timing constraints for Tang Primer 25K SPI Flash Emulator
//
// spi_clk_pin is an external input clock from the SPI master (max 30 MHz).
// Constraining it here enables the PnR tool to perform timing analysis on
// all paths in the spi_clk domain and optimize routing accordingly.
create_clock -name spi_clk -period 33.333 [get_ports {spi_clk_pin}]
