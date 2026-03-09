# Gowin CLI Synthesis Script for Tang Primer 25K SPI Flash Emulator
# Usage: gw_sh build.tcl

# Device: GW5A-LV25MG121NC1/I0 (Tang Primer 25K, MBGA121N package)
# GW5A-25B variant includes BSRAM
set_device -name GW5A-25B GW5A-LV25MG121NC1/I0

# Add Verilog source files
add_file src/top.v
add_file src/spi_trx.v
add_file src/sdram.v
add_file src/glue.v
add_file src/uart.v
add_file src/fifo.v
add_file src/util.v
add_file src/pll.v

# Add constraints
add_file tangprimer25k.cst

# Set top module
set_option -top_module top

# Synthesis options
set_option -verilog_std v2001
set_option -use_sspi_as_gpio 1
set_option -use_mspi_as_gpio 1
set_option -use_ready_as_gpio 1
set_option -use_done_as_gpio 1
set_option -print_all_synthesis_warning 1

# Place and route options
set_option -place_option 1
set_option -route_option 1

# Output directory
set_option -output_base_name spi_flash

# Run synthesis
run syn

# Run place and route
run pnr
