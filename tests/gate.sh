#!/usr/bin/env bash
# Gate-level tests: run the behavioural testbenches against the netlist
# Yosys produces for GW5A (synth_gowin -family gw5a, as in the open-source
# build) instead of the RTL. This catches RTL that Yosys maps differently
# from Gowin synthesis. Zero-delay: no place-and-route or timing.
#
# uart_tb (two parameterisations of one module) and ft245_tb (peeks FSM
# state that synthesis re-encodes) stay RTL-only.
set -euo pipefail
cd "$(dirname "$0")/.."

build=$(mktemp -d)
trap 'rm -rf "$build"' EXIT

RTL="src/spi_trx.v src/spi_prefetch.v src/sdram.v src/glue.v src/uart.v"
RTL="$RTL src/ft245.v src/fifo.v src/logger.v src/util.v"

# Yosys' IOBUF model assigns IO to its input I instead of its output O.
sed 's/^  assign I = IO;/  assign O = IO;/' \
    "$(yosys-config --datdir)/gowin/cells_sim.v" >"$build/cells_sim.v"

# synth <tb> <module> [keep wires] [chparam args]
# Wires the testbench peeks at are kept so dead-register removal cannot
# hide them.
synth() {
    local tb=$1 mod=$2 keep=${3:-} chparam=${4:-} pads=-noiopads extra=""
    if [ "$mod" = top ]; then pads=""; extra="src/top.v tests/pll_stub.v"; fi
    [ -z "$chparam" ] || chparam="chparam $chparam $mod;"
    [ -z "$keep" ] || keep="setattr -set keep 1 $(printf 'w:%s ' $keep);"
    # Yosys treats newlines in -p as command separators.
    yosys -q -l "$build/$tb.$mod.log" -p "read_verilog -Isrc $RTL $extra; \
        $chparam hierarchy -top $mod; $keep \
        synth_gowin -family gw5a -nolutram -noflatten $pads -top $mod; \
        write_verilog -noattr $build/$tb.$mod.v" >/dev/null
}

sim() {
    local tb=$1; shift
    echo "Gate-level testing $tb"
    verilator --binary --timing -j 2 --top-module "${tb}_tb" -Isrc \
        -Wno-fatal -Wno-lint -Wno-style -Wno-PINNOTFOUND -Wno-TIMESCALEMOD \
        --Mdir "$build/$tb" "tests/${tb}_tb.sv" "$@" \
        tests/gw5a_cells_sim.v "$build/cells_sim.v" >"$build/$tb.log" 2>&1 \
        || { cat "$build/$tb.log"; exit 1; }
    "$build/$tb/V${tb}_tb"
}

synth fifo fifo "" "-set WIDTH 8 -set NUM 16 -set FREESPACE 1"
sim fifo "$build/fifo.fifo.v"

synth logger logger
sim logger "$build/logger.logger.v"

synth spi_flash glue
synth spi_flash spi_trx
sim spi_flash "$build/spi_flash.glue.v" "$build/spi_flash.spi_trx.v"

synth sdram_controller glue
synth sdram_controller sdram
sim sdram_controller "$build/sdram_controller.glue.v" "$build/sdram_controller.sdram.v"

synth quad_fast sdram "" "-set CLK_FREQ_MHZ 120 -set BURST_LEN 4"
synth quad_fast spi_trx
sim quad_fast "$build/quad_fast.sdram.v" "$build/quad_fast.spi_trx.v"

synth toctou top "spi_addr_latched refreshcount spi_cmd_read_ack trap_triggered
                  log_addr_sync log_addr_valid_sync"
sim toctou "$build/toctou.top.v"
