# RTL correctness changes: synthesis utilization

Counts are the sum of mapped `LUT1`–`LUT4` cells in the flattened top-level
netlist, measured with the flake's Yosys 0.69+norbert and:

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

Final total: **3627 LUTs**, **+493** versus the 3134-LUT baseline. Most of
the last block is the 16-entry UART receive FIFO, which `-nolutram` maps to
flip-flops and LUT multiplexers, and the continuous-read decode.

The logger queue also adds four `SDPX9B` BSRAM cells (the original four
`DPX9B` cells remain). It trades those blocks and LUTs for capture-order
preservation, atomic event payloads, and explicit loss accounting.
