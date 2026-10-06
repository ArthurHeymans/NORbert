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

The logger queue also adds four `SDPX9B` BSRAM cells (the original four
`DPX9B` cells remain). It trades those blocks and LUTs for capture-order
preservation, atomic event payloads, and explicit loss accounting.
