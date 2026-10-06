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
