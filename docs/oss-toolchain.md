# Experimental open-source Gowin toolchain

NORbert's flake locks the public `norbert-experimental` branches in these forks:

- [Yosys](https://github.com/ArthurHeymans/yosys/tree/norbert-experimental)
- [nextpnr](https://github.com/ArthurHeymans/nextpnr/tree/norbert-experimental)
- [Apicula](https://github.com/ArthurHeymans/apicula/tree/norbert-experimental)

`flake.lock` pins immutable revisions, including Yosys/nextpnr submodules. The
fork branches may advance, but an existing lock file does not follow them.
The packaged nextpnr and Apicula device database cover **GW5A-25A only**. Yosys
uses the Verilog frontend; Slang is disabled in this package configuration.
The `oss` development shell omits Gowin's proprietary tools and download.
The default shell keeps both flows available.

```sh
nix build .#yosys .#nextpnr .#apicula
nix build .#checks.x86_64-linux.oss-toolchain
nix develop .#oss --command make build-oss
```

The focused check runs Apicula's software/database tests, the Yosys optimization
and tristate tests, direct IOBUF simulation, synthesized bidirectional-pad
simulation, and BRAM simulations for gw1n/gw2a/gw5a. nextpnr's package enables its
native tests, including the new PLL tests. This is not a full Yosys regression
suite or hardware/timing sign-off.

To deliberately update the toolchain pins:

```sh
nix flake update yosys-src nextpnr-src apicula-src
```

## Topic branches

Integration merges are for NORbert, not upstream PRs. The smaller topic branches
remain separate and are based on fetched upstream history. Prepare submissions
from these branches, never from `norbert-experimental`.

### Apicula

All branches are in [ArthurHeymans/apicula](https://github.com/ArthurHeymans/apicula).

| Branch | Scope / dependency |
| --- | --- |
| `iologico-empty-fix` | Missing attribute conversion; focused software test. |
| `ram16sdp-init-default` | Missing INIT parameters default to zero; explicit INIT remains unchanged. |
| `gw5a-ssram` | GW5A-25A shadow SRAM; includes `ram16sdp-init-default`. The new mode-code lookup is GW5A-specific. |
| `gw5a-io-registers` | IO register fuses and CE ports; includes `iologico-empty-fix`. Requires nextpnr's matching IO-register branch and rebuilt databases. |
| `gw5a-io-defaults` | LVCMOS33 defaults only; other standards retain the old defaults and explicit attributes take precedence. |

The integration branch alone includes the cached GW5A database. See its
[provenance note](https://github.com/ArthurHeymans/apicula/blob/norbert-experimental/doc/norbert-database.md).
Its capabilities and SHA-256 are checked, but reproducible regeneration remains
outstanding: the available Gowin 1.9.11.03 FSE file is incompatible with the
current parser, while Apicula specifies 1.9.10.03. Do not claim this artifact was
freshly regenerated or that its original vendor-input version was re-established.

### nextpnr

All branches are in [ArthurHeymans/nextpnr](https://github.com/ArthurHeymans/nextpnr).

| Branch | Scope / dependency |
| --- | --- |
| `gowin-port-clock-constraints` | Preserve an input-port constraint across buffer removal. |
| `gowin-pll-clock-constraints` | Includes the port-constraint fix; static integer-divider inference, existing constraints preserved, bypass waveforms copied. |
| `gowin-ssram-error` | Actionable diagnostic when a device has no shadow SRAM. |
| `gowin-i2c-pin-warning` | Warn about IO on unreleased configuration I2C pins. |
| `gowin-5a-ioreg` | GW5A register ports/feature flag; tristate FFs remain in fabric. Requires Apicula's IO-register branch/database. |

PLL inference declines dynamic dividers/duty adjustment, custom rPLL duty cycles,
fractional PLLA operation, spread spectrum and active PLLA management clocks.
It does not establish phase relationships or provide generated-clock timing
sign-off. Native tests cover dividers, explicit constraints, bypass waveforms,
unconstrained inputs and unsupported configurations.

### Yosys

All branches are in [ArthurHeymans/yosys](https://github.com/ArthurHeymans/yosys).

| Branch | Scope / dependency |
| --- | --- |
| `gowin-iobuf-sim` | Correct the IOBUF output assignment; direct primitive and synthesized-pad tests. |
| `gowin-remove-buffers` | Lower internal `$buf` cells using `simplemap` before export; otherwise nextpnr cannot place some netlists produced by current upstream Yosys. |
| `ast-debug-output` | Independent removal of the stray `make 1` log. |
| `ast-strided-writes` | Use the existing case method for non-overlapping strided writes. |
| `memory-map-const-wr` | Reorder constant-data writes where write priorities permit. |
| `opt-dff-shared-mux` | Private shared-mux feedback; bounded graph/path exploration and an external-fanout equivalence case. |
| `tribuf-nested` | Includes the IOBUF fix; preserve resolved pad reads when folding internal tristates. |
| `gowin-bram-sim` | Behavioural models, family port lists, mixed-width/byte-enable tests and output pipeline controls. |
| `gowin-wide-lut-costs` | Cost LUT5–LUT8 according to their LUT4 usage. |
| `gowin-comparator-mapping` | Includes wide-LUT costs; use the existing comparator mapping helpers. |
| `gowin-lut-mapping` | Includes both preceding mapping topics; experimental LUT4/area-oriented default. |

The small IOBUF, buffer-export, missing-attribute, INIT-default and clock-transfer
fixes are the first candidates for upstream discussion. Buffer lowering is limited
to `$buf` cells; normalizing the whole mapped design disconnects bidirectional
pad aliases and is not used. Pad-read tests supply a defined external level when
the design reads a released pad, rather than asserting a logic value for a
physically floating input. The mapping-default change is retained
for experiments, not presented as a generally superior upstream default.

## Remaining acceptance work

- Establish a reproducible, versioned database-generation input before submitting
  hardware support; validate the IO-register types and A/B pin combinations on
  hardware and compare older-family generated databases.
- Benchmark mapping changes across designs/families/widths and nextpnr seeds.
  NORbert alone is insufficient evidence for a new synthesis default.
- Expand signed/out-of-range strided-write, mixed-write-priority and shared-mux
  budget-fallback coverage before requesting core-pass acceptance.
- Validate additional BRAM modes against vendor models, including collisions and
  asymmetric ports. Output pipeline CE/OCE/reset behaviour was checked against
  the available vendor models for all three families; this does not establish
  complete primitive equivalence.
- General partial-width nested tristates remain outside the folding subset.

[YosysHQ's interim LLM policy](https://blog.yosyshq.com/p/interim-yosyshq-llm-policy/)
applies before any submission. These branches were prepared with AI assistance;
publication in a personal fork is not a claim of policy compliance or upstream
acceptance. Review implementation provenance, own and explain each change, and
write submission text/comments yourself. For LLM-generated bugfixes, the policy
prefers minimal failing-test issues or an explicitly agreed exception. No upstream
PRs or issue comments are created by this preparation.

The README's earlier UART hardware results remain historical evidence, not
validation of every new revision. Releases continue to use the Gowin flow.
