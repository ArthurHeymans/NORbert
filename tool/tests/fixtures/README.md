# Independent compatibility wire fixtures

These 128-byte format-2 profiles independently encode the controller-compatible
wire contract documented in `tool/CHIP_PROFILES.md`. Geometry records retain
SPI transaction opcodes, size exponents and little-endian 24-bit region counts;
they do not describe enforced protection.

- `w25q64-fv.profile`: catalog `W25Q64BV/W25Q64CV/W25Q64FV`, 8 MiB.
- `w25q128-v.profile`: W25Q128.V, 16 MiB, known BP indication mask 1C and
  optional SR2 QE metadata. All common commands accepted, no QE gating.
- `at25df321.profile`: 4 MiB, 3C Atmel register alias, no sector protection.
- `sst26vf032b.profile`: 4 MiB, zero-valued BPR reads, five D8 boot regions.
- `sst25vf016b.profile` / `sst25vf032b.profile`: 2/4 MiB, AAI and byte-only02.

Every profile defaults to SR1/SR2/SR3=0, UID=0, unprotected flags/range/locks=0.
All `*.profile.hex` files contain 128 two-digit bytes, one per line, compatible
with `$readmemh`; matching JSON records identify the exact catalog name, JEDEC
bytes and capacity. SST26 hex fixtures cover all three BPR response widths.
Binary fixtures and hex fixtures agree byte-for-byte where both exist. Header
bytes were independently re-encoded for format2; original geometry stayed intact.

`w25q128-v.sfdp` independently encodes the JESD216B BFPT contract. Field positions
follow Linux `sfdp_bfpt_reads`: 1-1-2 uses DW1 bit16 and DW4 low half; 1-2-2
uses DW1 bit20 and DW4 high half. QER is zero in DW15[22:20] because quad is
ungated; addressing is DW1[18:17], EN4B/EX4B DW16. Mode/dummy clocks and page
size describe the emulator, never QPI/DTR/octal or a physical-chip SFDP capture.
