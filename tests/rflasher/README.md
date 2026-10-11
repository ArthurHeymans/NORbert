# rflasher / NORbert controller interoperability

Run with Verilator, a C++17 compiler, Make and Rust:

```sh
make test-rflasher
# External RTL, with separate Verilator output:
make -C tests/rflasher test-rflasher \
  NORBERT_RTL_ROOT=/path/to/NORbert BUILD=/absolute/build/path
```

`Cargo.lock` pins actual rflasher PR73 core/chips commit
`54c4a43981f68bd0ec498bb2471b107b3eb47266`. Initial Cargo execution needs
network access unless dependencies are cached. This project is its own
workspace; Cargo artifacts stay in ignored `tests/rflasher/target` by default.

## Real engines, modeled memory

`wrapper.v` instantiates real `spi_trx`, `spi_prefetch`, `glue`, `spi_program`,
`host_protocol` and `sdram_client_mux`. Configuration uses UART **byte-interface**
commands VERSION (protocol **9**), STOP, CHIPCONFIG, CHIPPROFILE (format **2**),
and START. There are no internal configuration pokes, mock SPI commands or
replacement protection implementations. Each test owns a separate bridge
process; tests run serially.

`bridge.cpp` models only external SDRAM: a 64MiB, initially-FF vector, explicit
request acceptance and 12-system-tick busy latency. SPI is mode 0, single-lane,
with 12 system ticks per half-SCK. This tests protocol/state behavior, not
physical timing margins. Independent memory inspection checks array effects.
Read chunks are capped at 512 bytes to keep pipe responses bounded.

The line-oriented bridge returns one hex line per request:

* `H hexbytes`: host input, returning host output bytes.
* `S hexbytes`: complete full-duplex SPI transaction.
* `B hexbytes` / `C`: transfer with CS held low / release CS, for STOP tests.
* `D microseconds`: advance system clocks; driver polling still reads real WIP.
* `M decimal-address hexbytes`: seed backing memory (setup only).
* `R decimal-address decimal-length`: inspect backing memory independently.

## Compatibility coverage

Fifteen focused tests cover:

* Actual JEDEC/SFDP driver probing and erase/program/readback/NOR RMW for
  Micron N25Q128/256/512, Macronix MX25L12833F/MX66L51235F, GigaDevice
  GD25Q128/GD25Q256D, ISSI IS25LP128/IS25LP512MG, Spansion S25FL128L and
  EON EN25Q64 and Atmel AT26DF041. High-bank/native-four-byte operations are
  checked against independent low-bank sentinels, including 64MiB capacity.
  AT26DF041 also uses its actual 256-byte `81` eraser and byte-only PP framing;
  its geometry takes priority over the Micron config opcode alias.
  Legacy `90`/`AB` framing and `9E` JEDEC alias are checked independently.
* Raw BP/SRWD/CMP/SRL, global/sector-lock and QE-off initialization attempts,
  followed by successful raw PP and actual driver array operations. Status
  writes are harmless shadows, protection indications read clear, sector/BPR
  reads return zero, and lock writes do not protect or mask the array.
* SST26 6/10/18-byte BPR framing and reads spanning former lock boundaries;
  Winbond eight-byte `4B` UID, SST25 real AAI and byte-only `02` framing.
* Micron config register shadows and `70` flag status (busy `00`, ready `80`);
  NVCR persistence across `66/99`, active-low four-byte mode and selected fast
  read latency, including VCR `0`/`F` default codes. Macronix `15` config/QE
  initialization, active-mode readback and six-clock fast reads; Spansion
  addressed `71`/`65` CR1/CR2 and small config shadows, WEL/WIP polling,
  24-clock fast read and high-bank real-driver mutation. Exact non-byte dummy
  intervals are checked by extracting bit-aligned data from captured SPI RX,
  without changing the byte-oriented bridge.
* `66/99` reset, incomplete PP address/data frames and page-wrap boundaries.
* STOP cancellation of unterminated PP/erase commands and stale page data;
  accepted programming drains before STOP ACK permits host RAM loading.
* Complete malformed profiles (including format, page size, geometry, opcode
  collision, polling-bit mask and final-byte rejection), partial-profile
  timeout and recovery.
* Real parsing and byte-exact readback of an uploaded BFPT, with end-of-table
  FF padding and no high-address aliases.
* Custom addressed `50` erasing all representative SST26 nonuniform regions,
  with both neighboring sentinels; actual C5/C8 and 17/16 EAR/bank helpers.

Standard mutation checks erase 4KiB, write 513 bytes at offset 7 across page
boundaries, read through SPI, perform a second NOR-AND write, and inspect
memory/sentinels and prefetch diagnostics. The independent profile helper
encodes exact pinned catalog geometry and framing hints, without an audited
protection whitelist or geometry truncation. It asserts power-of-two capacity
at most 64MiB, page size at most 256, descriptor/region limits, counts,
alignment and complete capacity coverage for every eraser.

This is a controller interoperability suite, **not chip-protection fidelity**
or a claim to test every controller. The bridge exercises single-lane SDR SPI;
RTL dual/quad lanes require separate coverage. QPI command-lane, DTR/octal,
NAND and DataFlash engines are out of scope. No protection enforcement or
restoration guarantee is provided.

## Optional host profile cross-check

Tests do not load stale format1 production profile fixtures. Set
`NORBERT_PROFILE_FIXTURES=/absolute/path` to independently generated **format2**
128-byte binary profiles named `<catalog-name>.bin` for every `configured()`
entry above plus S25FL256L, W25Q128.V, AT25DF321, SST25VF016B/032B and
SST26VF016B(A)/032B(A)/064B(A). The tests compare version, register alias,
BP/error hints, programming/ID framing, chip erase, unprotected initial flags,
page size and all geometry bytes, then configure RTL with the production packet.
Fixtures must request UID `NORbert8`. Unused capability/shadow defaults are not
compared. The W25Q128 BFPT fixture is used separately for SFDP parsing, never
as a profile-format shortcut.
