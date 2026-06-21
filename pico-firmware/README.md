# NORbert Pico firmware

Embassy/Rust firmware for a Raspberry Pi Pico bridge that speaks a native
postcard-framed RPC protocol over both:

- USB vendor bulk endpoints, with WinUSB descriptors for Windows hosts
- TCP port `24500` over a W5500 using `embassy-net-wiznet`

The Pico then presents an FT245-style asynchronous FIFO bus to the existing
NORbert FPGA gateware, so the FPGA byte protocol remains unchanged.

## Layout

```text
pico-firmware/
  protocol/   no_std postcard wire types shared with host tooling
  firmware/   RP2040 Embassy application
```

## Current pin map

### FPGA FT245-compatible bus

| Pico GPIO | Signal        |
| --------: | ------------- |
| GP0..GP7  | D0..D7        |
| GP8       | RXF# to FPGA  |
| GP9       | TXE# to FPGA  |
| GP10      | RD# from FPGA |
| GP11      | WR# from FPGA |

### W5500

| Pico GPIO | Signal |
| --------: | ------ |
| GP16      | MISO   |
| GP17      | CS#    |
| GP18      | SCK    |
| GP19      | MOSI   |
| GP20      | RESET# |
| GP21      | INT#   |

## Wire framing

Every RPC frame is:

```text
u16 little-endian postcard_body_length
postcard(HostFrame | DeviceFrame)
```

This same framing is used on USB bulk and TCP, so host code can share one
codec and swap transports.  The fixed-width length prefix uses `zerocopy`'s
byteorder-aware `U16` type.  The postcard body remains serde/postcard-owned
rather than reinterpreted with `zerocopy`, because it is a variable-length
schema-bearing message.

## FPGA command coverage

The RPC layer currently exposes:

- bridge version and stats
- FPGA protocol version
- start/stop/status
- hold control
- RAM read/write, chunked to `MAX_CHUNK = 1024`
- chip configuration including SFDP
- log control and log polling
- TOCTOU set/arm/disarm/reset/reset-all

## Build

Use a Rust 2024-compatible toolchain.  The dependency graph is resolved and
locked in `Cargo.lock`; use `--locked` for reproducible builds.  In particular,
`heapless` 0.9.x and `embedded-hal-bus` 0.3.x require Rust 1.87+ and 1.81+
respectively.

Install the RP2040 Rust target, then build from this directory:

```sh
rustup target add thumbv6m-none-eabi
cargo check -p norbert-pico-firmware --locked
cargo build -p norbert-pico-firmware --release --locked
```

This environment currently has a Nix-provided Rust toolchain without the
`thumbv6m-none-eabi` target installed via rustup, so protocol host checks are
the primary validation available here.

## Dependency notes

- Workspace crates use Rust edition 2024.
- Direct dependencies are declared at the newest compatible stable versions used
  for this no-std Embassy/RP2040 firmware, with exact transitive versions
  recorded in `Cargo.lock`.
- `zerocopy` is kept at 0.8.52, the latest stable release.  Cargo advertises
  0.9.0-alpha.0 as newer, but it is a pre-release and is intentionally not used
  for firmware bring-up.
- `zerocopy` is used only for fixed-width byte-order fields (`u16` frame
  lengths and FPGA command length fields).  The 24-bit FPGA addresses remain
  explicit byte pushes because `zerocopy` has no native U24 scalar, and the
  postcard bodies remain serde/postcard encoded.

## FT245 backend status

The firmware includes a conservative SIO FT245-device-side backend for bring-up.
It preserves the same public `Ft245Bus` API that the RPC layer uses. The next
performance step is to replace the per-byte SIO strobes with a PIO/DMA backend
behind the same API; that should not require changes to USB/TCP/postcard code.
