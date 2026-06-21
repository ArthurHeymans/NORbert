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
codec and swap transports.

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

Install the RP2040 Rust target, then build from this directory:

```sh
rustup target add thumbv6m-none-eabi
cargo check -p norbert-pico-firmware
cargo build -p norbert-pico-firmware --release
```

This environment currently has a Nix-provided Rust toolchain without the
`thumbv6m-none-eabi` target installed via rustup, so only the protocol crate was
checked here.

## FT245 backend status

The firmware includes a conservative SIO FT245-device-side backend for bring-up.
It preserves the same public `Ft245Bus` API that the RPC layer uses. The next
performance step is to replace the per-byte SIO strobes with a PIO/DMA backend
behind the same API; that should not require changes to USB/TCP/postcard code.
