#![no_std]

use heapless::Vec;
use serde::{Deserialize, Serialize};
use zerocopy::byteorder::little_endian::U16 as LeU16;

/// Maximum user payload carried in one RPC frame.
pub const MAX_CHUNK: usize = 1024;

/// Maximum postcard body size, excluding the two-byte length prefix.
pub const MAX_FRAME_BODY: usize = 1152;

/// Maximum serialized frame size, including the two-byte length prefix.
pub const MAX_FRAME: usize = MAX_FRAME_BODY + 2;

/// TCP port used by the bridge RPC server.
pub const TCP_PORT: u16 = 24500;

/// NORbert FPGA byte protocol expected by current gateware.
pub const FPGA_PROTOCOL_VERSION: u8 = 5;

/// Bridge firmware protocol version.
pub const BRIDGE_PROTOCOL_VERSION: u16 = 1;

/// One host-to-device RPC frame.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct HostFrame {
    /// Host-selected sequence number echoed in the response.
    pub seq: u32,
    /// Request payload.
    pub request: Request,
}

/// One device-to-host RPC frame.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DeviceFrame {
    /// Sequence number from the request this response completes.
    pub seq: u32,
    /// Response payload.
    pub response: Response,
}

/// Host-to-bridge request set.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Request {
    /// Return bridge firmware identity.
    BridgeVersion,
    /// Query the FPGA byte-protocol version.
    FpgaVersion,
    /// Query whether SPI flash emulation is running in the FPGA.
    Status,
    /// Enable SPI flash emulation in the FPGA.
    Start,
    /// Disable SPI flash emulation in the FPGA.
    Stop,
    /// Assert or release the target flash #HOLD line through the FPGA.
    Hold { asserted: bool },
    /// Enable or disable FPGA SPI transaction logging.
    LogControl { enabled: bool },
    /// Poll up to 255 raw logger bytes from the FPGA.
    LogPoll,
    /// Read a chunk of SDRAM-backed flash contents from the FPGA.
    RamRead { address: u32, length: u16 },
    /// Write a chunk of SDRAM-backed flash contents to the FPGA.
    RamWrite {
        address: u32,
        data: Vec<u8, MAX_CHUNK>,
    },
    /// Apply a chip identity/SFDP table to the FPGA.
    ChipConfig(ChipConfig),
    /// Manage one of the FPGA TOCTOU traps.
    Toctou(ToctouRequest),
    /// Return bridge-side counters.
    Stats,
}

/// Device-to-host response set.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Response {
    /// Command completed successfully and has no extra data.
    Ack,
    /// Bridge firmware identity.
    BridgeVersion(BridgeVersion),
    /// FPGA protocol version byte.
    FpgaVersion(u8),
    /// Current FPGA SPI emulation state.
    Status { running: bool },
    /// Raw bytes returned from the FPGA.
    Data(Vec<u8, MAX_CHUNK>),
    /// Raw SPI logger bytes returned from the FPGA.
    Log(Vec<u8, 256>),
    /// Bridge-side counters.
    Stats(BridgeStats),
    /// Request failed.
    Error(ErrorCode),
}

/// Bridge firmware identity.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BridgeVersion {
    /// RPC protocol version spoken by this firmware.
    pub protocol: u16,
    /// Maximum payload accepted in [`Request::RamWrite`].
    pub max_chunk: u16,
    /// Firmware build description as UTF-8 bytes.
    pub build: Vec<u8, 32>,
}

/// Bridge runtime counters.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BridgeStats {
    /// Successfully handled RPC requests.
    pub requests: u32,
    /// Failed RPC requests.
    pub errors: u32,
    /// Bytes sent from host transports to the FPGA FIFO bus.
    pub fpga_tx_bytes: u32,
    /// Bytes received from the FPGA FIFO bus to host transports.
    pub fpga_rx_bytes: u32,
}

/// FPGA chip-configuration command payload.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ChipConfig {
    /// Three JEDEC ID bytes: manufacturer, device high, device low.
    pub jedec_id: [u8; 3],
    /// True when the emulated flash should support four-byte addressing.
    pub supports_4byte_address: bool,
    /// Chip erase size expressed as 8-byte SDRAM bursts.
    pub chip_erase_bursts: u32,
    /// SFDP table bytes. Maximum accepted by current FPGA gateware is 128.
    pub sfdp: Vec<u8, 128>,
}

/// FPGA TOCTOU trap management request.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ToctouRequest {
    /// Configure an entry but leave it disarmed.
    Set {
        /// Trap index, 0 through 3.
        index: u8,
        /// Start byte address.
        start: u32,
        /// Byte-address match mask.
        mask: u32,
        /// Replacement base byte address.
        replace: u32,
    },
    /// Arm an entry.
    Arm { index: u8 },
    /// Disarm an entry.
    Disarm { index: u8 },
    /// Clear an entry's triggered flag.
    Reset { index: u8 },
    /// Reset and disarm all entries.
    ResetAll,
}

/// Error reported in [`Response::Error`].
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ErrorCode {
    /// The received frame was not valid postcard data.
    BadPostcard,
    /// The length prefix exceeded the configured maximum.
    FrameTooLarge,
    /// The request violates FPGA alignment/length constraints.
    BadArgument,
    /// The FPGA returned a non-ACK status byte.
    FpgaRejected(u8),
    /// The FPGA bus timed out.
    FpgaTimeout,
    /// A response could not fit into the configured frame size.
    EncodeFailed,
}

/// Encode a host frame into `out` as `u16_le length || postcard_body`.
///
/// Returns the total number of bytes to transmit, including the length prefix.
pub fn encode_host_frame(frame: &HostFrame, out: &mut [u8]) -> Result<usize, ErrorCode> {
    encode_prefixed(frame, out)
}

/// Encode a device frame into `out` as `u16_le length || postcard_body`.
///
/// Returns the total number of bytes to transmit, including the length prefix.
pub fn encode_device_frame(frame: &DeviceFrame, out: &mut [u8]) -> Result<usize, ErrorCode> {
    encode_prefixed(frame, out)
}

/// Decode a postcard host-frame body, excluding the two-byte length prefix.
pub fn decode_host_body(body: &[u8]) -> Result<HostFrame, ErrorCode> {
    postcard::from_bytes(body).map_err(|_| ErrorCode::BadPostcard)
}

/// Decode a postcard device-frame body, excluding the two-byte length prefix.
pub fn decode_device_body(body: &[u8]) -> Result<DeviceFrame, ErrorCode> {
    postcard::from_bytes(body).map_err(|_| ErrorCode::BadPostcard)
}

fn encode_prefixed<T: Serialize>(value: &T, out: &mut [u8]) -> Result<usize, ErrorCode> {
    if out.len() < 2 {
        return Err(ErrorCode::EncodeFailed);
    }

    let body_len = postcard::to_slice(value, &mut out[2..])
        .map_err(|_| ErrorCode::EncodeFailed)?
        .len();
    if body_len > u16::MAX as usize {
        return Err(ErrorCode::EncodeFailed);
    }

    let len = LeU16::new(body_len as u16).to_bytes();
    let Some(prefix) = out.get_mut(..2) else {
        return Err(ErrorCode::EncodeFailed);
    };
    prefix.copy_from_slice(&len);
    Ok(body_len + 2)
}

/// Return the little-endian length prefix as a `usize`.
pub fn frame_len(prefix: [u8; 2]) -> usize {
    LeU16::from(prefix).get() as usize
}
