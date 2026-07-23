use clap::{Parser, Subcommand, ValueEnum};
use std::path::PathBuf;

// ---------------------------------------------------------------------------
// CLI definition
// ---------------------------------------------------------------------------

#[derive(Parser)]
#[command(name = "spi-flash-tool")]
#[command(about = "Tool for the Tang Primer 25K SPI Flash Emulator (UART + FT245)", long_about = None)]
pub(crate) struct Cli {
    /// Serial port device (ignored when --ft245 is used)
    #[arg(short, long, default_value = "/dev/ttyUSB0")]
    pub(crate) port: String,

    /// Use FT2232H FT245 async FIFO instead of UART
    #[arg(long)]
    pub(crate) ft245: bool,

    /// FT2232H serial number (for --ft245, when multiple devices are connected)
    #[arg(long)]
    pub(crate) ft_serial: Option<String>,

    #[command(subcommand)]
    pub(crate) command: Commands,
}

#[derive(Subcommand)]
pub(crate) enum Commands {
    /// Get protocol version from the device
    Version,

    /// Read data from the flash emulator
    Read {
        /// Start address (hex or decimal)
        #[arg(value_parser = parse_address)]
        address: u32,

        /// Number of bytes to read
        #[arg(value_parser = parse_address)]
        length: u32,

        /// Output file (stdout if not specified)
        #[arg(short, long)]
        output: Option<PathBuf>,
    },

    /// Write data to the flash emulator
    Write {
        /// Start address (hex or decimal)
        #[arg(value_parser = parse_address)]
        address: u32,

        /// Data to write (hex string, e.g., "deadbeef")
        data: String,
    },

    /// Load a file into the flash emulator
    Load {
        /// File to load
        file: PathBuf,

        /// Start address (hex or decimal)
        #[arg(short, long, default_value = "0", value_parser = parse_address)]
        address: u32,

        /// Verify after writing
        #[arg(short, long)]
        verify: bool,
    },

    /// Dump flash contents to a file
    Dump {
        /// Output file
        file: PathBuf,

        /// Start address
        #[arg(short, long, default_value = "0", value_parser = parse_address)]
        address: u32,

        /// Number of bytes to dump
        #[arg(short, long, value_parser = parse_address)]
        length: u32,
    },

    /// Configure the emulated flash chip identity
    Configure {
        /// Path to a chip database directory (containing .ron files).
        /// By default, use rflasher's compiled-in chip database.
        #[arg(long, value_name = "DIR")]
        chip_db: Option<PathBuf>,

        /// Chip name to emulate (substring match, e.g. "W25Q128.V")
        #[arg(short, long)]
        chip: String,
    },

    /// Control target flash #HOLD pin for SPI bus sharing
    ///
    /// Asserts #HOLD on the existing SPI flash so NORbert can respond
    /// instead.  The target flash tristates its outputs and ignores
    /// all SPI commands while held.  Mutually exclusive with quad I/O.
    Hold {
        /// Assert or release the target flash #HOLD pin
        #[arg(value_enum)]
        state: HoldState,
    },

    /// Monitor SPI bus activity in real time via FT245 logging.
    /// Displays commands, addresses, and data flow.  Also detects
    /// addresses read more than once (potential TOCTOU targets).
    Monitor,

    /// Configure TOCTOU attack traps.  On second read of a trapped
    /// address range, the FPGA serves replacement data from a
    /// different SDRAM location.
    Toctou {
        #[command(subcommand)]
        action: ToctouAction,
    },

    /// List available serial ports
    Ports,

    /// List connected FT2232H devices
    FtList,

    /// Diagnostic: send test bytes and report what comes back
    Probe,

    /// Start SPI emulation (target can now read the loaded firmware)
    Start,

    /// Stop SPI emulation (required before RAM/config operations)
    Stop,

    /// Query whether SPI emulation is currently running
    Status,
}

#[derive(Clone, Copy, ValueEnum)]
pub(crate) enum HoldState {
    On,
    Off,
}

#[derive(Subcommand)]
pub(crate) enum ToctouAction {
    /// Configure a trap entry (does NOT arm it)
    Set {
        /// Trap index (0-3)
        #[arg(value_parser = clap::value_parser!(u8).range(0..=3))]
        index: u8,
        /// Start address of the range to trap (hex or decimal)
        #[arg(value_parser = parse_address)]
        start: u32,
        /// Mask: 1-bits must match, 0-bits are don't-care (e.g. 0xFFF000 for 4KB)
        #[arg(value_parser = parse_address)]
        mask: u32,
        /// Replacement byte address base in SDRAM
        #[arg(value_parser = parse_address)]
        replace: u32,
    },
    /// Arm a trap (starts monitoring for the address range)
    Arm {
        /// Trap index (0-3)
        #[arg(value_parser = clap::value_parser!(u8).range(0..=3))]
        index: u8,
    },
    /// Disarm a trap
    Disarm {
        /// Trap index (0-3)
        #[arg(value_parser = clap::value_parser!(u8).range(0..=3))]
        index: u8,
    },
    /// Reset a trap (clear triggered state so first read serves original data again)
    Reset {
        /// Trap index (0-3)
        #[arg(value_parser = clap::value_parser!(u8).range(0..=3))]
        index: u8,
    },
    /// Reset all traps (disarm + clear triggered for all 4 entries)
    ResetAll,
}

fn parse_address(s: &str) -> std::result::Result<u32, String> {
    if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u32::from_str_radix(hex, 16).map_err(|e| e.to_string())
    } else {
        s.parse()
            .map_err(|e: std::num::ParseIntError| e.to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hold_state_is_validated_by_clap() {
        assert!(Cli::try_parse_from(["tool", "hold", "on"]).is_ok());
        assert!(Cli::try_parse_from(["tool", "hold", "invalid"]).is_err());
    }

    #[test]
    fn trap_index_is_range_checked_by_clap() {
        assert!(Cli::try_parse_from(["tool", "toctou", "arm", "3"]).is_ok());
        assert!(Cli::try_parse_from(["tool", "toctou", "arm", "4"]).is_err());
    }

    #[test]
    fn address_parser_accepts_hex_and_decimal() {
        assert_eq!(parse_address("0x10"), Ok(16));
        assert_eq!(parse_address("16"), Ok(16));
    }
}
