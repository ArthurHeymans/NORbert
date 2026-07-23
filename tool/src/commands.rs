mod diagnostics;
mod monitor;

use diagnostics::{cmd_ft_list, cmd_probe};
use monitor::cmd_monitor;

use crate::chip::{self, FlashChipExt};
use crate::cli::{Cli, HoldState, ToctouAction};
use crate::device::FlashDevice;
use crate::protocol::*;
use crate::sfdp;
#[cfg(any(feature = "d2xx", feature = "ftdi"))]
use crate::transport::{FT2232H_PID, FTDI_VID};
use anyhow::{Context, Result, anyhow, bail};
use indicatif::{ProgressBar, ProgressStyle};
use std::fs::File;
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::thread;
use std::time::Duration;
// ---------------------------------------------------------------------------
// Command implementations
// ---------------------------------------------------------------------------

fn decode_hex(s: &str) -> Result<Vec<u8>, String> {
    let mut decoded = Vec::with_capacity(s.len() / 2);
    let mut high_nibble = None;

    for (position, byte) in s.bytes().enumerate() {
        if byte.is_ascii_whitespace() {
            continue;
        }
        let nibble = match byte {
            b'0'..=b'9' => byte - b'0',
            b'a'..=b'f' => byte - b'a' + 10,
            b'A'..=b'F' => byte - b'A' + 10,
            _ => return Err(format!("Invalid hex character at position {position}")),
        };
        match high_nibble.take() {
            Some(high) => decoded.push((high << 4) | nibble),
            None => high_nibble = Some(nibble),
        }
    }

    if high_nibble.is_some() {
        return Err("Hex string must contain complete byte pairs".to_string());
    }
    Ok(decoded)
}

fn open_device(cli: &Cli) -> Result<FlashDevice> {
    if cli.ft245 {
        #[cfg(any(feature = "d2xx", feature = "ftdi"))]
        {
            let backend = if cfg!(feature = "d2xx") {
                "D2XX"
            } else {
                "rs-ftdi"
            };
            eprintln!("Opening FT2232H in async FIFO mode ({})...", backend);
            return FlashDevice::open_ft245(cli.ft_serial.as_deref());
        }
        #[cfg(not(any(feature = "d2xx", feature = "ftdi")))]
        bail!(
            "--ft245 requires the 'd2xx' or 'ftdi' feature (build with --features d2xx, --features ftdi, or --no-default-features --features d2xx)"
        );
    }
    FlashDevice::open_serial(&cli.port)
}

fn cmd_version(cli: &Cli) -> Result<()> {
    let mut device = open_device(cli)?;
    let version = device.get_version()?;
    println!("Protocol version: {}", version);
    if version != PROTOCOL_VERSION {
        eprintln!(
            "Warning: Expected version {}, got {}",
            PROTOCOL_VERSION, version
        );
    }
    Ok(())
}

fn cmd_read(cli: &Cli, address: u32, length: u32, output: Option<PathBuf>) -> Result<()> {
    let mut device = open_device(cli)?;

    device.with_emulation_stopped(|device| {
        let pb = ProgressBar::new(length as u64);
        pb.set_style(
            ProgressStyle::default_bar()
                .template(
                    "[{elapsed_precise}] [{bar:40.cyan/blue}] {bytes}/{total_bytes} ({bytes_per_sec}, {eta})",
                )?
                .progress_chars("#>-"),
        );

        let result = device.read_with_progress(address, length, |completed| {
            pb.inc(completed as u64);
        })?;
        pb.finish_with_message("done");

        match output {
            Some(path) => {
                let mut file = File::create(&path)?;
                file.write_all(&result)?;
                println!("Wrote {} bytes to {}", result.len(), path.display());
            }
            None => {
                for (i, chunk) in result.chunks(16).enumerate() {
                    print!("{:08x}: ", address as usize + i * 16);
                    for b in chunk {
                        print!("{:02x} ", b);
                    }
                    for _ in chunk.len()..16 {
                        print!("   ");
                    }
                    print!(" |");
                    for b in chunk {
                        let c = *b as char;
                        if c.is_ascii_graphic() || c == ' ' {
                            print!("{}", c);
                        } else {
                            print!(".");
                        }
                    }
                    println!("|");
                }
            }
        }

        Ok(())
    })
}

fn cmd_write(cli: &Cli, address: u32, data_hex: &str) -> Result<()> {
    let data = decode_hex(data_hex).map_err(|e| anyhow!("Invalid hex string: {e}"))?;

    if data.is_empty() {
        bail!("No data to write");
    }

    let mut device = open_device(cli)?;

    device.with_emulation_stopped(|device| device.write(address, &data))?;
    println!("Wrote {} bytes to 0x{:08x}", data.len(), address);
    Ok(())
}

fn cmd_load(cli: &Cli, file: &PathBuf, address: u32, verify: bool) -> Result<()> {
    let mut f = File::open(file).with_context(|| format!("Failed to open {}", file.display()))?;
    let mut data = Vec::new();
    f.read_to_end(&mut data)?;

    if data.is_empty() {
        bail!("File is empty");
    }

    let transport_name = if cli.ft245 { "FT245" } else { "UART" };
    println!(
        "Loading {} ({} bytes) to 0x{:08x} via {}",
        file.display(),
        data.len(),
        address,
        transport_name
    );

    let mut device = open_device(cli)?;

    // Load is the one command whose natural end state is "ready to
    // serve", so the dance is stop -> write -> (verify) -> start.
    device.with_emulation_then_start(|device| {
        let progress = ProgressBar::new(data.len() as u64);
        progress.set_style(
            ProgressStyle::default_bar()
                .template(
                    "[{elapsed_precise}] [{bar:40.cyan/blue}] {bytes}/{total_bytes} ({bytes_per_sec}, {eta})",
                )?
                .progress_chars("#>-"),
        );
        device.write_with_progress(address, &data, |completed| {
            progress.inc(completed as u64);
        })?;
        progress.finish_with_message("loaded");
        println!("Loaded {} bytes", data.len());

        if verify {
            println!("Verifying...");
            let progress = ProgressBar::new(data.len() as u64);
            progress.set_style(
                ProgressStyle::default_bar()
                    .template(
                        "[{elapsed_precise}] [{bar:40.green/blue}] {bytes}/{total_bytes} ({bytes_per_sec}, {eta})",
                    )?
                    .progress_chars("#>-"),
            );
            let readback = device.read_with_progress(address, data.len() as u32, |completed| {
                progress.inc(completed as u64);
            })?;
            progress.finish_with_message("verified");
            let mismatches: Vec<_> = data
                .iter()
                .zip(&readback)
                .enumerate()
                .filter(|(_, (expected, actual))| expected != actual)
                .collect();

            for (offset, (expected, actual)) in mismatches.iter().take(10) {
                eprintln!(
                    "Mismatch at 0x{:08x}: expected 0x{:02x}, got 0x{:02x}",
                    address + *offset as u32,
                    expected,
                    actual
                );
            }
            if !mismatches.is_empty() {
                bail!("Verification failed: {} byte(s) differ", mismatches.len());
            }
            println!("Verification passed!");
        }

        Ok(())
    })
}

fn cmd_dump(cli: &Cli, file: &Path, address: u32, length: u32) -> Result<()> {
    cmd_read(cli, address, length, Some(file.to_path_buf()))
}

fn cmd_configure(cli: &Cli, chip_db_path: Option<&Path>, chip_name: &str) -> Result<()> {
    let chip_db_path = chip_db_path.map(|path| {
        let path_string = path.to_string_lossy();
        if let Some(stripped) = path_string.strip_prefix("~/") {
            let home = std::env::var("HOME").unwrap_or_else(|_| "/root".to_string());
            PathBuf::from(format!("{home}/{stripped}"))
        } else {
            path.to_path_buf()
        }
    });

    if let Some(path) = chip_db_path.as_deref() {
        eprintln!("Loading chip database override from {}...", path.display());
    } else {
        eprintln!("Loading rflasher's compiled-in chip database...");
    }
    let chips = chip::load_chip_db(chip_db_path.as_deref())?;
    eprintln!("Loaded {} chip definitions", chips.len());

    // Find requested chip
    let chip = chip::find_chip_by_name(&chips, chip_name)?;

    eprintln!(
        "Selected: {} {} (JEDEC {:02X} {:04X})",
        chip.vendor, chip.name, chip.jedec_manufacturer, chip.jedec_device,
    );
    eprintln!(
        "  Size:       {} KB ({} MB)",
        chip.total_size / 1024,
        chip.total_size / (1024 * 1024),
    );
    eprintln!("  Page size:  {} bytes", chip.page_size);
    eprintln!("  4-byte addr: {}", chip.supports_4byte());
    eprintln!("  Dual I/O:   {}", chip.supports_dual());
    eprintln!("  Quad I/O:   {}", chip.supports_quad());
    if chip.aai_word() {
        eprintln!("  Write mode: AAI Word Program (0xAD)");
    }
    if chip.write_byte() {
        eprintln!("  Write mode: Byte program");
    }

    let erase_ops = chip.sector_erase_ops();
    for op in &erase_ops {
        if let Some(block_size) = chip::erase_block_size(op) {
            eprintln!(
                "  Erase:      0x{:02X} ({} KB)",
                op.opcode,
                block_size / 1024
            );
        }
    }

    // Generate SFDP table.  For chips that do not support SFDP in real
    // hardware (e.g. older SST25VFxxx), the table is all-0xFF so SFDP
    // probes see no valid signature, matching the real part.
    let sfdp_table = sfdp::generate_sfdp(chip).context("Failed to generate SFDP table")?;
    if chip.supports_sfdp() {
        eprintln!("  SFDP table: {} bytes (valid)", sfdp_table.len());
    } else {
        eprintln!(
            "  SFDP table: {} bytes (blank, chip does not support SFDP)",
            sfdp_table.len()
        );
    }

    // Send to FPGA.  Must be stopped while CHIPCONFIG is processed,
    // otherwise spi_trx could read inconsistent cfg_* values mid-transfer.
    let mut device = open_device(cli)?;
    device.with_emulation_stopped(|device| device.send_chip_config(chip, &sfdp_table))?;

    eprintln!("Configuration applied successfully.");
    Ok(())
}

fn cmd_hold(cli: &Cli, state: HoldState) -> Result<()> {
    let enable = matches!(state, HoldState::On);
    let mut device = open_device(cli)?;
    device.set_hold(enable)?;
    if enable {
        eprintln!("HOLD asserted: target flash silenced");
    } else {
        eprintln!("HOLD released: target flash active");
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// TOCTOU command -- configure trap entries
// ---------------------------------------------------------------------------

fn cmd_toctou(cli: &Cli, action: &ToctouAction) -> Result<()> {
    let mut device = open_device(cli)?;

    match action {
        ToctouAction::Set {
            index,
            start,
            mask,
            replace,
        } => {
            device.toctou_set(*index, *start, *mask, *replace)?;
            eprintln!(
                "Trap {} configured: start=0x{:06X} mask=0x{:06X} replace=0x{:06X}",
                index, start, mask, replace
            );
            eprintln!("Use 'toctou arm {}' to activate.", index);
        }
        ToctouAction::Arm { index } => {
            device.toctou_arm(*index)?;
            eprintln!("Trap {} armed.", index);
        }
        ToctouAction::Disarm { index } => {
            device.toctou_disarm(*index)?;
            eprintln!("Trap {} disarmed.", index);
        }
        ToctouAction::Reset { index } => {
            device.toctou_reset(*index)?;
            eprintln!("Trap {} reset (triggered flag cleared).", index);
        }
        ToctouAction::ResetAll => {
            device.toctou_reset_all()?;
            eprintln!("All traps reset and disarmed.");
        }
    }

    Ok(())
}

fn cmd_ports() -> Result<()> {
    let ports = serialport::available_ports()?;
    if ports.is_empty() {
        println!("No serial ports found");
    } else {
        println!("Available serial ports:");
        for port in ports {
            print!("  {}", port.port_name);
            match port.port_type {
                serialport::SerialPortType::UsbPort(info) => {
                    if let Some(manufacturer) = info.manufacturer {
                        print!(" - {}", manufacturer);
                    }
                    if let Some(product) = info.product {
                        print!(" {}", product);
                    }
                }
                serialport::SerialPortType::PciPort => print!(" (PCI)"),
                serialport::SerialPortType::BluetoothPort => print!(" (Bluetooth)"),
                serialport::SerialPortType::Unknown => {}
            }
            println!();
        }
    }
    Ok(())
}

fn cmd_start(cli: &Cli) -> Result<()> {
    let mut device = open_device(cli)?;
    device.start_emulation()?;
    println!("SPI emulation started.");
    Ok(())
}

fn cmd_stop(cli: &Cli) -> Result<()> {
    let mut device = open_device(cli)?;
    device.stop_emulation()?;
    println!("SPI emulation stopped.");
    Ok(())
}

fn cmd_status(cli: &Cli) -> Result<()> {
    let mut device = open_device(cli)?;
    let running = device.status()?;
    println!(
        "SPI emulation: {}",
        if running { "running" } else { "stopped" }
    );
    Ok(())
}

pub(crate) fn run(cli: &Cli) -> Result<()> {
    use crate::cli::Commands;
    match &cli.command {
        Commands::Version => cmd_version(cli),
        Commands::Read {
            address,
            length,
            output,
        } => cmd_read(cli, *address, *length, output.clone()),
        Commands::Write { address, data } => cmd_write(cli, *address, data),
        Commands::Load {
            file,
            address,
            verify,
        } => cmd_load(cli, file, *address, *verify),
        Commands::Dump {
            file,
            address,
            length,
        } => cmd_dump(cli, file, *address, *length),
        Commands::Configure { chip_db, chip } => cmd_configure(cli, chip_db.as_deref(), chip),
        Commands::Hold { state } => cmd_hold(cli, *state),
        Commands::Monitor => cmd_monitor(cli),
        Commands::Toctou { action } => cmd_toctou(cli, action),
        Commands::Ports => cmd_ports(),
        Commands::FtList => cmd_ft_list(),
        Commands::Probe => cmd_probe(cli),
        Commands::Start => cmd_start(cli),
        Commands::Stop => cmd_stop(cli),
        Commands::Status => cmd_status(cli),
    }
}

#[cfg(test)]
mod tests {
    use super::decode_hex;

    #[test]
    fn hex_decoder_accepts_ascii_whitespace() {
        assert_eq!(
            decode_hex("de ad\nBE\tEF").unwrap(),
            [0xde, 0xad, 0xbe, 0xef]
        );
    }

    #[test]
    fn hex_decoder_rejects_invalid_and_incomplete_input() {
        assert!(decode_hex("0g").is_err());
        assert!(decode_hex("abc").is_err());
    }
}
