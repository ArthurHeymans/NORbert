mod chip;
mod sfdp;

use anyhow::{anyhow, bail, Context, Result};
use clap::{Parser, Subcommand};
use indicatif::{ProgressBar, ProgressStyle};
use serialport::SerialPort;
use std::fs::File;
use std::io::{Read, Write};
use std::path::PathBuf;
use std::thread;
use std::time::{Duration, Instant};

const BAUD_RATE: u32 = 2_000_000;
const BLOCK_SIZE: usize = 2048; // Max transfer size (256 * 8 bytes)
const PROTOCOL_VERSION: u8 = 2;

// Protocol commands (shared between UART and FT245 transports)
const CMD_VERSION: u8 = 0x30;
const CMD_READ: u8 = 0x31;
const CMD_WRITE: u8 = 0x32;
const CMD_CHIPCONFIG: u8 = 0x33;

// FT2232H USB identifiers
const FTDI_VID: u16 = 0x0403;
const FT2232H_PID: u16 = 0x6010;

// ---------------------------------------------------------------------------
// Transport trait -- abstracts UART serial vs FT245 FIFO
// ---------------------------------------------------------------------------

trait Transport {
    fn write_all(&mut self, data: &[u8]) -> Result<()>;
    fn read_exact(&mut self, buf: &mut [u8]) -> Result<()>;
}

// ---------------------------------------------------------------------------
// UART transport (existing serial port path)
// ---------------------------------------------------------------------------

struct SerialTransport {
    port: Box<dyn SerialPort>,
}

impl SerialTransport {
    fn open(port_name: &str) -> Result<Self> {
        let mut port = serialport::new(port_name, BAUD_RATE)
            .timeout(Duration::from_secs(2))
            .open()
            .with_context(|| format!("Failed to open serial port {}", port_name))?;

        // Sync with FPGA: USB-UART bridges can send spurious bytes on
        // port open (DTR/RTS toggling, line glitches). Wait for the
        // FPGA's idle timeout to fire and reset the parser, then flush.
        thread::sleep(Duration::from_millis(5));

        let mut discard = [0u8; 256];
        port.set_timeout(Duration::from_millis(1))?;
        while port.read(&mut discard).unwrap_or(0) > 0 {}
        port.set_timeout(Duration::from_secs(2))?;

        Ok(Self { port })
    }
}

impl Transport for SerialTransport {
    fn write_all(&mut self, data: &[u8]) -> Result<()> {
        self.port.write_all(data)?;
        Ok(())
    }

    fn read_exact(&mut self, buf: &mut [u8]) -> Result<()> {
        self.port.read_exact(buf)?;
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// FT245 transport (FT2232H synchronous FIFO via libftd2xx)
// ---------------------------------------------------------------------------

struct Ft245Transport {
    ft: libftd2xx::Ft2232h,
}

impl Ft245Transport {
    fn open(serial: Option<&str>) -> Result<Self> {
        use libftd2xx::FtdiCommon;

        // FT2232H EEPROM must be configured for "245 FIFO" on Channel A
        // (one-time setup using FT_PROG).  No special BitMode is needed
        // for async FIFO -- just open, reset, and read/write.
        let mut ft = if let Some(sn) = serial {
            libftd2xx::Ft2232h::with_serial_number(sn)
                .with_context(|| format!("No FT2232H with serial '{}'", sn))?
        } else {
            // Default: open Channel A by its standard description.
            // If EEPROM was reprogrammed with a custom description,
            // use --ft-serial to select by serial number instead.
            libftd2xx::Ft2232h::with_description("Dual RS232-HS A")
                .context("No FT2232H found (looking for 'Dual RS232-HS A')")?
        };

        ft.reset().context("FT2232H reset failed")?;
        ft.purge_all().context("FT2232H purge failed")?;

        // Set USB transfer size for bulk throughput
        ft.set_usb_parameters(65536)
            .context("Failed to set USB parameters")?;

        // Low latency for responsive transfers
        ft.set_latency_timer(Duration::from_millis(2))
            .context("Failed to set latency timer")?;

        // Drain any stale data in the receive buffer
        let mut trash = [0u8; 4096];
        loop {
            let n = ft.queue_status().context("Failed to query queue status")?;
            if n == 0 {
                break;
            }
            let to_read = std::cmp::min(n, trash.len());
            let _ = ft.read(&mut trash[..to_read]);
        }

        Ok(Self { ft })
    }
}

impl Transport for Ft245Transport {
    fn write_all(&mut self, data: &[u8]) -> Result<()> {
        use libftd2xx::FtdiCommon;
        self.ft.write_all(data)?;
        Ok(())
    }

    fn read_exact(&mut self, buf: &mut [u8]) -> Result<()> {
        use libftd2xx::FtdiCommon;
        let mut pos = 0;
        let deadline = Instant::now() + Duration::from_secs(5);
        while pos < buf.len() {
            if Instant::now() > deadline {
                bail!("FT245 read timeout: got {} of {} bytes", pos, buf.len());
            }
            let n = self.ft.read(&mut buf[pos..])?;
            if n == 0 {
                thread::sleep(Duration::from_micros(100));
            }
            pos += n;
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// FlashDevice -- protocol implementation over any transport
// ---------------------------------------------------------------------------

struct FlashDevice {
    transport: Box<dyn Transport>,
}

impl FlashDevice {
    fn open_serial(port_name: &str) -> Result<Self> {
        Ok(Self {
            transport: Box::new(SerialTransport::open(port_name)?),
        })
    }

    fn open_ft245(serial: Option<&str>) -> Result<Self> {
        Ok(Self {
            transport: Box::new(Ft245Transport::open(serial)?),
        })
    }

    fn get_version(&mut self) -> Result<u8> {
        self.transport.write_all(&[CMD_VERSION])?;
        let mut buf = [0u8; 1];
        self.transport.read_exact(&mut buf)?;
        Ok(buf[0])
    }

    fn read_block(&mut self, address: u32, length: usize) -> Result<Vec<u8>> {
        if length == 0 || length > BLOCK_SIZE {
            bail!("Invalid length: must be 1-{}", BLOCK_SIZE);
        }
        if address % 8 != 0 {
            bail!("Address must be 8-byte aligned");
        }
        if length % 8 != 0 {
            bail!("Length must be a multiple of 8");
        }

        let addr_units = address / 8;
        let len_units = length / 8;

        let cmd = [
            CMD_READ,
            ((addr_units >> 16) & 0xFF) as u8,
            ((addr_units >> 8) & 0xFF) as u8,
            (addr_units & 0xFF) as u8,
            len_units as u8,
        ];

        self.transport.write_all(&cmd)?;

        let mut buf = vec![0u8; length];
        self.transport.read_exact(&mut buf)?;
        Ok(buf)
    }

    fn write_block(&mut self, address: u32, data: &[u8]) -> Result<()> {
        if data.is_empty() || data.len() > BLOCK_SIZE {
            bail!("Invalid data length: must be 1-{}", BLOCK_SIZE);
        }
        if address % 8 != 0 {
            bail!("Address must be 8-byte aligned");
        }
        if data.len() % 8 != 0 {
            bail!("Data length must be a multiple of 8");
        }

        let addr_units = address / 8;
        let len_units = data.len() / 8;

        // Combine header and data into a single write to avoid
        // transfer gaps that could trigger the FPGA's idle timeout
        let mut buf = Vec::with_capacity(5 + data.len());
        buf.extend_from_slice(&[
            CMD_WRITE,
            ((addr_units >> 16) & 0xFF) as u8,
            ((addr_units >> 8) & 0xFF) as u8,
            (addr_units & 0xFF) as u8,
            len_units as u8,
        ]);
        buf.extend_from_slice(data);
        self.transport.write_all(&buf)?;

        // Wait for completion byte
        let mut resp = [0u8; 1];
        self.transport.read_exact(&mut resp)?;
        if resp[0] != 0x01 {
            bail!("Unexpected response: 0x{:02x}", resp[0]);
        }
        Ok(())
    }

    #[allow(dead_code)]
    fn read(&mut self, address: u32, length: u32) -> Result<Vec<u8>> {
        let mut result = Vec::with_capacity(length as usize);
        let mut addr = address;
        let mut remaining = length as usize;

        let start_offset = (addr % 8) as usize;
        if start_offset != 0 {
            let aligned_addr = addr - start_offset as u32;
            let chunk_len = std::cmp::min(8 - start_offset, remaining);
            let block = self.read_block(aligned_addr, 8)?;
            result.extend_from_slice(&block[start_offset..start_offset + chunk_len]);
            addr += chunk_len as u32;
            remaining -= chunk_len;
        }

        while remaining >= 8 {
            let chunk_len = std::cmp::min(BLOCK_SIZE, remaining / 8 * 8);
            let block = self.read_block(addr, chunk_len)?;
            result.extend_from_slice(&block);
            addr += chunk_len as u32;
            remaining -= chunk_len;
        }

        if remaining > 0 {
            let block = self.read_block(addr, 8)?;
            result.extend_from_slice(&block[..remaining]);
        }

        Ok(result)
    }

    /// Send chip configuration to the FPGA.
    ///
    /// Protocol (CMD_CHIPCONFIG = 0x33):
    ///   Byte 0:    0x33
    ///   Byte 1:    JEDEC manufacturer ID
    ///   Byte 2:    JEDEC device ID high byte
    ///   Byte 3:    JEDEC device ID low byte
    ///   Byte 4:    Flags (bit 0 = supports 4-byte addressing)
    ///   Byte 5-7:  Chip erase burst count (23-bit, MSB first)
    ///              = (total_size / 8) - 1
    ///   Byte 8:    SFDP table length (0-128)
    ///   Byte 9+:   SFDP table data
    ///
    /// Response: 0x01 on success.
    fn send_chip_config(&mut self, chip: &chip::FlashChip, sfdp_table: &[u8]) -> Result<()> {
        let jedec = chip.jedec_id_bytes();
        let erase_bursts = chip.chip_erase_bursts();
        let flags: u8 = if chip.supports_4byte { 0x01 } else { 0x00 };

        let sfdp_len = sfdp_table.len();
        if sfdp_len > 128 {
            bail!("SFDP table too large: {} bytes (max 128)", sfdp_len);
        }

        // Build the full command in one buffer to avoid USB transfer gaps
        let mut buf = Vec::with_capacity(9 + sfdp_len);
        buf.push(CMD_CHIPCONFIG);
        buf.push(jedec[0]); // manufacturer
        buf.push(jedec[1]); // device high
        buf.push(jedec[2]); // device low
        buf.push(flags);
        buf.push(((erase_bursts >> 16) & 0x7F) as u8);
        buf.push(((erase_bursts >> 8) & 0xFF) as u8);
        buf.push((erase_bursts & 0xFF) as u8);
        buf.push(sfdp_len as u8);
        buf.extend_from_slice(sfdp_table);
        self.transport.write_all(&buf)?;

        // Wait for ack
        let mut resp = [0u8; 1];
        self.transport.read_exact(&mut resp)?;
        if resp[0] != 0x01 {
            bail!("Chip config failed: unexpected response 0x{:02x}", resp[0]);
        }
        Ok(())
    }

    fn write(&mut self, address: u32, data: &[u8]) -> Result<()> {
        if address % 8 != 0 {
            bail!("Address must be 8-byte aligned for writes");
        }

        let mut padded = data.to_vec();
        if padded.len() % 8 != 0 {
            padded.resize((padded.len() + 7) / 8 * 8, 0xFF);
        }

        let mut addr = address;
        let mut offset = 0;

        while offset < padded.len() {
            let chunk_len = std::cmp::min(BLOCK_SIZE, padded.len() - offset);
            self.write_block(addr, &padded[offset..offset + chunk_len])?;
            addr += chunk_len as u32;
            offset += chunk_len;
        }

        Ok(())
    }
}

// ---------------------------------------------------------------------------
// CLI definition
// ---------------------------------------------------------------------------

#[derive(Parser)]
#[command(name = "spi-flash-tool")]
#[command(about = "Tool for the Tang Primer 25K SPI Flash Emulator (UART + FT245)", long_about = None)]
struct Cli {
    /// Serial port device (ignored when --ft245 is used)
    #[arg(short, long, default_value = "/dev/ttyUSB0")]
    port: String,

    /// Use FT2232H FT245 synchronous FIFO instead of UART
    #[arg(long)]
    ft245: bool,

    /// FT2232H serial number (for --ft245, when multiple devices are connected)
    #[arg(long)]
    ft_serial: Option<String>,

    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
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
        /// Path to rflasher chip database directory (containing .ron files)
        #[arg(long, default_value = "~/src/rflasher/chips/vendors")]
        chip_db: String,

        /// Chip name to emulate (substring match, e.g. "W25Q128.V")
        #[arg(short, long)]
        chip: String,
    },

    /// List available serial ports
    Ports,

    /// List connected FT2232H devices
    FtList,
}

fn parse_address(s: &str) -> Result<u32, String> {
    if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u32::from_str_radix(hex, 16).map_err(|e| e.to_string())
    } else {
        s.parse()
            .map_err(|e: std::num::ParseIntError| e.to_string())
    }
}

// ---------------------------------------------------------------------------
// Command implementations
// ---------------------------------------------------------------------------

fn open_device(cli: &Cli) -> Result<FlashDevice> {
    if cli.ft245 {
        eprintln!("Opening FT2232H in sync FIFO mode...");
        FlashDevice::open_ft245(cli.ft_serial.as_deref())
    } else {
        FlashDevice::open_serial(&cli.port)
    }
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

    let pb = ProgressBar::new(length as u64);
    pb.set_style(
        ProgressStyle::default_bar()
            .template("[{elapsed_precise}] [{bar:40.cyan/blue}] {bytes}/{total_bytes} ({eta})")?
            .progress_chars("#>-"),
    );

    let mut result = Vec::with_capacity(length as usize);
    let mut addr = address;
    let mut remaining = length as usize;

    let start_offset = (addr % 8) as usize;
    if start_offset != 0 {
        let aligned_addr = addr - start_offset as u32;
        let chunk_len = std::cmp::min(8 - start_offset, remaining);
        let block = device.read_block(aligned_addr, 8)?;
        result.extend_from_slice(&block[start_offset..start_offset + chunk_len]);
        addr += chunk_len as u32;
        remaining -= chunk_len;
        pb.inc(chunk_len as u64);
    }

    while remaining >= 8 {
        let chunk_len = std::cmp::min(BLOCK_SIZE, remaining / 8 * 8);
        let block = device.read_block(addr, chunk_len)?;
        result.extend_from_slice(&block);
        addr += chunk_len as u32;
        remaining -= chunk_len;
        pb.inc(chunk_len as u64);
    }

    if remaining > 0 {
        let block = device.read_block(addr, 8)?;
        result.extend_from_slice(&block[..remaining]);
        pb.inc(remaining as u64);
    }

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
}

fn cmd_write(cli: &Cli, address: u32, data_hex: &str) -> Result<()> {
    let data =
        hex::decode(data_hex.replace(' ', "")).map_err(|e| anyhow!("Invalid hex string: {}", e))?;

    if data.is_empty() {
        bail!("No data to write");
    }

    let mut device = open_device(cli)?;

    let mut padded = data.clone();
    if padded.len() % 8 != 0 {
        padded.resize((padded.len() + 7) / 8 * 8, 0xFF);
    }

    device.write_block(address, &padded)?;
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

    let mut padded = data.clone();
    if padded.len() % 8 != 0 {
        padded.resize((padded.len() + 7) / 8 * 8, 0xFF);
    }

    let pb = ProgressBar::new(padded.len() as u64);
    pb.set_style(
        ProgressStyle::default_bar()
            .template("[{elapsed_precise}] [{bar:40.cyan/blue}] {bytes}/{total_bytes} ({eta})")?
            .progress_chars("#>-"),
    );

    let mut addr = address;
    let mut offset = 0;

    while offset < padded.len() {
        let chunk_len = std::cmp::min(BLOCK_SIZE, padded.len() - offset);
        device.write_block(addr, &padded[offset..offset + chunk_len])?;
        addr += chunk_len as u32;
        offset += chunk_len;
        pb.inc(chunk_len as u64);
    }

    pb.finish_with_message("done");
    println!("Loaded {} bytes", padded.len());

    if verify {
        println!("Verifying...");
        let pb = ProgressBar::new(padded.len() as u64);
        pb.set_style(
            ProgressStyle::default_bar()
                .template(
                    "[{elapsed_precise}] [{bar:40.green/blue}] {bytes}/{total_bytes} ({eta})",
                )?
                .progress_chars("#>-"),
        );

        let mut addr = address;
        let mut offset = 0;
        let mut errors = 0;

        while offset < padded.len() {
            let chunk_len = std::cmp::min(BLOCK_SIZE, padded.len() - offset);
            let readback = device.read_block(addr, chunk_len)?;

            for (i, (a, b)) in padded[offset..offset + chunk_len]
                .iter()
                .zip(readback.iter())
                .enumerate()
            {
                if a != b {
                    if errors < 10 {
                        eprintln!(
                            "Mismatch at 0x{:08x}: expected 0x{:02x}, got 0x{:02x}",
                            addr + i as u32,
                            a,
                            b
                        );
                    }
                    errors += 1;
                }
            }

            addr += chunk_len as u32;
            offset += chunk_len;
            pb.inc(chunk_len as u64);
        }

        pb.finish_with_message("done");

        if errors > 0 {
            bail!("Verification failed: {} byte(s) differ", errors);
        }
        println!("Verification passed!");
    }

    Ok(())
}

fn cmd_dump(cli: &Cli, file: &PathBuf, address: u32, length: u32) -> Result<()> {
    cmd_read(cli, address, length, Some(file.clone()))
}

fn cmd_configure(cli: &Cli, chip_db_path: &str, chip_name: &str) -> Result<()> {
    // Expand ~ in path
    let expanded = if chip_db_path.starts_with("~/") {
        let home = std::env::var("HOME").unwrap_or_else(|_| "/root".to_string());
        format!("{}/{}", home, &chip_db_path[2..])
    } else {
        chip_db_path.to_string()
    };
    let db_path = std::path::Path::new(&expanded);

    // Load chip database
    eprintln!("Loading chip database from {}...", db_path.display());
    let chips = chip::load_chip_db(db_path)?;
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
    eprintln!("  4-byte addr: {}", chip.supports_4byte);
    eprintln!("  Dual I/O:   {}", chip.supports_dual);
    eprintln!("  Quad I/O:   {}", chip.supports_quad);

    let erase_ops = chip.sector_erase_ops();
    for op in &erase_ops {
        eprintln!(
            "  Erase:      0x{:02X} ({} KB)",
            op.opcode,
            op.block_size / 1024
        );
    }

    // Generate SFDP table
    let sfdp_table = sfdp::generate_sfdp(chip);
    eprintln!("  SFDP table: {} bytes", sfdp_table.len());

    // Send to FPGA
    let mut device = open_device(cli)?;
    device.send_chip_config(chip, &sfdp_table)?;

    eprintln!("Configuration applied successfully.");
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

fn cmd_ft_list() -> Result<()> {
    let devices = libftd2xx::list_devices().context("Failed to enumerate FTDI devices")?;

    if devices.is_empty() {
        println!("No FTDI devices found");
        println!(
            "Hint: FT2232H has VID:PID {:04x}:{:04x}",
            FTDI_VID, FT2232H_PID
        );
        return Ok(());
    }

    println!("FTDI devices ({} found):", devices.len());
    for (i, dev) in devices.iter().enumerate() {
        println!("  [{}] {:?}", i, dev);
    }
    println!();
    println!("Use --ft-serial <SERIAL> to select a specific device.");
    println!("Channel A (\"Dual RS232-HS A\") is used by default for sync FIFO.");
    Ok(())
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() -> Result<()> {
    let cli = Cli::parse();

    match &cli.command {
        Commands::Version => cmd_version(&cli),
        Commands::Read {
            address,
            length,
            output,
        } => cmd_read(&cli, *address, *length, output.clone()),
        Commands::Write { address, data } => cmd_write(&cli, *address, data),
        Commands::Load {
            file,
            address,
            verify,
        } => cmd_load(&cli, file, *address, *verify),
        Commands::Dump {
            file,
            address,
            length,
        } => cmd_dump(&cli, file, *address, *length),
        Commands::Configure { chip_db, chip } => cmd_configure(&cli, chip_db, chip),
        Commands::Ports => cmd_ports(),
        Commands::FtList => cmd_ft_list(),
    }
}

mod hex {
    pub fn decode(s: impl AsRef<str>) -> Result<Vec<u8>, String> {
        let s = s.as_ref();
        if s.len() % 2 != 0 {
            return Err("Hex string must have even length".to_string());
        }

        (0..s.len())
            .step_by(2)
            .map(|i| {
                u8::from_str_radix(&s[i..i + 2], 16)
                    .map_err(|e| format!("Invalid hex at position {}: {}", i, e))
            })
            .collect()
    }
}
