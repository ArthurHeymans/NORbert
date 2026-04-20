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
use std::time::Duration;
#[cfg(any(feature = "d2xx", feature = "ftdi"))]
use std::time::Instant;

const BAUD_RATE: u32 = 2_000_000;
// Max bursts per command: 16-bit len field (v3), 8 bytes per burst.
// 64KB blocks are unreliable -- the FPGA loses a byte during sustained
// 8192-burst writes (likely a glue/SDRAM timing race under refresh
// pressure).  16KB is the largest size that passes 10/10 stress tests.
const BLOCK_SIZE: usize = 16384; // 2048 bursts * 8 bytes

// UART read block size: must be smaller than BLOCK_SIZE because the
// glue module inhibits SDRAM refresh for the entire serial-path read
// duration.  At 2Mbaud (~200KB/s), a 16KB read takes ~82ms -- exceeding
// the SDRAM's 64ms refresh window and risking data loss.  4KB takes
// ~20ms, well within budget.  Writes are unaffected (data flows
// host-to-FPGA, SDRAM writes complete in microseconds).
const UART_READ_BLOCK_SIZE: usize = 4096; // 512 bursts * 8 bytes

const PROTOCOL_VERSION: u8 = 4;

// Protocol commands (shared between UART and FT245 transports)
const CMD_VERSION: u8 = 0x30;
const CMD_READ: u8 = 0x31;
const CMD_WRITE: u8 = 0x32;
const CMD_CHIPCONFIG: u8 = 0x33;
const CMD_START: u8 = 0x34; // Enable SPI emulation (protocol v4+)
const CMD_STOP: u8 = 0x35; // Disable SPI emulation (protocol v4+)
const CMD_STATUS: u8 = 0x36; // Query emulation state (protocol v4+)

// FT2232H USB identifiers
#[cfg(any(feature = "d2xx", feature = "ftdi"))]
const FTDI_VID: u16 = 0x0403;
#[cfg(any(feature = "d2xx", feature = "ftdi"))]
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

        // Sync with FPGA.  Two distinct glitch sources conspire here:
        //
        //   1. A fresh JTAG load of the FPGA briefly tri-states then
        //      redrives the UART TX pin; the USB-serial bridge decodes
        //      that transition as a byte -- typically 0xFF (line was
        //      momentarily high with no start bit).
        //   2. Opening /dev/ttyUSB* toggles DTR/RTS which can glitch
        //      the line a second time.
        //
        // Either stray can land in the host's tty buffer any time from
        // a few ms to several tens of ms after port open.  We defend in
        // two layers: a generous settle + drain, then an *active*
        // resync that sends CMD_VERSION and discards any non-version
        // bytes that appear before the real reply.
        thread::sleep(Duration::from_millis(20));

        let mut discard = [0u8; 256];
        port.set_timeout(Duration::from_millis(5))?;
        while port.read(&mut discard).unwrap_or(0) > 0 {}

        // Active resync: send CMD_VERSION, read everything that
        // arrives within the reply window, and accept the last byte
        // seen.  A stray 0xFF that arrived just before or during the
        // reply will appear *before* the version byte; the last byte
        // is then the FPGA's reply.  Retry a few times to cover a
        // stray that arrives *after* the reply (it would be seen as
        // the last byte on the first attempt, forcing a retry).
        const SYNC_ATTEMPTS: u32 = 5;
        for attempt in 0..SYNC_ATTEMPTS {
            port.write_all(&[CMD_VERSION])?;
            port.flush().ok();

            // 10 ms window covers FPGA command processing (a few µs)
            // plus any late-arriving stray bytes.
            thread::sleep(Duration::from_millis(10));

            let mut last: Option<u8> = None;
            port.set_timeout(Duration::from_millis(5))?;
            loop {
                let mut buf = [0u8; 1];
                match port.read_exact(&mut buf) {
                    Ok(()) => last = Some(buf[0]),
                    Err(_) => break,
                }
            }

            // 0x00 is a leaked modem-status zero; 0xFF is the usual
            // glitch byte.  Anything else is a plausible version reply
            // (versions 1..=127 are in range for this protocol).
            match last {
                Some(v) if v != 0x00 && v != 0xFF => {
                    // Double-check the line is now quiet.
                    port.set_timeout(Duration::from_millis(5))?;
                    while port.read(&mut discard).unwrap_or(0) > 0 {}
                    port.set_timeout(Duration::from_secs(2))?;
                    if attempt > 0 {
                        eprintln!(
                            "Serial resync: version 0x{:02x} after {} retries",
                            v,
                            attempt + 1
                        );
                    }
                    return Ok(Self { port });
                }
                _ => {} // keep trying
            }
        }
        bail!(
            "Failed to sync with FPGA on {} after {} attempts (only saw 0x00/0xFF junk)",
            port_name,
            SYNC_ATTEMPTS
        );
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
// FT245 transport -- D2XX backend (FTDI's proprietary driver)
// ---------------------------------------------------------------------------

#[cfg(feature = "d2xx")]
struct Ft245Transport {
    ft: libftd2xx::Ft2232h,
}

#[cfg(feature = "d2xx")]
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
            libftd2xx::Ft2232h::with_description("NORbert FT245 A")
                .context("No FT2232H found (looking for 'NORbert FT245 A')")?
        };

        ft.reset().context("FT2232H reset failed")?;
        ft.purge_all().context("FT2232H purge failed")?;

        // Set USB transfer size for bulk throughput
        ft.set_usb_parameters(65536)
            .context("Failed to set USB parameters")?;

        // Minimum latency timer -- each ACK/response byte sits in the
        // FT2232H TX FIFO until either a full USB packet fills or this
        // timer fires.  1ms is the FT2232H minimum.
        ft.set_latency_timer(Duration::from_millis(1))
            .context("Failed to set latency timer")?;

        // D2XX read/write timeouts (prevents infinite blocking)
        ft.set_timeouts(Duration::from_secs(5), Duration::from_secs(5))
            .context("Failed to set timeouts")?;

        // The USB reset can glitch the FT2232H data bus, injecting
        // garbage bytes into the FPGA's protocol parser.  Wait 5ms
        // (covers USB reset settling + FPGA idle timeout of ~546µs).
        thread::sleep(Duration::from_millis(5));
        ft.purge_all().ok();

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

#[cfg(feature = "d2xx")]
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
// FT245 transport -- rs-ftdi backend (pure Rust, nusb)
// ---------------------------------------------------------------------------

#[cfg(feature = "ftdi")]
struct Ft245Transport {
    dev: ftdi::FtdiDevice,
}

#[cfg(feature = "ftdi")]
impl Ft245Transport {
    fn open(serial: Option<&str>) -> Result<Self> {
        // Open the FT2232H Channel A.  The EEPROM must already be
        // configured for "245 FIFO" mode (same as the D2XX path).
        let mut dev = if let Some(sn) = serial {
            let filter = ftdi::DeviceFilter::new(FTDI_VID, FT2232H_PID).serial(sn);
            ftdi::FtdiDevice::open_with_filter(&filter, ftdi::Interface::A)
                .with_context(|| format!("No FT2232H with serial '{}'", sn))?
        } else {
            // Try to find by description first
            let filter =
                ftdi::DeviceFilter::new(FTDI_VID, FT2232H_PID).description("NORbert FT245");
            ftdi::FtdiDevice::open_with_filter(&filter, ftdi::Interface::A)
                .or_else(|_| {
                    // Fall back to opening any FT2232H on interface A
                    ftdi::FtdiDevice::open_with_interface(FTDI_VID, FT2232H_PID, ftdi::Interface::A)
                })
                .context("No FT2232H found")?
        };

        // Reset the FT2232H's internal state and flush FIFOs.
        dev.usb_reset().context("FT2232H reset failed")?;
        dev.flush_all().context("FT2232H flush failed")?;

        // Minimum latency timer -- each ACK/response byte sits in the
        // FT2232H TX FIFO until either a full USB packet fills or this
        // timer fires.  1ms is the FT2232H minimum.
        dev.set_latency_timer(1)
            .context("Failed to set latency timer")?;

        // Configure timeouts
        dev.set_read_timeout(Duration::from_secs(5));
        dev.set_write_timeout(Duration::from_secs(5));

        // Use large read chunks for throughput
        dev.set_read_chunksize(65536);
        dev.set_write_chunksize(65536);

        // The USB reset can glitch the FT2232H data bus, injecting
        // garbage bytes into the FPGA's protocol parser.  Wait for the
        // FPGA's idle timeout (~546µs at 120MHz) to reset the parser.
        // Use 5ms for margin (covers USB reset settling + idle timeout).
        // Then drain any residual response bytes.
        thread::sleep(Duration::from_millis(5));
        dev.flush_all().ok();

        let mut trash = [0u8; 4096];
        loop {
            match dev.read_data(&mut trash) {
                Ok(0) => break,
                Ok(_) => continue,
                Err(_) => break,
            }
        }

        Ok(Self { dev })
    }
}

#[cfg(feature = "ftdi")]
impl Transport for Ft245Transport {
    fn write_all(&mut self, data: &[u8]) -> Result<()> {
        self.dev.write_all(data).map_err(|e| anyhow!(e))?;
        Ok(())
    }

    fn read_exact(&mut self, buf: &mut [u8]) -> Result<()> {
        let mut pos = 0;
        let deadline = Instant::now() + Duration::from_secs(5);
        while pos < buf.len() {
            if Instant::now() > deadline {
                bail!("FT245 read timeout: got {} of {} bytes", pos, buf.len());
            }
            let n = self
                .dev
                .read_data(&mut buf[pos..])
                .map_err(|e| anyhow!(e))?;
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
    read_block_size: usize,
}

impl FlashDevice {
    fn open_serial(port_name: &str) -> Result<Self> {
        Ok(Self {
            transport: Box::new(SerialTransport::open(port_name)?),
            read_block_size: UART_READ_BLOCK_SIZE,
        })
    }

    #[cfg(any(feature = "d2xx", feature = "ftdi"))]
    fn open_ft245(serial: Option<&str>) -> Result<Self> {
        Ok(Self {
            transport: Box::new(Ft245Transport::open(serial)?),
            read_block_size: BLOCK_SIZE,
        })
    }

    fn get_version(&mut self) -> Result<u8> {
        self.transport.write_all(&[CMD_VERSION])?;
        let mut buf = [0u8; 1];
        self.transport.read_exact(&mut buf)?;
        Ok(buf[0])
    }

    /// Read one response byte, transparently skipping stray 0x00 bytes
    /// that the FT2232H 245 FIFO mode occasionally leaks through D2XX
    /// (the 2-byte USB IN modem status header).  Used by all single-
    /// byte-ack commands.
    fn read_ack(&mut self, context: &str) -> Result<u8> {
        let mut resp = [0u8; 1];
        let mut skipped = 0u32;
        loop {
            self.transport.read_exact(&mut resp)?;
            if resp[0] != 0x00 {
                return Ok(resp[0]);
            }
            skipped += 1;
            if skipped > 8 {
                bail!("{}: too many 0x00 bytes before ACK", context);
            }
        }
    }

    /// Start SPI emulation.  ACK = 0x01.  Safe to call while running
    /// (no-op that still acks).
    fn start_emulation(&mut self) -> Result<()> {
        self.transport.write_all(&[CMD_START])?;
        let ack = self.read_ack("start")?;
        if ack != 0x01 {
            bail!("start: unexpected response 0x{:02x}", ack);
        }
        Ok(())
    }

    /// Stop SPI emulation.  ACK = 0x01.  Safe to call while stopped.
    ///
    /// Always deliverable: even when SPI emulation is actively driving
    /// the target, the FPGA's always-safe dispatcher picks up this
    /// command ahead of the usual SPI-idle gate.
    fn stop_emulation(&mut self) -> Result<()> {
        self.transport.write_all(&[CMD_STOP])?;
        let ack = self.read_ack("stop")?;
        if ack != 0x01 {
            bail!("stop: unexpected response 0x{:02x}", ack);
        }
        Ok(())
    }

    /// Query the current emulation state.  Returns true when running.
    ///
    /// The FPGA encodes stopped as 0x02 (not 0x00) so that any leaked
    /// FT2232H modem-status zeros can be transparently skipped.
    fn status(&mut self) -> Result<bool> {
        self.transport.write_all(&[CMD_STATUS])?;
        let resp = self.read_ack("status")?;
        match resp {
            0x01 => Ok(true),  // running
            0x02 => Ok(false), // stopped
            other => bail!("status: unexpected response 0x{:02x}", other),
        }
    }

    /// Ensure emulation is stopped for the duration of `f`, then
    /// restore the prior state.  On v3 or older firmware (which has
    /// no START/STOP/STATUS opcodes) `f` runs unconditionally and the
    /// caller is responsible for ensuring the SPI bus is idle.
    ///
    /// State is restored on a best-effort basis even when `f` returns
    /// an error so a failed operation doesn't leave the emulator
    /// unexpectedly stopped.
    fn with_emulation_stopped<F, R>(&mut self, f: F) -> Result<R>
    where
        F: FnOnce(&mut Self) -> Result<R>,
    {
        let version = self.get_version()?;
        if version < 4 {
            eprintln!(
                "Note: FPGA protocol v{} has no START/STOP; assuming idle SPI bus.",
                version
            );
            return f(self);
        }

        let was_running = self.status()?;
        if was_running {
            eprintln!("Stopping SPI emulation for operation...");
            self.stop_emulation()?;
        }
        let result = f(self);
        if was_running {
            match self.start_emulation() {
                Ok(()) => eprintln!("SPI emulation resumed."),
                Err(e) => eprintln!("Warning: failed to resume SPI emulation: {}", e),
            }
        }
        result
    }

    /// Stop emulation, run `f`, then start emulation unconditionally.
    /// Used by `load`, whose natural end state is "ready to serve".
    fn with_emulation_then_start<F, R>(&mut self, f: F) -> Result<R>
    where
        F: FnOnce(&mut Self) -> Result<R>,
    {
        let version = self.get_version()?;
        if version < 4 {
            eprintln!(
                "Note: FPGA protocol v{} has no START/STOP; assuming idle SPI bus.",
                version
            );
            return f(self);
        }

        eprintln!("Stopping SPI emulation...");
        self.stop_emulation()?;
        let result = f(self);
        match self.start_emulation() {
            Ok(()) => eprintln!("SPI emulation started; target can now read the loaded data."),
            Err(e) => eprintln!("Warning: failed to start SPI emulation: {}", e),
        }
        result
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

        // v3 header: cmd + 3 addr bytes + 2 len bytes (big-endian)
        let cmd = [
            CMD_READ,
            ((addr_units >> 16) & 0xFF) as u8,
            ((addr_units >> 8) & 0xFF) as u8,
            (addr_units & 0xFF) as u8,
            ((len_units >> 8) & 0xFF) as u8,
            (len_units & 0xFF) as u8,
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
        // transfer gaps that could trigger the FPGA's idle timeout.
        // v3 header: cmd + 3 addr bytes + 2 len bytes (big-endian)
        let mut buf = Vec::with_capacity(6 + data.len());
        buf.extend_from_slice(&[
            CMD_WRITE,
            ((addr_units >> 16) & 0xFF) as u8,
            ((addr_units >> 8) & 0xFF) as u8,
            (addr_units & 0xFF) as u8,
            ((len_units >> 8) & 0xFF) as u8,
            (len_units & 0xFF) as u8,
        ]);
        buf.extend_from_slice(data);
        self.transport.write_all(&buf)?;

        // Wait for completion byte (read_ack skips leaked FT2232H 0x00s).
        let ack = self.read_ack("write")?;
        if ack != 0x01 {
            bail!(
                "Unexpected ACK: 0x{:02x} (expected 0x01, data[0..4]={:02x?})",
                ack,
                &data[..std::cmp::min(4, data.len())]
            );
        }
        Ok(())
    }

    #[allow(dead_code)]
    fn read(&mut self, address: u32, length: u32) -> Result<Vec<u8>> {
        let rbs = self.read_block_size;
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
            let chunk_len = std::cmp::min(rbs, remaining / 8 * 8);
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

        let ack = self.read_ack("chip config")?;
        if ack != 0x01 {
            bail!("Chip config failed: unexpected response 0x{:02x}", ack);
        }
        Ok(())
    }

    #[allow(dead_code)]
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

        let total_blocks = (padded.len() + BLOCK_SIZE - 1) / BLOCK_SIZE;
        let mut block_num = 0;
        while offset < padded.len() {
            let chunk_len = std::cmp::min(BLOCK_SIZE, padded.len() - offset);
            self.write_block(addr, &padded[offset..offset + chunk_len])
                .with_context(|| {
                    format!(
                        "Block {}/{} at addr 0x{:06x}",
                        block_num, total_blocks, addr
                    )
                })?;
            addr += chunk_len as u32;
            offset += chunk_len;
            block_num += 1;
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

    /// Use FT2232H FT245 async FIFO instead of UART
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

    /// Diagnostic: send test bytes and report what comes back
    Probe,

    /// Start SPI emulation (target can now read the loaded firmware)
    Start,

    /// Stop SPI emulation (required before RAM/config operations)
    Stop,

    /// Query whether SPI emulation is currently running
    Status,
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
        bail!("--ft245 requires the 'd2xx' or 'ftdi' feature (build with --features d2xx or --features ftdi)");
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
        let rbs = device.read_block_size;

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
            let chunk_len = std::cmp::min(rbs, remaining / 8 * 8);
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
    })
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

    device.with_emulation_stopped(|device| device.write_block(address, &padded))?;
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

    // Load is the one command whose natural end state is "ready to
    // serve", so the dance is stop -> write -> (verify) -> start.
    device.with_emulation_then_start(|device| {
        let pb = ProgressBar::new(padded.len() as u64);
        pb.set_style(
            ProgressStyle::default_bar()
                .template("[{elapsed_precise}] [{bar:40.cyan/blue}] {bytes}/{total_bytes} ({eta})")?
                .progress_chars("#>-"),
        );

        let mut addr = address;
        let mut offset = 0;
        let total_blocks = (padded.len() + BLOCK_SIZE - 1) / BLOCK_SIZE;
        let mut block_num = 0;

        while offset < padded.len() {
            let chunk_len = std::cmp::min(BLOCK_SIZE, padded.len() - offset);
            device
                .write_block(addr, &padded[offset..offset + chunk_len])
                .with_context(|| {
                    format!(
                        "Block {}/{} at addr 0x{:06x}",
                        block_num, total_blocks, addr
                    )
                })?;
            addr += chunk_len as u32;
            offset += chunk_len;
            block_num += 1;
            pb.inc(chunk_len as u64);
        }

        pb.finish_with_message("done");
        println!("Loaded {} bytes ({} blocks)", padded.len(), block_num);

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

            let rbs = device.read_block_size;
            while offset < padded.len() {
                let chunk_len = std::cmp::min(rbs, padded.len() - offset);
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
    })
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

    // Send to FPGA.  Must be stopped while CHIPCONFIG is processed,
    // otherwise spi_trx could read inconsistent cfg_* values mid-transfer.
    let mut device = open_device(cli)?;
    device.with_emulation_stopped(|device| device.send_chip_config(chip, &sfdp_table))?;

    eprintln!("Configuration applied successfully.");
    Ok(())
}

// ---------------------------------------------------------------------------
// Probe command -- D2XX backend
// ---------------------------------------------------------------------------

#[cfg(feature = "d2xx")]
fn cmd_probe(cli: &Cli) -> Result<()> {
    use libftd2xx::FtdiCommon;

    // We need raw access to the FT2232H, not the Transport abstraction,
    // because we want short timeouts per-probe and queue_status checks.
    eprintln!("Opening FT2232H for probe (D2XX)...");
    let serial = cli.ft_serial.as_deref();
    let mut ft = if let Some(sn) = serial {
        libftd2xx::Ft2232h::with_serial_number(sn)
            .with_context(|| format!("No FT2232H with serial '{}'", sn))?
    } else {
        libftd2xx::Ft2232h::with_description("NORbert FT245 A")
            .context("No FT2232H found (looking for 'NORbert FT245 A')")?
    };

    ft.reset().context("FT2232H reset failed")?;
    ft.purge_all().context("FT2232H purge failed")?;
    ft.set_usb_parameters(65536)?;
    ft.set_latency_timer(Duration::from_millis(2))?;
    ft.set_timeouts(Duration::from_millis(500), Duration::from_secs(5))?;

    // Drain stale data
    let mut trash = [0u8; 4096];
    loop {
        let n = ft.queue_status()?;
        if n == 0 {
            break;
        }
        let to_read = std::cmp::min(n, trash.len());
        let _ = ft.read(&mut trash[..to_read]);
    }

    // Test bytes: mix of valid commands, diagnostics, and non-commands
    let probes: Vec<(u8, &str, Option<u8>)> = vec![
        (0x30, "CMD_VERSION", Some(0x02)),
        (0x00, "NOP / zero", None),
        (0x55, "non-command 0x55", None),
        (0xAA, "non-command 0xAA", None),
        (0xFF, "non-command 0xFF", None),
    ];

    println!("FT245 probe (D2XX): send 1 byte, wait 200ms, read back\n");
    println!(
        "{:<30} {:>4}   {:>8}  {}",
        "Test", "Sent", "Expected", "Response"
    );
    println!("{}", "-".repeat(70));

    for (byte, label, expected) in &probes {
        // Purge before each probe
        ft.purge_all()?;
        thread::sleep(Duration::from_millis(50));

        // Drain anything residual
        loop {
            let n = ft.queue_status()?;
            if n == 0 {
                break;
            }
            let to_read = std::cmp::min(n, trash.len());
            let _ = ft.read(&mut trash[..to_read]);
        }

        // Send the byte
        ft.write_all(&[*byte])?;

        // Wait for response
        thread::sleep(Duration::from_millis(200));

        // Check what came back
        let n = ft.queue_status()?;
        let exp_str = match expected {
            Some(v) => format!("0x{:02X}", v),
            None => "none".to_string(),
        };
        if n == 0 {
            let status = if expected.is_none() {
                "OK"
            } else {
                "FAIL(none)"
            };
            println!(
                "{:<30} 0x{:02X}   {:>8}  (no response) {}",
                label, byte, exp_str, status
            );
        } else {
            let to_read = std::cmp::min(n, 32);
            let mut buf = vec![0u8; to_read];
            let got = ft.read(&mut buf)?;
            buf.truncate(got);
            let hex_str: Vec<String> = buf.iter().map(|b| format!("0x{:02X}", b)).collect();
            let status = match expected {
                Some(v) if got == 1 && buf[0] == *v => "OK",
                Some(_) if got == 1 && buf[0] == *byte => "FAIL(echo)",
                _ => "FAIL",
            };
            println!(
                "{:<30} 0x{:02X}   {:>8}  {} bytes: [{}] {}",
                label,
                byte,
                exp_str,
                got,
                hex_str.join(", "),
                status,
            );
        }
    }

    // Bonus: send multiple bytes at once and see what comes back
    println!("\n--- Multi-byte test ---");
    ft.purge_all()?;
    thread::sleep(Duration::from_millis(50));
    loop {
        let n = ft.queue_status()?;
        if n == 0 {
            break;
        }
        let to_read = std::cmp::min(n, trash.len());
        let _ = ft.read(&mut trash[..to_read]);
    }

    let multi = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];
    ft.write_all(&multi)?;
    thread::sleep(Duration::from_millis(200));
    let n = ft.queue_status()?;
    if n == 0 {
        println!("Sent {:?}: (no response)", multi);
    } else {
        let to_read = std::cmp::min(n, 64);
        let mut buf = vec![0u8; to_read];
        let got = ft.read(&mut buf)?;
        buf.truncate(got);
        let hex_str: Vec<String> = buf.iter().map(|b| format!("0x{:02X}", b)).collect();
        let is_echo = buf == multi[..got];
        println!(
            "Sent {} bytes: {:02X?}\nGot  {} bytes: [{}]{}",
            multi.len(),
            multi,
            got,
            hex_str.join(", "),
            if is_echo { " << EXACT ECHO" } else { "" },
        );
    }

    Ok(())
}

// ---------------------------------------------------------------------------
// Probe command -- rs-ftdi backend
// ---------------------------------------------------------------------------

#[cfg(all(feature = "ftdi", not(feature = "d2xx")))]
fn cmd_probe(cli: &Cli) -> Result<()> {
    // Open FT2232H via rs-ftdi
    eprintln!("Opening FT2232H for probe (rs-ftdi)...");
    let serial = cli.ft_serial.as_deref();
    let mut dev = if let Some(sn) = serial {
        let filter = ftdi::DeviceFilter::new(FTDI_VID, FT2232H_PID).serial(sn);
        ftdi::FtdiDevice::open_with_filter(&filter, ftdi::Interface::A)
            .with_context(|| format!("No FT2232H with serial '{}'", sn))?
    } else {
        let filter = ftdi::DeviceFilter::new(FTDI_VID, FT2232H_PID).description("NORbert FT245");
        ftdi::FtdiDevice::open_with_filter(&filter, ftdi::Interface::A)
            .or_else(|_| {
                ftdi::FtdiDevice::open_with_interface(FTDI_VID, FT2232H_PID, ftdi::Interface::A)
            })
            .context("No FT2232H found")?
    };

    dev.usb_reset().context("FT2232H reset failed")?;
    dev.flush_all().context("FT2232H flush failed")?;
    dev.set_latency_timer(2)?;
    dev.set_read_timeout(Duration::from_millis(500));
    dev.set_write_timeout(Duration::from_secs(5));

    // Drain stale data
    let mut trash = [0u8; 4096];
    loop {
        match dev.read_data(&mut trash) {
            Ok(0) => break,
            Ok(_) => continue,
            Err(_) => break,
        }
    }

    // Test bytes
    let probes: Vec<(u8, &str, Option<u8>)> = vec![
        (0x30, "CMD_VERSION", Some(0x02)),
        (0x00, "NOP / zero", None),
        (0x55, "non-command 0x55", None),
        (0xAA, "non-command 0xAA", None),
        (0xFF, "non-command 0xFF", None),
    ];

    println!("FT245 probe (rs-ftdi): send 1 byte, wait 200ms, read back\n");
    println!(
        "{:<30} {:>4}   {:>8}  {}",
        "Test", "Sent", "Expected", "Response"
    );
    println!("{}", "-".repeat(70));

    for (byte, label, expected) in &probes {
        // Flush before each probe
        dev.flush_all().ok();
        thread::sleep(Duration::from_millis(50));

        // Drain
        loop {
            match dev.read_data(&mut trash) {
                Ok(0) => break,
                Ok(_) => continue,
                Err(_) => break,
            }
        }

        // Send the byte
        dev.write_all(&[*byte]).map_err(|e| anyhow!(e))?;

        // Wait for response
        thread::sleep(Duration::from_millis(200));

        // Check what came back
        let mut buf = [0u8; 32];
        let got = dev.read_data(&mut buf).unwrap_or(0);
        let exp_str = match expected {
            Some(v) => format!("0x{:02X}", v),
            None => "none".to_string(),
        };
        if got == 0 {
            let status = if expected.is_none() {
                "OK"
            } else {
                "FAIL(none)"
            };
            println!(
                "{:<30} 0x{:02X}   {:>8}  (no response) {}",
                label, byte, exp_str, status
            );
        } else {
            let hex_str: Vec<String> = buf[..got].iter().map(|b| format!("0x{:02X}", b)).collect();
            let status = match expected {
                Some(v) if got == 1 && buf[0] == *v => "OK",
                Some(_) if got == 1 && buf[0] == *byte => "FAIL(echo)",
                _ => "FAIL",
            };
            println!(
                "{:<30} 0x{:02X}   {:>8}  {} bytes: [{}] {}",
                label,
                byte,
                exp_str,
                got,
                hex_str.join(", "),
                status,
            );
        }
    }

    // Multi-byte test
    println!("\n--- Multi-byte test ---");
    dev.flush_all().ok();
    thread::sleep(Duration::from_millis(50));
    loop {
        match dev.read_data(&mut trash) {
            Ok(0) => break,
            Ok(_) => continue,
            Err(_) => break,
        }
    }

    let multi = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];
    dev.write_all(&multi).map_err(|e| anyhow!(e))?;
    thread::sleep(Duration::from_millis(200));
    let mut buf = [0u8; 64];
    let got = dev.read_data(&mut buf).unwrap_or(0);
    if got == 0 {
        println!("Sent {:?}: (no response)", multi);
    } else {
        let hex_str: Vec<String> = buf[..got].iter().map(|b| format!("0x{:02X}", b)).collect();
        let is_echo = &buf[..got] == &multi[..got];
        println!(
            "Sent {} bytes: {:02X?}\nGot  {} bytes: [{}]{}",
            multi.len(),
            multi,
            got,
            hex_str.join(", "),
            if is_echo { " << EXACT ECHO" } else { "" },
        );
    }

    Ok(())
}

// ---------------------------------------------------------------------------
// Probe command -- no FT245 backend
// ---------------------------------------------------------------------------

#[cfg(not(any(feature = "d2xx", feature = "ftdi")))]
fn cmd_probe(_cli: &Cli) -> Result<()> {
    bail!("Probe requires the 'd2xx' or 'ftdi' feature");
}

// ---------------------------------------------------------------------------
// FT-list command
// ---------------------------------------------------------------------------

#[cfg(feature = "d2xx")]
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
    println!("Channel A is used by default for async FIFO.");
    Ok(())
}

#[cfg(all(feature = "ftdi", not(feature = "d2xx")))]
fn cmd_ft_list() -> Result<()> {
    let devices = ftdi::find_devices(FTDI_VID, FT2232H_PID)
        .map_err(|e| anyhow!(e))
        .context("Failed to enumerate FTDI devices")?;

    if devices.is_empty() {
        println!("No FT2232H devices found");
        println!(
            "Hint: FT2232H has VID:PID {:04x}:{:04x}",
            FTDI_VID, FT2232H_PID
        );
        return Ok(());
    }

    println!("FT2232H devices ({} found):", devices.len());
    for (i, dev) in devices.iter().enumerate() {
        println!(
            "  [{}] bus={} addr={} vid={:04x} pid={:04x}",
            i,
            dev.busnum(),
            dev.device_address(),
            dev.vendor_id(),
            dev.product_id(),
        );
    }
    println!();
    println!("Use --ft-serial <SERIAL> to select a specific device.");
    println!("Channel A is used by default for async FIFO.");
    Ok(())
}

#[cfg(not(any(feature = "d2xx", feature = "ftdi")))]
fn cmd_ft_list() -> Result<()> {
    bail!("ft-list requires the 'd2xx' or 'ftdi' feature");
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
        Commands::Probe => cmd_probe(&cli),
        Commands::Start => cmd_start(&cli),
        Commands::Stop => cmd_stop(&cli),
        Commands::Status => cmd_status(&cli),
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
