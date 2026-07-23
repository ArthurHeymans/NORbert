use crate::protocol::CMD_VERSION;
#[cfg(all(feature = "ftdi", not(feature = "d2xx")))]
use anyhow::anyhow;
use anyhow::{Context, Result, bail};
use serialport::SerialPort;
use std::io::{Read, Write};
use std::thread;
use std::time::Duration;
#[cfg(any(feature = "d2xx", feature = "ftdi"))]
use std::time::Instant;

const BAUD_RATE: u32 = 2_000_000;
// Max bursts per command: 16-bit len field (v3), 8 bytes per burst.
// 64KB blocks are unreliable -- the FPGA loses a byte during sustained
// 8192-burst writes (likely a glue/SDRAM timing race under refresh
// pressure).  16KB is the largest size that passes 10/10 stress tests.
pub(crate) const BLOCK_SIZE: usize = 16384; // 2048 bursts * 8 bytes

// UART read block size: must be smaller than BLOCK_SIZE because the
// glue module inhibits SDRAM refresh for the entire serial-path read
// duration.  At 2Mbaud (~200KB/s), a 16KB read takes ~82ms -- exceeding
// the SDRAM's 64ms refresh window and risking data loss.  4KB takes
// ~20ms, well within budget.  Writes are unaffected (data flows
// host-to-FPGA, SDRAM writes complete in microseconds).
pub(crate) const UART_READ_BLOCK_SIZE: usize = 4096; // 512 bursts * 8 bytes
pub(crate) const SDRAM_SIZE_BYTES: u64 = 64 * 1024 * 1024;

// FT2232H USB identifiers
#[cfg(any(feature = "d2xx", feature = "ftdi"))]
pub(crate) const FTDI_VID: u16 = 0x0403;
#[cfg(any(feature = "d2xx", feature = "ftdi"))]
pub(crate) const FT2232H_PID: u16 = 0x6010;

// ---------------------------------------------------------------------------
// Transport trait -- abstracts UART serial vs FT245 FIFO
// ---------------------------------------------------------------------------

pub(crate) trait Transport {
    fn write_all(&mut self, data: &[u8]) -> Result<()>;
    fn read_exact(&mut self, buf: &mut [u8]) -> Result<()>;
}

// ---------------------------------------------------------------------------
// UART transport (existing serial port path)
// ---------------------------------------------------------------------------

pub(crate) struct SerialTransport {
    port: Box<dyn SerialPort>,
}

impl SerialTransport {
    pub(crate) fn open(port_name: &str) -> Result<Self> {
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
pub(crate) struct Ft245Transport {
    ft: libftd2xx::Ft2232h,
}

#[cfg(feature = "d2xx")]
impl Ft245Transport {
    pub(crate) fn open(serial: Option<&str>) -> Result<Self> {
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

#[cfg(all(feature = "ftdi", not(feature = "d2xx")))]
pub(crate) struct Ft245Transport {
    dev: ftdi::FtdiDevice,
}

#[cfg(all(feature = "ftdi", not(feature = "d2xx")))]
impl Ft245Transport {
    pub(crate) fn open(serial: Option<&str>) -> Result<Self> {
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

#[cfg(all(feature = "ftdi", not(feature = "d2xx")))]
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
