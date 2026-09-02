use crate::device::{ConnectionKind, FlashDevice, Transport, UART_BAUD_RATE};
#[cfg(feature = "ftdi")]
use crate::device::{FT2232H_PID, FTDI_VID};
use crate::protocol::{CMD_VERSION, is_supported_protocol_version};
#[cfg(feature = "ftdi")]
use anyhow::anyhow;
use anyhow::{Context, Result, bail};
use serialport::SerialPort;
use std::io::{Read, Write};
use std::thread;
use std::time::Duration;

impl FlashDevice {
    pub(crate) fn open_serial(port_name: &str) -> Result<Self> {
        Self::new(
            SerialTransport::open(port_name)?,
            ConnectionKind::Uart,
            None,
        )
    }

    #[cfg(feature = "ftdi")]
    pub(crate) fn open_ft245(serial: Option<&str>) -> Result<Self> {
        Self::new(Ft245Transport::open(serial)?, ConnectionKind::Ft245, None)
    }
}

// ---------------------------------------------------------------------------
// UART transport (existing serial port path)
// ---------------------------------------------------------------------------

pub(crate) struct SerialTransport {
    port: Box<dyn SerialPort>,
}

impl SerialTransport {
    pub(crate) fn open(port_name: &str) -> Result<Self> {
        let mut port = serialport::new(port_name, UART_BAUD_RATE)
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
        let mut last_unsupported = None;
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

            // Accept only versions whose command semantics this tool knows.
            // Any other byte is noise or an incompatible device response.
            match last {
                Some(v) if is_supported_protocol_version(v) => {
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
                Some(v) if v != 0x00 && v != 0xff => last_unsupported = Some(v),
                _ => {} // keep trying
            }
        }
        match last_unsupported {
            Some(version) => bail!(
                "Failed to sync with FPGA on {port_name}: unsupported protocol version {version}"
            ),
            None => bail!(
                "Failed to sync with FPGA on {} after {} attempts (only saw 0x00/0xFF junk)",
                port_name,
                SYNC_ATTEMPTS
            ),
        }
    }
}

#[maybe_async::maybe_async(?Send)]
impl Transport for SerialTransport {
    async fn write_all(&mut self, data: &[u8]) -> Result<()> {
        self.port.write_all(data)?;
        Ok(())
    }

    async fn read(&mut self, buffer: &mut [u8]) -> Result<usize> {
        Ok(self.port.read(buffer)?)
    }

    async fn disconnect(&mut self) -> Result<()> {
        Ok(())
    }

    fn is_connected(&self) -> bool {
        true
    }
}

// ---------------------------------------------------------------------------
// FT245 transport -- ftdi-nusb backend (pure Rust, nusb)
// ---------------------------------------------------------------------------

#[cfg(feature = "ftdi")]
pub(crate) fn open_ft2232h(serial: Option<&str>) -> Result<ftdi_nusb::blocking::FtdiDevice> {
    if let Some(serial) = serial {
        let filter = ftdi_nusb::DeviceFilter::new(FTDI_VID, FT2232H_PID).serial(serial);
        return ftdi_nusb::blocking::FtdiDevice::open_with_filter(&filter, ftdi_nusb::Interface::A)
            .with_context(|| format!("No FT2232H with serial '{serial}'"));
    }

    // DeviceFilter matches the USB product string configured in ft2232h.conf;
    // the interface is selected separately below.
    let filter = ftdi_nusb::DeviceFilter::new(FTDI_VID, FT2232H_PID).description("NORbert FT245");
    ftdi_nusb::blocking::FtdiDevice::open_with_filter(&filter, ftdi_nusb::Interface::A)
        .or_else(|_| {
            ftdi_nusb::blocking::FtdiDevice::open_with_interface(
                FTDI_VID,
                FT2232H_PID,
                ftdi_nusb::Interface::A,
            )
        })
        .context("No FT2232H found")
}

#[cfg(feature = "ftdi")]
pub(crate) struct Ft245Transport {
    dev: ftdi_nusb::blocking::FtdiDevice,
}

#[cfg(feature = "ftdi")]
impl Ft245Transport {
    pub(crate) fn open(serial: Option<&str>) -> Result<Self> {
        // Open the FT2232H Channel A. The EEPROM must already be configured
        // for "245 FIFO" mode.
        let mut dev = open_ft2232h(serial)?;

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
#[maybe_async::maybe_async(?Send)]
impl Transport for Ft245Transport {
    async fn write_all(&mut self, data: &[u8]) -> Result<()> {
        self.dev.write_all(data).map_err(|e| anyhow!(e))?;
        Ok(())
    }

    async fn read(&mut self, buffer: &mut [u8]) -> Result<usize> {
        self.dev.read_data(buffer).map_err(|e| anyhow!(e))
    }

    async fn disconnect(&mut self) -> Result<()> {
        Ok(())
    }

    fn is_connected(&self) -> bool {
        true
    }
}
