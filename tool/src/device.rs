use crate::chip::{self, FlashChipExt};
use crate::protocol::*;
#[cfg(any(feature = "d2xx", feature = "ftdi"))]
use crate::transport::Ft245Transport;
use crate::transport::{
    BLOCK_SIZE, SDRAM_SIZE_BYTES, SerialTransport, Transport, UART_READ_BLOCK_SIZE,
};
use anyhow::{Context, Result, bail};
use std::mem::size_of;
use zerocopy::IntoBytes;

// ---------------------------------------------------------------------------
// FlashDevice -- protocol implementation over any transport
// ---------------------------------------------------------------------------

pub(crate) struct FlashDevice {
    transport: Box<dyn Transport>,
    read_block_size: usize,
}

fn validate_sdram_range(address: u32, length: usize, what: &str) -> Result<()> {
    let end = (address as u64)
        .checked_add(length as u64)
        .with_context(|| format!("{} range overflows address arithmetic", what))?;
    if end > SDRAM_SIZE_BYTES {
        bail!(
            "{} range 0x{:08x}..0x{:08x} exceeds {} MiB SDRAM backing store",
            what,
            address,
            end,
            SDRAM_SIZE_BYTES / (1024 * 1024)
        );
    }
    Ok(())
}

impl FlashDevice {
    pub(crate) fn open_serial(port_name: &str) -> Result<Self> {
        Ok(Self {
            transport: Box::new(SerialTransport::open(port_name)?),
            read_block_size: UART_READ_BLOCK_SIZE,
        })
    }

    #[cfg(any(feature = "d2xx", feature = "ftdi"))]
    pub(crate) fn open_ft245(serial: Option<&str>) -> Result<Self> {
        Ok(Self {
            transport: Box::new(Ft245Transport::open(serial)?),
            read_block_size: BLOCK_SIZE,
        })
    }

    pub(crate) fn get_version(&mut self) -> Result<u8> {
        self.transport.write_all(&[CMD_VERSION])?;
        self.read_ack("version")
    }

    /// Read one response byte, transparently skipping stray 0x00 bytes
    /// that the FT2232H 245 FIFO mode occasionally leaks through D2XX
    /// (the 2-byte USB IN modem status header).  Used by all single-
    /// byte-ack commands.
    pub(crate) fn read_ack(&mut self, context: &str) -> Result<u8> {
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

    fn expect_ack(&mut self, context: &str) -> Result<()> {
        let ack = self.read_ack(context)?;
        if ack != 0x01 {
            bail!("{}: unexpected response 0x{:02x}", context, ack);
        }
        Ok(())
    }

    /// Start SPI emulation.  ACK = 0x01.  Safe to call while running
    /// (no-op that still acks).
    pub(crate) fn start_emulation(&mut self) -> Result<()> {
        self.transport.write_all(&[CMD_START])?;
        self.expect_ack("start")
    }

    /// Stop SPI emulation.  ACK = 0x01.  Safe to call while stopped.
    ///
    /// Always deliverable: even when SPI emulation is actively driving
    /// the target, the FPGA's always-safe dispatcher picks up this
    /// command ahead of the usual SPI-idle gate.
    pub(crate) fn stop_emulation(&mut self) -> Result<()> {
        self.transport.write_all(&[CMD_STOP])?;
        self.expect_ack("stop")
    }

    /// Query the current emulation state.  Returns true when running.
    ///
    /// The FPGA encodes stopped as 0x02 (not 0x00) so that any leaked
    /// FT2232H modem-status zeros can be transparently skipped.
    pub(crate) fn status(&mut self) -> Result<bool> {
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
    pub(crate) fn with_emulation_stopped<F, R>(&mut self, f: F) -> Result<R>
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
        let operation = f(self);
        let restoration = if was_running {
            self.start_emulation()
        } else {
            Ok(())
        };

        match (operation, restoration) {
            (Ok(value), Ok(())) => {
                if was_running {
                    eprintln!("SPI emulation resumed.");
                }
                Ok(value)
            }
            (Ok(_), Err(resume_error)) => {
                Err(resume_error
                    .context("operation completed, but SPI emulation could not be resumed"))
            }
            (Err(operation_error), Ok(())) => Err(operation_error),
            (Err(operation_error), Err(resume_error)) => Err(operation_error.context(format!(
                "SPI emulation also could not be resumed: {resume_error:#}"
            ))),
        }
    }

    /// Stop emulation, run `f`, then start emulation unconditionally.
    /// Used by `load`, whose natural end state is "ready to serve".
    pub(crate) fn with_emulation_then_start<F, R>(&mut self, f: F) -> Result<R>
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
        let operation = f(self);
        let restoration = self.start_emulation();

        match (operation, restoration) {
            (Ok(value), Ok(())) => {
                eprintln!("SPI emulation started; target can now read the loaded data.");
                Ok(value)
            }
            (Ok(_), Err(start_error)) => {
                Err(start_error.context("load completed, but SPI emulation could not be started"))
            }
            (Err(operation_error), Ok(())) => Err(operation_error),
            (Err(operation_error), Err(start_error)) => Err(operation_error.context(format!(
                "SPI emulation also could not be started: {start_error:#}"
            ))),
        }
    }

    pub(crate) fn read_block(&mut self, address: u32, length: usize) -> Result<Vec<u8>> {
        if length == 0 || length > BLOCK_SIZE {
            bail!("Invalid length: must be 1-{}", BLOCK_SIZE);
        }
        if !address.is_multiple_of(8) {
            bail!("Address must be 8-byte aligned");
        }
        if !length.is_multiple_of(8) {
            bail!("Length must be a multiple of 8");
        }

        validate_sdram_range(address, length, "read")?;

        let addr_units = address / 8;
        let len_units = length / 8;

        // v3 header: cmd + 3 addr bytes + 2 len bytes (big-endian)
        let len_units = u16::try_from(len_units).context("read burst count exceeds 16 bits")?;
        let header =
            RamHeader::read(addr_units, len_units).context("read burst address exceeds 24 bits")?;

        self.transport.write_all(header.as_bytes())?;

        let mut buf = vec![0u8; length];
        self.transport.read_exact(&mut buf)?;
        Ok(buf)
    }

    pub(crate) fn write_block(&mut self, address: u32, data: &[u8]) -> Result<()> {
        if data.is_empty() || data.len() > BLOCK_SIZE {
            bail!("Invalid data length: must be 1-{}", BLOCK_SIZE);
        }
        if !address.is_multiple_of(8) {
            bail!("Address must be 8-byte aligned");
        }
        if !data.len().is_multiple_of(8) {
            bail!("Data length must be a multiple of 8");
        }

        validate_sdram_range(address, data.len(), "write")?;

        let addr_units = address / 8;
        let len_units = data.len() / 8;

        // Combine header and data into a single write to avoid
        // transfer gaps that could trigger the FPGA's idle timeout.
        // v3 header: cmd + 3 addr bytes + 2 len bytes (big-endian)
        let len_units = u16::try_from(len_units).context("write burst count exceeds 16 bits")?;
        let header = RamHeader::write(addr_units, len_units)
            .context("write burst address exceeds 24 bits")?;
        let mut buf = Vec::with_capacity(size_of::<RamHeader>() + data.len());
        buf.extend_from_slice(header.as_bytes());
        buf.extend_from_slice(data);
        self.transport.write_all(&buf)?;

        // Wait for completion byte (read_ack skips leaked FT2232H 0x00s).
        self.expect_ack("write")
    }

    pub(crate) fn read_with_progress(
        &mut self,
        address: u32,
        length: u32,
        mut progress: impl FnMut(usize),
    ) -> Result<Vec<u8>> {
        validate_sdram_range(address, length as usize, "read")?;
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
            progress(chunk_len);
        }

        while remaining >= 8 {
            let chunk_len = std::cmp::min(rbs, remaining / 8 * 8);
            let block = self.read_block(addr, chunk_len)?;
            result.extend_from_slice(&block);
            addr += chunk_len as u32;
            remaining -= chunk_len;
            progress(chunk_len);
        }

        if remaining > 0 {
            let block = self.read_block(addr, 8)?;
            result.extend_from_slice(&block[..remaining]);
            progress(remaining);
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
    pub(crate) fn send_chip_config(
        &mut self,
        chip: &chip::FlashChip,
        sfdp_table: &[u8],
    ) -> Result<()> {
        let jedec = chip.jedec_id_bytes();
        let erase_bursts = chip.chip_erase_bursts();
        let flags: u8 = if chip.supports_4byte() { 0x01 } else { 0x00 };

        let sfdp_len = sfdp_table.len();
        if sfdp_len > 128 {
            bail!("SFDP table too large: {} bytes (max 128)", sfdp_len);
        }
        if chip.total_size as u64 > SDRAM_SIZE_BYTES {
            bail!(
                "chip size {} bytes exceeds {} MiB SDRAM backing store",
                chip.total_size,
                SDRAM_SIZE_BYTES / (1024 * 1024)
            );
        }

        let sfdp_len = u8::try_from(sfdp_len).context("SFDP table length exceeds 8 bits")?;
        let header = ChipConfigHeader::new(jedec, flags, erase_bursts, sfdp_len)
            .context("chip erase burst count exceeds the protocol's 23-bit field")?;

        // Build the full command in one buffer to avoid USB transfer gaps.
        let mut buf = Vec::with_capacity(size_of::<ChipConfigHeader>() + sfdp_table.len());
        buf.extend_from_slice(header.as_bytes());
        buf.extend_from_slice(sfdp_table);
        self.transport.write_all(&buf)?;

        self.expect_ack("chip config")
    }

    /// Control the target flash #HOLD pin.
    ///
    /// When enabled, IO3 is driven LOW continuously, asserting #HOLD on
    /// the target flash and silencing it so NORbert can respond instead.
    /// Mutually exclusive with quad I/O.
    ///
    /// Protocol (CMD_HOLDCTL = 0x37):
    ///   Byte 0: 0x37
    ///   Byte 1: 0x01 = assert hold, 0x00 = release hold
    ///   Response: 0x01 on success.
    pub(crate) fn set_hold(&mut self, enable: bool) -> Result<()> {
        self.transport
            .write_all(ControlRequest::hold(enable).as_bytes())?;

        self.expect_ack("hold control")
    }

    pub(crate) fn log_start(&mut self) -> Result<()> {
        self.transport
            .write_all(ControlRequest::log(true).as_bytes())?;
        self.expect_ack("log start")
    }

    pub(crate) fn log_stop(&mut self) -> Result<()> {
        self.transport
            .write_all(ControlRequest::log(false).as_bytes())?;
        self.expect_ack("log stop")
    }

    /// Drain the logger ring FIFO in a single poll.
    ///
    /// Protocol (CMD_LOGPOLL = 0x3A):
    ///   Host:  0x3A
    ///   FPGA:  [escaped log byte]* 0xA0
    /// The response ends when an unescaped 0xA0 is seen.  Raw payload
    /// 0xA0 is encoded as 0xA5 0x00; raw 0xA5 is encoded as 0xA5 0x05.
    /// The FPGA caps each poll at 255 raw data bytes; any remaining log
    /// data is picked up on the next poll.
    pub(crate) fn log_poll(&mut self) -> Result<Vec<u8>> {
        self.transport.write_all(&[CMD_LOGPOLL])?;
        let mut out = Vec::with_capacity(256);
        loop {
            let mut byte = [0u8; 1];
            self.transport.read_exact(&mut byte)?;
            match byte[0] {
                LOG_POLL_TERMINATOR => return Ok(out),
                LOG_POLL_ESCAPE => {
                    let mut code = [0u8; 1];
                    self.transport.read_exact(&mut code)?;
                    match code[0] {
                        0x00 => out.push(LOG_POLL_TERMINATOR),
                        0x05 => out.push(LOG_POLL_ESCAPE),
                        other => bail!("LOGPOLL: invalid escape code 0x{:02x}", other),
                    }
                }
                b => out.push(b),
            }
            // Safety net: cap log poll responses so a misbehaving
            // device cannot lock the tool in this loop.  Should never
            // trip given the FPGA's LOG_POLL_MAX = 255 raw bytes plus
            // escapes and terminator.
            if out.len() > 1024 {
                bail!(
                    "log poll overran 1024 bytes without terminator; last byte 0x{:02x}",
                    byte[0]
                );
            }
        }
    }

    pub(crate) fn toctou_set(
        &mut self,
        index: u8,
        start: u32,
        mask: u32,
        replace: u32,
    ) -> Result<()> {
        let request = ToctouSetRequest::new(index, start, mask, replace)
            .context("TOCTOU addresses and masks must fit in 24 bits")?;
        self.transport.write_all(request.as_bytes())?;
        self.expect_ack("TOCTOU set")
    }

    pub(crate) fn toctou_arm(&mut self, index: u8) -> Result<()> {
        let request = ToctouIndexRequest::arm(index).context("trap index must be 0-3")?;
        self.transport.write_all(request.as_bytes())?;
        self.expect_ack("TOCTOU arm")
    }

    pub(crate) fn toctou_disarm(&mut self, index: u8) -> Result<()> {
        let request = ToctouIndexRequest::disarm(index).context("trap index must be 0-3")?;
        self.transport.write_all(request.as_bytes())?;
        self.expect_ack("TOCTOU disarm")
    }

    pub(crate) fn toctou_reset(&mut self, index: u8) -> Result<()> {
        let request = ToctouIndexRequest::reset(index).context("trap index must be 0-3")?;
        self.transport.write_all(request.as_bytes())?;
        self.expect_ack("TOCTOU reset")
    }

    pub(crate) fn toctou_reset_all(&mut self) -> Result<()> {
        self.transport
            .write_all(ControlRequest::toctou_reset_all().as_bytes())?;
        self.expect_ack("TOCTOU reset-all")
    }

    pub(crate) fn write(&mut self, address: u32, data: &[u8]) -> Result<()> {
        self.write_with_progress(address, data, |_| {})
    }

    pub(crate) fn write_with_progress(
        &mut self,
        address: u32,
        data: &[u8],
        mut progress: impl FnMut(usize),
    ) -> Result<()> {
        if data.is_empty() {
            bail!("No data to write");
        }
        validate_sdram_range(address, data.len(), "write")?;

        let start_offset = (address % 8) as usize;
        let aligned_address = address - start_offset as u32;
        let aligned_len = (start_offset + data.len()).div_ceil(8) * 8;
        let mut aligned = vec![0u8; aligned_len];

        // Preserve bytes outside the requested range in the first and last
        // bursts instead of silently overwriting them with padding.
        if start_offset != 0 {
            aligned[..8].copy_from_slice(&self.read_block(aligned_address, 8)?);
        }
        let end_offset = start_offset + data.len();
        if !end_offset.is_multiple_of(8) {
            let last_offset = aligned_len - 8;
            if last_offset != 0 || start_offset == 0 {
                let last_address = aligned_address + last_offset as u32;
                aligned[last_offset..].copy_from_slice(&self.read_block(last_address, 8)?);
            }
        }
        aligned[start_offset..end_offset].copy_from_slice(data);

        let total_blocks = aligned.len().div_ceil(BLOCK_SIZE);
        for (block_num, chunk) in aligned.chunks(BLOCK_SIZE).enumerate() {
            let block_address = aligned_address + (block_num * BLOCK_SIZE) as u32;
            self.write_block(block_address, chunk).with_context(|| {
                format!(
                    "Block {}/{} at addr 0x{:06x}",
                    block_num + 1,
                    total_blocks,
                    block_address
                )
            })?;

            let chunk_start = block_num * BLOCK_SIZE;
            let chunk_end = chunk_start + chunk.len();
            let written_start = chunk_start.max(start_offset);
            let written_end = chunk_end.min(end_offset);
            if written_end > written_start {
                progress(written_end - written_start);
            }
        }

        Ok(())
    }
}
