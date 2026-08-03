use crate::protocol::*;
use anyhow::{Context, Result, bail};
use std::mem::size_of;
use zerocopy::IntoBytes;

#[cfg(any(feature = "ftdi", feature = "wasm"))]
pub(crate) const FTDI_VID: u16 = 0x0403;
#[cfg(any(feature = "ftdi", feature = "wasm"))]
pub(crate) const FT2232H_PID: u16 = 0x6010;
pub(crate) const UART_BAUD_RATE: u32 = 2_000_000;

// 64 KiB transfers are unreliable in the FPGA glue/SDRAM path. Keep FT245
// protocol requests at or below the largest size that passes stress tests.
#[cfg(any(feature = "ftdi", feature = "wasm", test))]
const FT245_READ_BLOCK_SIZE: usize = 16_384;

// UART reads must complete inside the SDRAM refresh window. At 2 Mbaud a
// 4 KiB read takes about 20 ms, leaving adequate margin.
const UART_READ_BLOCK_SIZE: usize = 4_096;
const MAX_WRITE_BLOCK_SIZE: usize = 16_384;
pub(crate) const SDRAM_SIZE_BYTES: u64 = 64 * 1024 * 1024;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ConnectionKind {
    #[cfg(any(feature = "ftdi", feature = "wasm", test))]
    Ft245,
    Uart,
}

impl ConnectionKind {
    fn read_block_size(self) -> usize {
        match self {
            #[cfg(any(feature = "ftdi", feature = "wasm", test))]
            Self::Ft245 => FT245_READ_BLOCK_SIZE,
            Self::Uart => UART_READ_BLOCK_SIZE,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct ProtocolCapabilities {
    pub(crate) version: u8,
    pub(crate) emulation_control: bool,
    pub(crate) hold_control: bool,
    pub(crate) activity_log: bool,
    pub(crate) toctou: bool,
}

impl ProtocolCapabilities {
    fn from_version(version: u8) -> Result<Self> {
        if !is_supported_protocol_version(version) {
            bail!(
                "unsupported protocol version {version}; supported versions are {MIN_SUPPORTED_PROTOCOL_VERSION} through {PROTOCOL_VERSION}"
            );
        }
        Ok(Self {
            version,
            emulation_control: supports_emulation_control(version),
            hold_control: supports_hold_control(version),
            activity_log: supports_activity_log(version),
            toctou: supports_toctou(version),
        })
    }
}

#[cfg_attr(feature = "cli", allow(dead_code))]
#[maybe_async::maybe_async(?Send)]
pub(crate) trait Transport {
    async fn write_all(&mut self, data: &[u8]) -> Result<()>;
    async fn read(&mut self, buffer: &mut [u8]) -> Result<usize>;
    async fn disconnect(&mut self) -> Result<()>;
    fn is_connected(&self) -> bool;
}

pub(crate) struct FlashDevice {
    transport: Box<dyn Transport>,
    read_block_size: usize,
    capabilities: Option<ProtocolCapabilities>,
}

pub(crate) struct WriteSession<'a> {
    device: &'a mut FlashDevice,
    next_address: u32,
    remaining: usize,
    was_running: bool,
}

pub(crate) fn validate_sdram_range(address: u32, length: usize, what: &str) -> Result<()> {
    let end = (address as u64)
        .checked_add(length as u64)
        .with_context(|| format!("{what} range overflows address arithmetic"))?;
    if end > SDRAM_SIZE_BYTES {
        bail!(
            "{what} range 0x{address:08x}..0x{end:08x} exceeds {} MiB SDRAM backing store",
            SDRAM_SIZE_BYTES / (1024 * 1024)
        );
    }
    Ok(())
}

#[cfg_attr(feature = "cli", allow(dead_code))]
#[maybe_async::maybe_async(?Send)]
impl FlashDevice {
    pub(crate) fn new<T>(
        transport: T,
        connection: ConnectionKind,
        known_version: Option<u8>,
    ) -> Result<Self>
    where
        T: Transport + 'static,
    {
        let capabilities = known_version
            .map(ProtocolCapabilities::from_version)
            .transpose()?;
        Ok(Self {
            transport: Box::new(transport),
            read_block_size: connection.read_block_size(),
            capabilities,
        })
    }

    pub(crate) fn is_connected(&self) -> bool {
        self.transport.is_connected()
    }

    pub(crate) async fn disconnect(&mut self) -> Result<()> {
        self.transport.disconnect().await
    }

    pub(crate) async fn capabilities(&mut self) -> Result<ProtocolCapabilities> {
        if let Some(capabilities) = self.capabilities {
            return Ok(capabilities);
        }

        self.transport.write_all(&[CMD_VERSION]).await?;
        let version = self.read_ack("version").await?;
        let capabilities = ProtocolCapabilities::from_version(version)?;
        self.capabilities = Some(capabilities);
        Ok(capabilities)
    }

    pub(crate) async fn get_version(&mut self) -> Result<u8> {
        Ok(self.capabilities().await?.version)
    }

    async fn require_capability(
        &mut self,
        name: &str,
        supported: impl FnOnce(ProtocolCapabilities) -> bool,
    ) -> Result<()> {
        let capabilities = self.capabilities().await?;
        if supported(capabilities) {
            Ok(())
        } else {
            bail!(
                "{name} is unavailable with protocol version {}",
                capabilities.version
            )
        }
    }

    async fn read_exact(&mut self, buffer: &mut [u8]) -> Result<()> {
        let mut offset = 0;
        while offset < buffer.len() {
            let count = self.transport.read(&mut buffer[offset..]).await?;
            if count == 0 {
                continue;
            }
            if count > buffer.len() - offset {
                bail!("transport returned more bytes than requested");
            }
            offset += count;
        }
        Ok(())
    }

    /// Read one response byte while tolerating the known startup/status noise
    /// bytes emitted by the UART and FT2232H paths.
    pub(crate) async fn read_ack(&mut self, context: &str) -> Result<u8> {
        let mut response = [0u8; 1];
        for _ in 0..=8 {
            self.read_exact(&mut response).await?;
            if response[0] != 0x00 && response[0] != 0xff {
                return Ok(response[0]);
            }
        }
        bail!("{context}: too many synchronization bytes before ACK")
    }

    async fn expect_ack(&mut self, context: &str) -> Result<()> {
        let response = self.read_ack(context).await?;
        if response == 0x01 {
            Ok(())
        } else {
            bail!("{context}: unexpected response 0x{response:02x}")
        }
    }

    pub(crate) async fn start_emulation(&mut self) -> Result<()> {
        self.require_capability("emulation control", |c| c.emulation_control)
            .await?;
        self.command_with_ack(CMD_START, "start").await
    }

    pub(crate) async fn stop_emulation(&mut self) -> Result<()> {
        self.require_capability("emulation control", |c| c.emulation_control)
            .await?;
        self.command_with_ack(CMD_STOP, "stop").await
    }

    pub(crate) async fn status(&mut self) -> Result<bool> {
        self.require_capability("emulation status", |c| c.emulation_control)
            .await?;
        self.transport.write_all(&[CMD_STATUS]).await?;
        match self.read_ack("status").await? {
            0x01 => Ok(true),
            0x02 => Ok(false),
            response => bail!("status: unexpected response 0x{response:02x}"),
        }
    }

    pub(crate) async fn prepare_stopped(&mut self) -> Result<bool> {
        let capabilities = self.capabilities().await?;
        if !capabilities.emulation_control {
            return Ok(false);
        }
        let was_running = self.status().await?;
        if was_running {
            self.stop_emulation().await?;
        }
        Ok(was_running)
    }

    pub(crate) async fn finish_stopped<T>(
        &mut self,
        was_running: bool,
        operation: Result<T>,
    ) -> Result<T> {
        let restoration = if was_running {
            self.start_emulation().await
        } else {
            Ok(())
        };
        match (operation, restoration) {
            (Ok(value), Ok(())) => Ok(value),
            (Ok(_), Err(error)) => {
                Err(error.context("operation completed, but emulation could not be resumed"))
            }
            (Err(error), Ok(())) => Err(error),
            (Err(error), Err(resume_error)) => Err(error.context(format!(
                "emulation also could not be resumed: {resume_error:#}"
            ))),
        }
    }

    pub(crate) async fn read(&mut self, address: u32, length: u32) -> Result<Vec<u8>> {
        self.read_with_progress(address, length, |_| Ok(())).await
    }

    pub(crate) async fn read_with_progress(
        &mut self,
        address: u32,
        length: u32,
        mut progress: impl FnMut(usize) -> Result<()>,
    ) -> Result<Vec<u8>> {
        let mut result = Vec::with_capacity(length as usize);
        self.read_chunks(address, length, |_, chunk| {
            result.extend_from_slice(chunk);
            progress(chunk.len())
        })
        .await?;
        Ok(result)
    }

    pub(crate) async fn read_chunks(
        &mut self,
        address: u32,
        length: u32,
        mut on_chunk: impl FnMut(u32, &[u8]) -> Result<()>,
    ) -> Result<()> {
        validate_sdram_range(address, length as usize, "read")?;
        let was_running = self.prepare_stopped().await?;
        let operation = self.read_raw_chunks(address, length, &mut on_chunk).await;
        self.finish_stopped(was_running, operation).await
    }

    #[cfg(feature = "cli")]
    pub(crate) async fn read_raw_with_progress(
        &mut self,
        address: u32,
        length: u32,
        mut progress: impl FnMut(usize) -> Result<()>,
    ) -> Result<Vec<u8>> {
        let mut result = Vec::with_capacity(length as usize);
        self.read_raw_chunks(address, length, |_, chunk| {
            result.extend_from_slice(chunk);
            progress(chunk.len())
        })
        .await?;
        Ok(result)
    }

    async fn read_raw_chunks(
        &mut self,
        address: u32,
        length: u32,
        mut on_chunk: impl FnMut(u32, &[u8]) -> Result<()>,
    ) -> Result<()> {
        validate_sdram_range(address, length as usize, "read")?;
        let mut current = address;
        let mut remaining = length as usize;

        let start_offset = (current % 8) as usize;
        if start_offset != 0 {
            let block = self.read_block(current - start_offset as u32, 8).await?;
            let count = (8 - start_offset).min(remaining);
            on_chunk(current, &block[start_offset..start_offset + count])?;
            current += count as u32;
            remaining -= count;
        }

        while remaining >= 8 {
            let count = self.read_block_size.min(remaining / 8 * 8);
            let block = self.read_block(current, count).await?;
            on_chunk(current, &block)?;
            current += count as u32;
            remaining -= count;
        }

        if remaining != 0 {
            let block = self.read_block(current, 8).await?;
            on_chunk(current, &block[..remaining])?;
        }
        Ok(())
    }

    pub(crate) async fn write(&mut self, address: u32, data: &[u8]) -> Result<()> {
        self.write_with_progress(address, data, |_| Ok(())).await
    }

    pub(crate) async fn begin_write(
        &mut self,
        address: u32,
        length: usize,
    ) -> Result<WriteSession<'_>> {
        if length == 0 {
            bail!("no data to write");
        }
        validate_sdram_range(address, length, "write")?;
        let was_running = self.prepare_stopped().await?;
        Ok(WriteSession {
            device: self,
            next_address: address,
            remaining: length,
            was_running,
        })
    }

    pub(crate) async fn write_with_progress(
        &mut self,
        address: u32,
        data: &[u8],
        mut progress: impl FnMut(usize) -> Result<()>,
    ) -> Result<()> {
        let mut session = self.begin_write(address, data.len()).await?;
        let operation = session.write_chunk(data, &mut progress).await;
        session.finish(operation).await
    }

    pub(crate) async fn write_raw_with_progress(
        &mut self,
        address: u32,
        data: &[u8],
        mut progress: impl FnMut(usize) -> Result<()>,
    ) -> Result<()> {
        if data.is_empty() {
            bail!("no data to write");
        }
        validate_sdram_range(address, data.len(), "write")?;

        let mut current_address = address;
        let mut remaining = data;
        let mut operation_number = 0usize;
        let start_offset = (address % 8) as usize;
        let initial_bytes = if start_offset == 0 {
            0
        } else {
            (8 - start_offset).min(data.len())
        };
        let bytes_after_initial = data.len() - initial_bytes;
        let aligned_bytes = bytes_after_initial / 8 * 8;
        let total_operations = usize::from(initial_bytes != 0)
            + aligned_bytes.div_ceil(MAX_WRITE_BLOCK_SIZE)
            + usize::from(bytes_after_initial != aligned_bytes);

        if start_offset != 0 {
            let aligned_address = current_address - start_offset as u32;
            let count = (8 - start_offset).min(remaining.len());
            let mut burst = self.read_block(aligned_address, 8).await?;
            burst[start_offset..start_offset + count].copy_from_slice(&remaining[..count]);
            self.write_block(aligned_address, &burst).await?;
            current_address += count as u32;
            remaining = &remaining[count..];
            progress(count)?;
            operation_number += 1;
        }

        while remaining.len() >= 8 {
            let count = MAX_WRITE_BLOCK_SIZE.min(remaining.len() / 8 * 8);
            self.write_block(current_address, &remaining[..count])
                .await
                .with_context(|| {
                    format!(
                        "operation {}/{} at address 0x{:06x}",
                        operation_number + 1,
                        total_operations.max(operation_number + 1),
                        current_address
                    )
                })?;
            current_address += count as u32;
            remaining = &remaining[count..];
            progress(count)?;
            operation_number += 1;
        }

        if !remaining.is_empty() {
            let mut burst = self.read_block(current_address, 8).await?;
            burst[..remaining.len()].copy_from_slice(remaining);
            self.write_block(current_address, &burst).await?;
            progress(remaining.len())?;
        }
        Ok(())
    }

    pub(crate) async fn configure(
        &mut self,
        jedec_id: [u8; 3],
        total_size: u32,
        supports_4byte: bool,
        sfdp_table: &[u8],
    ) -> Result<()> {
        let was_running = self.prepare_stopped().await?;
        let operation = self
            .configure_raw(jedec_id, total_size, supports_4byte, sfdp_table)
            .await;
        self.finish_stopped(was_running, operation).await
    }

    async fn configure_raw(
        &mut self,
        jedec_id: [u8; 3],
        total_size: u32,
        supports_4byte: bool,
        sfdp_table: &[u8],
    ) -> Result<()> {
        let erase_bursts = capacity_erase_bursts(total_size)
            .context("chip size must be a power of two between 8 bytes and 64 MiB")?;
        if sfdp_table.len() > 128 {
            bail!("SFDP table exceeds 128 bytes");
        }

        let flags = u8::from(supports_4byte);
        let header = ChipConfigHeader::new(jedec_id, flags, erase_bursts, sfdp_table.len() as u8)
            .context("chip erase burst count exceeds the protocol field")?;
        let mut request = Vec::with_capacity(size_of::<ChipConfigHeader>() + sfdp_table.len());
        request.extend_from_slice(header.as_bytes());
        request.extend_from_slice(sfdp_table);
        self.transport.write_all(&request).await?;
        self.expect_ack("chip config").await
    }

    pub(crate) async fn set_hold(&mut self, enabled: bool) -> Result<()> {
        self.require_capability("target flash HOLD control", |c| c.hold_control)
            .await?;
        self.transport
            .write_all(ControlRequest::hold(enabled).as_bytes())
            .await?;
        self.expect_ack("hold control").await
    }

    pub(crate) async fn log_start(&mut self) -> Result<()> {
        self.require_capability("activity logging", |c| c.activity_log)
            .await?;
        self.transport
            .write_all(ControlRequest::log(true).as_bytes())
            .await?;
        self.expect_ack("log start").await
    }

    pub(crate) async fn log_stop(&mut self) -> Result<()> {
        self.require_capability("activity logging", |c| c.activity_log)
            .await?;
        self.transport
            .write_all(ControlRequest::log(false).as_bytes())
            .await?;
        self.expect_ack("log stop").await
    }

    pub(crate) async fn log_poll(&mut self) -> Result<Vec<u8>> {
        self.require_capability("activity logging", |c| c.activity_log)
            .await?;
        self.transport.write_all(&[CMD_LOGPOLL]).await?;
        let mut output = Vec::with_capacity(256);
        loop {
            let byte = self.read_byte().await?;
            match byte {
                LOG_POLL_TERMINATOR => return Ok(output),
                LOG_POLL_ESCAPE => match self.read_byte().await? {
                    0x00 => output.push(LOG_POLL_TERMINATOR),
                    0x05 => output.push(LOG_POLL_ESCAPE),
                    code => bail!("invalid log escape 0x{code:02x}"),
                },
                value => output.push(value),
            }
            if output.len() > 1024 {
                bail!("log poll exceeded 1024 bytes without terminator");
            }
        }
    }

    pub(crate) async fn toctou_set(
        &mut self,
        index: u8,
        start: u32,
        mask: u32,
        replace: u32,
    ) -> Result<()> {
        self.require_capability("TOCTOU traps", |c| c.toctou)
            .await?;
        let request = ToctouSetRequest::new(index, start, mask, replace)
            .context("TOCTOU index must be 0-3 and addresses must fit in 24 bits")?;
        self.transport.write_all(request.as_bytes()).await?;
        self.expect_ack("TOCTOU set").await
    }

    async fn toctou_index(
        &mut self,
        request: Option<ToctouIndexRequest>,
        context: &str,
    ) -> Result<()> {
        self.require_capability("TOCTOU traps", |c| c.toctou)
            .await?;
        let request = request.context("TOCTOU index must be 0-3")?;
        self.transport.write_all(request.as_bytes()).await?;
        self.expect_ack(context).await
    }

    pub(crate) async fn toctou_arm(&mut self, index: u8) -> Result<()> {
        self.toctou_index(ToctouIndexRequest::arm(index), "TOCTOU arm")
            .await
    }

    pub(crate) async fn toctou_disarm(&mut self, index: u8) -> Result<()> {
        self.toctou_index(ToctouIndexRequest::disarm(index), "TOCTOU disarm")
            .await
    }

    pub(crate) async fn toctou_reset(&mut self, index: u8) -> Result<()> {
        self.toctou_index(ToctouIndexRequest::reset(index), "TOCTOU reset")
            .await
    }

    pub(crate) async fn toctou_reset_all(&mut self) -> Result<()> {
        self.require_capability("TOCTOU traps", |c| c.toctou)
            .await?;
        self.transport
            .write_all(ControlRequest::toctou_reset_all().as_bytes())
            .await?;
        self.expect_ack("TOCTOU reset-all").await
    }

    async fn read_byte(&mut self) -> Result<u8> {
        let mut byte = [0u8; 1];
        self.read_exact(&mut byte).await?;
        Ok(byte[0])
    }

    async fn command_with_ack(&mut self, command: u8, context: &str) -> Result<()> {
        self.transport.write_all(&[command]).await?;
        self.expect_ack(context).await
    }

    async fn read_block(&mut self, address: u32, length: usize) -> Result<Vec<u8>> {
        if length == 0
            || length > MAX_WRITE_BLOCK_SIZE
            || !address.is_multiple_of(8)
            || !length.is_multiple_of(8)
        {
            bail!("read block must be 8-byte aligned and at most {MAX_WRITE_BLOCK_SIZE} bytes");
        }
        validate_sdram_range(address, length, "read")?;
        let header = RamHeader::read(address / 8, (length / 8) as u16)
            .context("read address exceeds the protocol field")?;
        self.transport.write_all(header.as_bytes()).await?;
        let mut data = vec![0u8; length];
        self.read_exact(&mut data).await?;
        Ok(data)
    }

    async fn write_block(&mut self, address: u32, data: &[u8]) -> Result<()> {
        if data.is_empty()
            || data.len() > MAX_WRITE_BLOCK_SIZE
            || !address.is_multiple_of(8)
            || !data.len().is_multiple_of(8)
        {
            bail!("write block must be 8-byte aligned and at most {MAX_WRITE_BLOCK_SIZE} bytes");
        }
        validate_sdram_range(address, data.len(), "write")?;
        let header = RamHeader::write(address / 8, (data.len() / 8) as u16)
            .context("write address exceeds the protocol field")?;
        let mut request = Vec::with_capacity(size_of::<RamHeader>() + data.len());
        request.extend_from_slice(header.as_bytes());
        request.extend_from_slice(data);
        self.transport.write_all(&request).await?;
        self.expect_ack("write").await
    }
}

#[maybe_async::maybe_async(?Send)]
impl WriteSession<'_> {
    pub(crate) async fn write_chunk(
        &mut self,
        data: &[u8],
        mut progress: impl FnMut(usize) -> Result<()>,
    ) -> Result<()> {
        if data.is_empty() {
            bail!("write chunk is empty");
        }
        if data.len() > self.remaining {
            bail!(
                "write chunk contains {} bytes, but only {} remain",
                data.len(),
                self.remaining
            );
        }

        self.device
            .write_raw_with_progress(self.next_address, data, &mut progress)
            .await?;
        self.next_address += data.len() as u32;
        self.remaining -= data.len();
        Ok(())
    }

    pub(crate) async fn finish(self, operation: Result<()>) -> Result<()> {
        let operation = operation.and_then(|()| {
            if self.remaining == 0 {
                Ok(())
            } else {
                bail!("write ended with {} bytes remaining", self.remaining)
            }
        });
        self.device
            .finish_stopped(self.was_running, operation)
            .await
    }
}

#[cfg(feature = "cli")]
impl FlashDevice {
    /// Load is the one operation whose natural end state is running, even if
    /// the emulator was stopped before the operation.
    pub(crate) fn with_emulation_then_start<F, R>(&mut self, operation: F) -> Result<R>
    where
        F: FnOnce(&mut Self) -> Result<R>,
    {
        let capabilities = self.capabilities()?;
        if !capabilities.emulation_control {
            eprintln!(
                "Note: FPGA protocol v{} has no START/STOP; assuming idle SPI bus.",
                capabilities.version
            );
            return operation(self);
        }

        eprintln!("Stopping SPI emulation...");
        self.stop_emulation()?;
        let result = operation(self);
        let restoration = self.start_emulation();
        match (result, restoration) {
            (Ok(value), Ok(())) => {
                eprintln!("SPI emulation started; target can now read the loaded data.");
                Ok(value)
            }
            (Ok(_), Err(error)) => {
                Err(error.context("load completed, but SPI emulation could not be started"))
            }
            (Err(error), Ok(())) => Err(error),
            (Err(error), Err(start_error)) => Err(error.context(format!(
                "SPI emulation also could not be started: {start_error:#}"
            ))),
        }
    }
}

#[cfg(all(test, feature = "cli"))]
mod tests {
    use super::*;
    use std::cell::RefCell;
    use std::collections::VecDeque;
    use std::rc::Rc;

    struct MockTransport {
        reads: VecDeque<u8>,
        writes: Rc<RefCell<Vec<Vec<u8>>>>,
        connected: bool,
    }

    impl MockTransport {
        fn new(reads: impl IntoIterator<Item = u8>) -> (Self, Rc<RefCell<Vec<Vec<u8>>>>) {
            let writes = Rc::new(RefCell::new(Vec::new()));
            (
                Self {
                    reads: reads.into_iter().collect(),
                    writes: writes.clone(),
                    connected: true,
                },
                writes,
            )
        }
    }

    #[maybe_async::maybe_async(?Send)]
    impl Transport for MockTransport {
        async fn write_all(&mut self, data: &[u8]) -> Result<()> {
            self.writes.borrow_mut().push(data.to_vec());
            Ok(())
        }

        async fn read(&mut self, buffer: &mut [u8]) -> Result<usize> {
            let count = buffer.len().min(self.reads.len());
            for slot in &mut buffer[..count] {
                *slot = self.reads.pop_front().unwrap();
            }
            Ok(count)
        }

        async fn disconnect(&mut self) -> Result<()> {
            self.connected = false;
            Ok(())
        }

        fn is_connected(&self) -> bool {
            self.connected
        }
    }

    #[test]
    fn unsupported_protocol_is_rejected_before_commands_continue() {
        let (transport, writes) = MockTransport::new([PROTOCOL_VERSION + 1]);
        let mut device = FlashDevice::new(transport, ConnectionKind::Ft245, None).unwrap();

        let error = device.get_version().unwrap_err().to_string();

        assert!(error.contains("unsupported protocol version"));
        assert_eq!(&*writes.borrow(), &[vec![CMD_VERSION]]);
    }

    #[test]
    fn shared_read_path_stops_and_restores_emulation() {
        let data = [0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17];
        let reads = [5, 1, 1].into_iter().chain(data).chain([1]);
        let (transport, writes) = MockTransport::new(reads);
        let mut device = FlashDevice::new(transport, ConnectionKind::Ft245, None).unwrap();

        assert_eq!(device.read(0, 8).unwrap(), data);
        assert_eq!(
            &*writes.borrow(),
            &[
                vec![CMD_VERSION],
                vec![CMD_STATUS],
                vec![CMD_STOP],
                RamHeader::read(0, 1).unwrap().as_bytes().to_vec(),
                vec![CMD_START],
            ]
        );
    }

    #[test]
    fn cancelled_streaming_read_still_restores_emulation() {
        let reads = [1, 1].into_iter().chain([0u8; 8]).chain([1]);
        let (transport, writes) = MockTransport::new(reads);
        let mut device =
            FlashDevice::new(transport, ConnectionKind::Ft245, Some(PROTOCOL_VERSION)).unwrap();

        let error = device
            .read_chunks(0, 8, |_, _| bail!("cancelled by test"))
            .unwrap_err()
            .to_string();

        assert!(error.contains("cancelled by test"));
        assert_eq!(writes.borrow().last(), Some(&vec![CMD_START]));
    }

    #[test]
    fn cancelled_write_session_still_restores_emulation() {
        let (transport, writes) = MockTransport::new([1, 1, 1, 1]);
        let mut device =
            FlashDevice::new(transport, ConnectionKind::Ft245, Some(PROTOCOL_VERSION)).unwrap();

        let mut session = device.begin_write(0, 16).unwrap();
        session.write_chunk(&[0xaa; 8], |_| Ok(())).unwrap();
        let error = session
            .finish(Err(anyhow::anyhow!("cancelled by test")))
            .unwrap_err()
            .to_string();

        assert!(error.contains("cancelled by test"));
        assert_eq!(writes.borrow().last(), Some(&vec![CMD_START]));
    }
}
