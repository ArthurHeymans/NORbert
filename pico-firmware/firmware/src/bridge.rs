use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use heapless::Vec;
use norbert_pico_protocol::{
    BRIDGE_PROTOCOL_VERSION, BridgeStats, BridgeVersion, ChipConfig, DeviceFrame, ErrorCode,
    FPGA_PROTOCOL_VERSION, HostFrame, MAX_CHUNK, Request, Response, ToctouRequest,
};
use zerocopy::byteorder::big_endian::U16 as BeU16;

use crate::ft245_bus::Ft245Bus;

const CMD_VERSION: u8 = 0x30;
const CMD_READ: u8 = 0x31;
const CMD_WRITE: u8 = 0x32;
const CMD_CHIPCONFIG: u8 = 0x33;
const CMD_START: u8 = 0x34;
const CMD_STOP: u8 = 0x35;
const CMD_STATUS: u8 = 0x36;
const CMD_HOLDCTL: u8 = 0x37;
const CMD_LOGCTL: u8 = 0x38;
const CMD_TOCTOU: u8 = 0x39;
const CMD_LOGPOLL: u8 = 0x3A;

const LOG_POLL_TERMINATOR: u8 = 0xA0;
const MAX_WRITE_FRAME: usize = MAX_CHUNK + 6;
const TOCTOU_SET: u8 = 0x01;
const TOCTOU_ARM: u8 = 0x02;
const TOCTOU_DISARM: u8 = 0x03;
const TOCTOU_RESET: u8 = 0x04;
const TOCTOU_RESET_ALL: u8 = 0x05;

/// Shared bridge state used by USB and Ethernet RPC frontends.
pub struct Bridge<'d> {
    bus: Mutex<CriticalSectionRawMutex, Ft245Bus<'d>>,
    counters: Mutex<CriticalSectionRawMutex, BridgeStats>,
}

impl<'d> Bridge<'d> {
    /// Create shared bridge state around an FT245-compatible FPGA bus.
    pub fn new(bus: Ft245Bus<'d>) -> Self {
        Self {
            bus: Mutex::new(bus),
            counters: Mutex::new(BridgeStats::default()),
        }
    }

    /// Process one RPC frame and return a response frame with the same sequence number.
    pub async fn handle(&self, frame: HostFrame) -> DeviceFrame {
        let response = match self.dispatch(frame.request).await {
            Ok(response) => {
                let mut counters = self.counters.lock().await;
                counters.requests = counters.requests.saturating_add(1);
                response
            }
            Err(error) => {
                let mut counters = self.counters.lock().await;
                counters.errors = counters.errors.saturating_add(1);
                Response::Error(error)
            }
        };

        DeviceFrame {
            seq: frame.seq,
            response,
        }
    }

    async fn dispatch(&self, request: Request) -> Result<Response, ErrorCode> {
        match request {
            Request::BridgeVersion => {
                let mut build = Vec::new();
                let _ = build.extend_from_slice(env!("CARGO_PKG_VERSION").as_bytes());
                Ok(Response::BridgeVersion(BridgeVersion {
                    protocol: BRIDGE_PROTOCOL_VERSION,
                    max_chunk: MAX_CHUNK as u16,
                    build,
                }))
            }
            Request::FpgaVersion => {
                let mut bus = self.bus.lock().await;
                bus.drain_rx().await;
                bus.write_all(&[CMD_VERSION]).await?;
                for _ in 0..8 {
                    let mut version = [0u8; 1];
                    bus.read_exact(&mut version).await?;
                    let [version] = version;
                    if version == FPGA_PROTOCOL_VERSION {
                        bus.drain_rx().await;
                        return Ok(Response::FpgaVersion(version));
                    }
                    if version != 0x01 && version != 0x02 {
                        bus.drain_rx().await;
                        return Ok(Response::FpgaVersion(version));
                    }
                }
                bus.drain_rx().await;
                Err(ErrorCode::FpgaTimeout)
            }
            Request::Status => {
                let status = self.fpga_status().await?;
                Ok(Response::Status { running: status })
            }
            Request::Start => {
                self.bus.lock().await.command_ack(&[CMD_START]).await?;
                Ok(Response::Ack)
            }
            Request::Stop => {
                self.bus.lock().await.command_ack(&[CMD_STOP]).await?;
                Ok(Response::Ack)
            }
            Request::Hold { asserted } => {
                let value = u8::from(asserted);
                self.bus
                    .lock()
                    .await
                    .command_ack(&[CMD_HOLDCTL, value])
                    .await?;
                Ok(Response::Ack)
            }
            Request::LogControl { enabled } => {
                let value = u8::from(enabled);
                self.bus
                    .lock()
                    .await
                    .command_ack(&[CMD_LOGCTL, value])
                    .await?;
                Ok(Response::Ack)
            }
            Request::LogPoll => self.log_poll().await,
            Request::RamRead { address, length } => self.ram_read(address, length).await,
            Request::RamWrite { address, data } => {
                self.ram_write(address, data.as_slice(), true).await
            }
            Request::RamWriteFast { address, data } => {
                self.ram_write(address, data.as_slice(), false).await
            }
            Request::ChipConfig(config) => self.chip_config(&config).await,
            Request::Toctou(request) => self.toctou(request).await,
            Request::Stats => self.stats().await,
        }
    }

    async fn fpga_status(&self) -> Result<bool, ErrorCode> {
        let mut bus = self.bus.lock().await;
        bus.drain_rx().await;
        bus.write_all(&[CMD_STATUS]).await?;
        for _ in 0..8 {
            let status = bus.read_nonzero_ack().await?;
            match status {
                0x01 => {
                    bus.drain_rx().await;
                    return Ok(true);
                }
                0x02 => {
                    bus.drain_rx().await;
                    return Ok(false);
                }
                FPGA_PROTOCOL_VERSION => continue,
                other => {
                    bus.drain_rx().await;
                    return Err(ErrorCode::FpgaRejected(other));
                }
            }
        }
        bus.drain_rx().await;
        Err(ErrorCode::FpgaTimeout)
    }

    async fn ram_read(&self, address: u32, length: u16) -> Result<Response, ErrorCode> {
        let len = length as usize;
        if len == 0 || len > MAX_CHUNK || (address & 7) != 0 || (len & 7) != 0 {
            return Err(ErrorCode::BadArgument);
        }
        let addr_units = address / 8;
        let [len_hi, len_lo] = be_u16_bytes((len / 8) as u16);
        let command = [
            CMD_READ,
            ((addr_units >> 16) & 0xFF) as u8,
            ((addr_units >> 8) & 0xFF) as u8,
            (addr_units & 0xFF) as u8,
            len_hi,
            len_lo,
        ];

        let mut data = Vec::<u8, MAX_CHUNK>::new();
        data.resize(len, 0).map_err(|_| ErrorCode::BadArgument)?;

        let mut bus = self.bus.lock().await;
        bus.write_all(&command).await?;
        bus.read_exact(&mut data).await?;
        Ok(Response::Data(data))
    }

    async fn ram_write(
        &self,
        address: u32,
        data: &[u8],
        drain_before: bool,
    ) -> Result<Response, ErrorCode> {
        if data.is_empty() || data.len() > MAX_CHUNK || (address & 7) != 0 || (data.len() & 7) != 0
        {
            return Err(ErrorCode::BadArgument);
        }
        let addr_units = address / 8;
        let [len_hi, len_lo] = be_u16_bytes((data.len() / 8) as u16);
        let mut command = Vec::<u8, MAX_WRITE_FRAME>::new();
        command
            .extend_from_slice(&[
                CMD_WRITE,
                ((addr_units >> 16) & 0xFF) as u8,
                ((addr_units >> 8) & 0xFF) as u8,
                (addr_units & 0xFF) as u8,
                len_hi,
                len_lo,
            ])
            .map_err(|_| ErrorCode::BadArgument)?;
        command
            .extend_from_slice(data)
            .map_err(|_| ErrorCode::BadArgument)?;

        let mut bus = self.bus.lock().await;
        if drain_before {
            bus.drain_rx().await;
        }
        bus.write_all(command.as_slice()).await?;
        let ack = bus.read_nonzero_ack().await?;
        if ack != 0x01 {
            return Err(ErrorCode::FpgaRejected(ack));
        }
        Ok(Response::Ack)
    }

    async fn chip_config(&self, config: &ChipConfig) -> Result<Response, ErrorCode> {
        if config.sfdp.len() > 128 || config.chip_erase_bursts > 0x7F_FFFF {
            return Err(ErrorCode::BadArgument);
        }

        let flags = u8::from(config.supports_4byte_address);
        let erase = config.chip_erase_bursts;
        let mut command = Vec::<u8, 137>::new();
        command
            .push(CMD_CHIPCONFIG)
            .map_err(|_| ErrorCode::BadArgument)?;
        command
            .extend_from_slice(config.jedec_id.as_slice())
            .map_err(|_| ErrorCode::BadArgument)?;
        command.push(flags).map_err(|_| ErrorCode::BadArgument)?;
        let erase_and_sfdp_len = [
            ((erase >> 16) & 0x7F) as u8,
            ((erase >> 8) & 0xFF) as u8,
            (erase & 0xFF) as u8,
            config.sfdp.len() as u8,
        ];
        command
            .extend_from_slice(erase_and_sfdp_len.as_slice())
            .map_err(|_| ErrorCode::BadArgument)?;
        command
            .extend_from_slice(config.sfdp.as_slice())
            .map_err(|_| ErrorCode::BadArgument)?;

        self.bus.lock().await.command_ack(&command).await?;
        Ok(Response::Ack)
    }

    async fn log_poll(&self) -> Result<Response, ErrorCode> {
        let mut bus = self.bus.lock().await;
        bus.drain_rx().await;
        bus.write_all(&[CMD_LOGPOLL]).await?;

        let mut out = Vec::<u8, 256>::new();
        loop {
            let mut byte = [0u8; 1];
            bus.read_exact(&mut byte).await?;
            let [byte] = byte;
            if byte == LOG_POLL_TERMINATOR {
                break;
            }
            out.push(byte).map_err(|_| ErrorCode::FrameTooLarge)?;
        }
        bus.drain_rx().await;
        Ok(Response::Log(out))
    }

    async fn toctou(&self, request: ToctouRequest) -> Result<Response, ErrorCode> {
        let mut command = Vec::<u8, 12>::new();
        command
            .push(CMD_TOCTOU)
            .map_err(|_| ErrorCode::BadArgument)?;
        match request {
            ToctouRequest::Set {
                index,
                start,
                mask,
                replace,
            } => {
                if index > 3 || start > 0xFF_FFFF || mask > 0xFF_FFFF || replace > 0xFF_FFFF {
                    return Err(ErrorCode::BadArgument);
                }
                command
                    .push(TOCTOU_SET)
                    .map_err(|_| ErrorCode::BadArgument)?;
                command.push(index).map_err(|_| ErrorCode::BadArgument)?;
                push_u24(&mut command, start)?;
                push_u24(&mut command, mask)?;
                push_u24(&mut command, replace)?;
            }
            ToctouRequest::Arm { index } => push_index(&mut command, TOCTOU_ARM, index)?,
            ToctouRequest::Disarm { index } => push_index(&mut command, TOCTOU_DISARM, index)?,
            ToctouRequest::Reset { index } => push_index(&mut command, TOCTOU_RESET, index)?,
            ToctouRequest::ResetAll => command
                .push(TOCTOU_RESET_ALL)
                .map_err(|_| ErrorCode::BadArgument)?,
        }

        self.bus.lock().await.command_ack(&command).await?;
        Ok(Response::Ack)
    }

    async fn stats(&self) -> Result<Response, ErrorCode> {
        let bus_stats = {
            let bus = self.bus.lock().await;
            bus.stats()
        };
        let mut stats = *self.counters.lock().await;
        stats.fpga_tx_bytes = bus_stats.tx_bytes;
        stats.fpga_rx_bytes = bus_stats.rx_bytes;
        Ok(Response::Stats(stats))
    }
}

fn be_u16_bytes(value: u16) -> [u8; 2] {
    BeU16::new(value).to_bytes()
}

fn push_index<const N: usize>(
    out: &mut Vec<u8, N>,
    subcmd: u8,
    index: u8,
) -> Result<(), ErrorCode> {
    if index > 3 {
        return Err(ErrorCode::BadArgument);
    }
    out.push(subcmd).map_err(|_| ErrorCode::BadArgument)?;
    out.push(index).map_err(|_| ErrorCode::BadArgument)
}

fn push_u24<const N: usize>(out: &mut Vec<u8, N>, value: u32) -> Result<(), ErrorCode> {
    let bytes = [
        ((value >> 16) & 0xFF) as u8,
        ((value >> 8) & 0xFF) as u8,
        (value & 0xFF) as u8,
    ];
    out.extend_from_slice(bytes.as_slice())
        .map_err(|_| ErrorCode::BadArgument)
}
