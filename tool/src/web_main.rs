//! Browser UI for NORbert using WebSerial.

#![allow(deprecated, unused_unsafe)]

use eframe::egui;
use spi_flash_tool::chip::{find_chip_by_name, load_chip_db_from_ron_strings, FlashChip};
use spi_flash_tool::gowin::{self, FlashOptions, ProgramProgress};
use spi_flash_tool::protocol::{
    self, format_log_event, format_log_summary, hexdump, parse_hex_bytes, parse_u32, LogParser,
    BAUD_RATE, BLOCK_SIZE, CMD_LOGPOLL, CMD_START, CMD_STATUS, CMD_STOP, CMD_VERSION,
    LOG_POLL_TERMINATOR, PROTOCOL_VERSION, TOCTOU_ARM, TOCTOU_DISARM, TOCTOU_RESET,
    TOCTOU_RESET_ALL, UART_READ_BLOCK_SIZE,
};
use std::cell::RefCell;
use std::rc::Rc;
use wasm_bindgen::prelude::*;
use wasm_bindgen::JsCast;
use wasm_bindgen_futures::{spawn_local, JsFuture};
use web_sys::{
    FlowControlType, HtmlInputElement, ReadableStream, ReadableStreamByobReader,
    ReadableStreamGetReaderOptions, ReadableStreamReaderMode, Response, SerialOptions, SerialPort,
    WritableStreamDefaultWriter,
};

include!(concat!(env!("OUT_DIR"), "/embedded_chip_db.rs"));

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_namespace = navigator, js_name = serial)]
    static SERIAL: web_sys::Serial;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Panel {
    Device,
    Memory,
    Chip,
    Monitor,
    Toctou,
    Bitstream,
}

struct WebSerialTransport {
    port: SerialPort,
    reader: ReadableStreamByobReader,
    writer: WritableStreamDefaultWriter,
    read_buffer: Vec<u8>,
}

impl WebSerialTransport {
    async fn request_and_open() -> Result<Self, String> {
        let port: SerialPort = JsFuture::from(unsafe { SERIAL.request_port() })
            .await
            .map_err(|e| format!("serial port request failed: {e:?}"))?
            .unchecked_into();

        let options = SerialOptions::new(BAUD_RATE);
        options.set_buffer_size((16 * 1024 * 1024) as u32);
        // Hardware flow control is harmless for bridges that ignore it and helps
        // with adapters that expose RTS/CTS.
        options.set_flow_control(FlowControlType::Hardware);

        JsFuture::from(port.open(&options))
            .await
            .map_err(|e| format!("serial open failed: {e:?}"))?;

        let readable: ReadableStream = port.readable();
        let reader_options = ReadableStreamGetReaderOptions::new();
        reader_options.set_mode(ReadableStreamReaderMode::Byob);
        let reader: ReadableStreamByobReader = readable
            .get_reader_with_options(&reader_options)
            .unchecked_into();

        let writer: WritableStreamDefaultWriter = port
            .writable()
            .get_writer()
            .map_err(|e| format!("serial writer setup failed: {e:?}"))?
            .unchecked_into();

        Ok(Self {
            port,
            reader,
            writer,
            read_buffer: Vec::new(),
        })
    }

    async fn close(&self) -> Result<(), String> {
        self.reader.release_lock();
        self.writer.release_lock();
        JsFuture::from(self.port.close())
            .await
            .map(|_| ())
            .map_err(|e| format!("serial close failed: {e:?}"))
    }

    async fn write_all(&mut self, data: &[u8]) -> Result<(), String> {
        JsFuture::from(self.writer.ready())
            .await
            .map_err(|e| format!("serial writer not ready: {e:?}"))?;
        let chunk = js_sys::Uint8Array::from(data);
        JsFuture::from(self.writer.write_with_chunk(&chunk))
            .await
            .map_err(|e| format!("serial write failed: {e:?}"))?;
        Ok(())
    }

    async fn read_byob(&mut self, len: usize) -> Result<Vec<u8>, String> {
        let view = js_sys::Uint8Array::new_with_length(len as u32);
        let result = JsFuture::from(self.reader.read_with_array_buffer_view(&view))
            .await
            .map_err(|e| format!("serial read failed: {e:?}"))?;
        let done = js_sys::Reflect::get(&result, &"done".into())
            .map_err(|e| format!("serial read result missing done: {e:?}"))?
            .as_bool()
            .unwrap_or(false);
        if done {
            return Err("serial stream ended".to_string());
        }
        let value = js_sys::Reflect::get(&result, &"value".into())
            .map_err(|e| format!("serial read result missing value: {e:?}"))?;
        let value: js_sys::Uint8Array = value
            .dyn_into()
            .map_err(|_| "serial read result was not Uint8Array".to_string())?;
        let mut out = vec![0; value.byte_length() as usize];
        value.copy_to(&mut out);
        Ok(out)
    }

    async fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), String> {
        let mut offset = 0;
        if !self.read_buffer.is_empty() {
            let n = self.read_buffer.len().min(buf.len());
            buf[..n].copy_from_slice(&self.read_buffer[..n]);
            self.read_buffer.drain(..n);
            offset = n;
        }
        while offset < buf.len() {
            let bytes = self.read_byob(buf.len() - offset).await?;
            if bytes.is_empty() {
                return Err("serial read returned no bytes".to_string());
            }
            let n = bytes.len().min(buf.len() - offset);
            buf[offset..offset + n].copy_from_slice(&bytes[..n]);
            offset += n;
            if n < bytes.len() {
                self.read_buffer.extend_from_slice(&bytes[n..]);
            }
        }
        Ok(())
    }
}

struct FlashDevice {
    transport: WebSerialTransport,
}

impl FlashDevice {
    async fn open() -> Result<Self, String> {
        Ok(Self {
            transport: WebSerialTransport::request_and_open().await?,
        })
    }

    async fn close(&self) -> Result<(), String> {
        self.transport.close().await
    }

    async fn get_version(&mut self) -> Result<u8, String> {
        self.transport.write_all(&[CMD_VERSION]).await?;
        let mut buf = [0u8; 1];
        self.transport.read_exact(&mut buf).await?;
        Ok(buf[0])
    }

    async fn read_ack(&mut self, context: &str) -> Result<u8, String> {
        let mut skipped = 0;
        loop {
            let mut resp = [0u8; 1];
            self.transport.read_exact(&mut resp).await?;
            if resp[0] != 0x00 {
                return Ok(resp[0]);
            }
            skipped += 1;
            if skipped > 8 {
                return Err(format!("{context}: too many 0x00 bytes before ACK"));
            }
        }
    }

    async fn start_emulation(&mut self) -> Result<(), String> {
        self.transport.write_all(&[CMD_START]).await?;
        expect_ack(self.read_ack("start").await?, "start")
    }

    async fn stop_emulation(&mut self) -> Result<(), String> {
        self.transport.write_all(&[CMD_STOP]).await?;
        expect_ack(self.read_ack("stop").await?, "stop")
    }

    async fn status(&mut self) -> Result<bool, String> {
        self.transport.write_all(&[CMD_STATUS]).await?;
        match self.read_ack("status").await? {
            0x01 => Ok(true),
            0x02 => Ok(false),
            other => Err(format!("status: unexpected response 0x{other:02x}")),
        }
    }

    async fn set_hold(&mut self, enable: bool) -> Result<(), String> {
        self.transport
            .write_all(&protocol::hold_command(enable))
            .await?;
        expect_ack(self.read_ack("hold").await?, "hold")
    }

    async fn send_chip_config(&mut self, chip: &FlashChip) -> Result<(), String> {
        let buf = protocol::chip_config_command(chip)?;
        self.transport.write_all(&buf).await?;
        expect_ack(self.read_ack("chip config").await?, "chip config")
    }

    async fn read_block(&mut self, address: u32, length: usize) -> Result<Vec<u8>, String> {
        let cmd = protocol::read_command(address, length)?;
        self.transport.write_all(&cmd).await?;
        let mut buf = vec![0; length];
        self.transport.read_exact(&mut buf).await?;
        Ok(buf)
    }

    async fn write_block(&mut self, address: u32, data: &[u8]) -> Result<(), String> {
        let buf = protocol::write_command(address, data)?;
        self.transport.write_all(&buf).await?;
        expect_ack(self.read_ack("write").await?, "write")
    }

    async fn read(
        &mut self,
        address: u32,
        length: u32,
        progress: ProgressFn,
    ) -> Result<Vec<u8>, String> {
        let mut result = Vec::with_capacity(length as usize);
        let mut addr = address;
        let mut remaining = length as usize;
        let total = remaining.max(1);

        let start_offset = (addr % 8) as usize;
        if start_offset != 0 {
            let aligned_addr = addr - start_offset as u32;
            let chunk_len = (8 - start_offset).min(remaining);
            let block = self.read_block(aligned_addr, 8).await?;
            result.extend_from_slice(&block[start_offset..start_offset + chunk_len]);
            addr += chunk_len as u32;
            remaining -= chunk_len;
            progress(result.len(), total);
        }

        while remaining >= 8 {
            let chunk_len = UART_READ_BLOCK_SIZE.min(remaining / 8 * 8);
            let block = self.read_block(addr, chunk_len).await?;
            result.extend_from_slice(&block);
            addr += chunk_len as u32;
            remaining -= chunk_len;
            progress(result.len(), total);
            yield_to_browser().await;
        }

        if remaining > 0 {
            let block = self.read_block(addr, 8).await?;
            result.extend_from_slice(&block[..remaining]);
            progress(result.len(), total);
        }
        Ok(result)
    }

    async fn write(
        &mut self,
        address: u32,
        data: &[u8],
        progress: ProgressFn,
    ) -> Result<(), String> {
        if !address.is_multiple_of(8) {
            return Err("address must be 8-byte aligned for writes".to_string());
        }
        let padded = protocol::padded_write_data(data);
        let total = padded.len().max(1);
        let mut addr = address;
        let mut offset = 0;
        while offset < padded.len() {
            let chunk_len = BLOCK_SIZE.min(padded.len() - offset);
            self.write_block(addr, &padded[offset..offset + chunk_len])
                .await?;
            addr += chunk_len as u32;
            offset += chunk_len;
            progress(offset.min(data.len()), total);
            yield_to_browser().await;
        }
        Ok(())
    }

    async fn log_start(&mut self) -> Result<(), String> {
        self.transport
            .write_all(&protocol::logctl_command(true))
            .await?;
        expect_ack(self.read_ack("log start").await?, "log start")
    }

    async fn log_stop(&mut self) -> Result<(), String> {
        self.transport
            .write_all(&protocol::logctl_command(false))
            .await?;
        expect_ack(self.read_ack("log stop").await?, "log stop")
    }

    async fn log_poll(&mut self) -> Result<Vec<u8>, String> {
        self.transport.write_all(&[CMD_LOGPOLL]).await?;
        let mut out = Vec::with_capacity(256);
        loop {
            let mut byte = [0u8; 1];
            self.transport.read_exact(&mut byte).await?;
            if byte[0] == LOG_POLL_TERMINATOR {
                return Ok(out);
            }
            out.push(byte[0]);
            if out.len() > 1024 {
                return Err("log poll overran 1024 bytes without terminator".to_string());
            }
        }
    }

    async fn toctou_set(
        &mut self,
        index: u8,
        start: u32,
        mask: u32,
        replace: u32,
    ) -> Result<(), String> {
        let buf = protocol::toctou_set_command(index, start, mask, replace);
        self.transport.write_all(&buf).await?;
        expect_ack(self.read_ack("TOCTOU set").await?, "TOCTOU set")
    }

    async fn toctou_simple(&mut self, op: u8, index: Option<u8>) -> Result<(), String> {
        if let Some(index) = index {
            self.transport
                .write_all(&protocol::toctou_index_command(op, index))
                .await?;
        } else {
            self.transport
                .write_all(&[protocol::CMD_TOCTOU, op])
                .await?;
        }
        expect_ack(self.read_ack("TOCTOU").await?, "TOCTOU")
    }
}

type ProgressFn = Box<dyn Fn(usize, usize)>;

fn expect_ack(ack: u8, context: &str) -> Result<(), String> {
    if ack == 0x01 {
        Ok(())
    } else {
        Err(format!("{context}: unexpected response 0x{ack:02x}"))
    }
}

async fn sleep_ms(ms: i32) {
    let promise = js_sys::Promise::new(&mut |resolve, _| {
        let _ = web_sys::window()
            .expect("window")
            .set_timeout_with_callback_and_timeout_and_arguments_0(&resolve, ms);
    });
    let _ = JsFuture::from(promise).await;
}

async fn yield_to_browser() {
    sleep_ms(0).await;
}

#[derive(Default)]
struct SharedState {
    device: Option<FlashDevice>,
    connected: bool,
    busy: bool,
    progress: f32,
    progress_message: String,
    log: String,
    monitor_running: bool,
    downloaded: Option<Vec<u8>>,
    pending_file: Option<(String, Vec<u8>)>,
    pending_bitstream_file: Option<(String, Vec<u8>)>,
}

fn update_fpga_progress(shared: &Rc<RefCell<SharedState>>, progress: ProgramProgress) {
    let mut s = shared.borrow_mut();
    s.progress = progress.done as f32 / progress.total.max(1) as f32;
    s.progress_message = format!("{}: {}/{}", progress.stage, progress.done, progress.total);
}

pub struct NorbertApp {
    shared: Rc<RefCell<SharedState>>,
    chips: Vec<FlashChip>,
    panel: Panel,
    status: String,
    status_error: bool,
    chip_query: String,
    selected_chip: Option<usize>,
    address: String,
    length: String,
    write_hex: String,
    upload_file_name: String,
    upload_file_data: Option<Vec<u8>>,
    bitstream_file_name: String,
    bitstream_file_data: Option<Vec<u8>>,
    bitstream_flash_offset: String,
    bitstream_verify_flash: bool,
    bitstream_unprotect_flash: bool,
    bitstream_chip_erase: bool,
    toctou_index: String,
    toctou_start: String,
    toctou_mask: String,
    toctou_replace: String,
    bitstream_url: String,
}

impl NorbertApp {
    fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        let chips =
            load_chip_db_from_ron_strings(EMBEDDED_CHIP_RON.iter().copied()).unwrap_or_default();
        Self {
            shared: Rc::new(RefCell::new(SharedState::default())),
            chips,
            panel: Panel::Device,
            status: "Connect a NORbert UART using WebSerial.".to_string(),
            status_error: false,
            chip_query: "W25Q64".to_string(),
            selected_chip: None,
            address: "0".to_string(),
            length: "0x100".to_string(),
            write_hex: "deadbeef".to_string(),
            upload_file_name: String::new(),
            upload_file_data: None,
            bitstream_file_name: String::new(),
            bitstream_file_data: None,
            bitstream_flash_offset: "0".to_string(),
            bitstream_verify_flash: true,
            bitstream_unprotect_flash: false,
            bitstream_chip_erase: false,
            toctou_index: "0".to_string(),
            toctou_start: "0".to_string(),
            toctou_mask: "0xFFFFFF".to_string(),
            toctou_replace: "0".to_string(),
            bitstream_url: "spi_flash.fs".to_string(),
        }
    }

    fn set_status(&mut self, message: impl Into<String>, error: bool) {
        self.status = message.into();
        self.status_error = error;
    }

    fn run_device_op<Fut, F>(&mut self, message: &str, op: F)
    where
        Fut: std::future::Future<Output = Result<String, String>> + 'static,
        F: FnOnce(Rc<RefCell<SharedState>>) -> Fut + 'static,
    {
        let shared = self.shared.clone();
        {
            let mut s = shared.borrow_mut();
            if s.busy {
                self.set_status("another operation is already running", true);
                return;
            }
            s.busy = true;
            s.progress = 0.0;
            s.progress_message = message.to_string();
        }
        spawn_local(async move {
            let result = op(shared.clone()).await;
            let mut s = shared.borrow_mut();
            s.busy = false;
            s.progress = 1.0;
            match result {
                Ok(msg) => {
                    s.log.push_str(&format!("✓ {msg}\n"));
                    s.progress_message = msg;
                }
                Err(err) => {
                    s.log.push_str(&format!("✗ {err}\n"));
                    s.progress_message = err;
                }
            }
        });
    }

    fn connect(&mut self) {
        self.run_device_op("Connecting...", |shared| async move {
            let mut dev = FlashDevice::open().await?;
            let version = dev.get_version().await?;
            let running = if version >= 4 {
                dev.status().await.ok()
            } else {
                None
            };
            let mut s = shared.borrow_mut();
            s.connected = true;
            s.device = Some(dev);
            Ok(match running {
                Some(true) => format!("Connected. Protocol v{version}; emulation running."),
                Some(false) => format!("Connected. Protocol v{version}; emulation stopped."),
                None => format!("Connected. Protocol v{version}."),
            })
        });
    }

    fn disconnect(&mut self) {
        self.run_device_op("Disconnecting...", |shared| async move {
            let dev = shared.borrow_mut().device.take();
            if let Some(dev) = dev {
                let _ = dev.close().await;
            }
            let mut s = shared.borrow_mut();
            s.connected = false;
            Ok("Disconnected".to_string())
        });
    }

    fn version(&mut self) {
        self.run_device_op("Reading version...", |shared| async move {
            let mut dev = take_device(&shared)?;
            let version = dev.get_version().await?;
            put_device(&shared, dev);
            Ok(if version == PROTOCOL_VERSION {
                format!("Protocol version: {version}")
            } else {
                format!("Protocol version: {version} (expected {PROTOCOL_VERSION})")
            })
        });
    }

    fn start_stop_status(&mut self, action: &'static str) {
        self.run_device_op(action, move |shared| async move {
            let mut dev = take_device(&shared)?;
            let msg = match action {
                "Start" => {
                    dev.start_emulation().await?;
                    "SPI emulation started".to_string()
                }
                "Stop" => {
                    dev.stop_emulation().await?;
                    "SPI emulation stopped".to_string()
                }
                _ => {
                    let running = dev.status().await?;
                    format!(
                        "SPI emulation is {}",
                        if running { "running" } else { "stopped" }
                    )
                }
            };
            put_device(&shared, dev);
            Ok(msg)
        });
    }

    fn set_hold(&mut self, enable: bool) {
        self.run_device_op("Setting #HOLD...", move |shared| async move {
            let mut dev = take_device(&shared)?;
            dev.set_hold(enable).await?;
            put_device(&shared, dev);
            Ok(format!(
                "#HOLD {}",
                if enable { "asserted" } else { "released" }
            ))
        });
    }

    fn configure_chip(&mut self) {
        let query = self.chip_query.clone();
        let chip = match find_chip_by_name(&self.chips, &query) {
            Ok(chip) => chip.clone(),
            Err(e) => {
                self.set_status(e.to_string(), true);
                return;
            }
        };
        self.run_device_op("Configuring chip...", move |shared| async move {
            let mut dev = take_device(&shared)?;
            let was_running = dev.status().await.unwrap_or(false);
            if was_running {
                dev.stop_emulation().await?;
            }
            dev.send_chip_config(&chip).await?;
            if was_running {
                dev.start_emulation().await?;
            }
            put_device(&shared, dev);
            Ok(format!(
                "Configured {} {} ({} bytes, JEDEC {:02X?})",
                chip.vendor,
                chip.name,
                chip.total_size,
                chip.jedec_id_bytes()
            ))
        });
    }

    fn read_memory(&mut self) {
        let address = match parse_u32(&self.address) {
            Ok(v) => v,
            Err(e) => return self.set_status(e, true),
        };
        let length = match parse_u32(&self.length) {
            Ok(v) => v,
            Err(e) => return self.set_status(e, true),
        };
        self.run_device_op("Reading...", move |shared| async move {
            let mut dev = take_device(&shared)?;
            let was_running = dev.status().await.unwrap_or(false);
            if was_running {
                dev.stop_emulation().await?;
            }
            let progress_shared = shared.clone();
            let data = dev
                .read(
                    address,
                    length,
                    Box::new(move |done, total| {
                        progress_shared.borrow_mut().progress = done as f32 / total as f32;
                    }),
                )
                .await?;
            if was_running {
                let _ = dev.start_emulation().await;
            }
            {
                let mut s = shared.borrow_mut();
                s.downloaded = Some(data.clone());
                s.log.push_str(&format!(
                    "{}\n",
                    hexdump(address, &data[..data.len().min(512)])
                ));
            }
            put_device(&shared, dev);
            Ok(format!("Read {} bytes from 0x{address:06X}", data.len()))
        });
    }

    fn write_hex(&mut self) {
        let address = match parse_u32(&self.address) {
            Ok(v) => v,
            Err(e) => return self.set_status(e, true),
        };
        let data = match parse_hex_bytes(&self.write_hex) {
            Ok(v) => v,
            Err(e) => return self.set_status(e, true),
        };
        self.write_data(address, data, false);
    }

    fn load_file(&mut self, verify: bool) {
        let address = match parse_u32(&self.address) {
            Ok(v) => v,
            Err(e) => return self.set_status(e, true),
        };
        let Some(data) = self.upload_file_data.clone() else {
            self.set_status("choose a file first", true);
            return;
        };
        self.write_data(address, data, verify);
    }

    fn write_data(&mut self, address: u32, data: Vec<u8>, verify: bool) {
        self.run_device_op("Writing...", move |shared| async move {
            let mut dev = take_device(&shared)?;
            dev.stop_emulation().await?;
            let progress_shared = shared.clone();
            dev.write(
                address,
                &data,
                Box::new(move |done, total| {
                    progress_shared.borrow_mut().progress = done as f32 / total as f32;
                }),
            )
            .await?;
            if verify {
                shared.borrow_mut().progress_message = "Verifying...".to_string();
                let read_back = dev
                    .read(address, data.len() as u32, Box::new(|_, _| {}))
                    .await?;
                if read_back != data {
                    put_device(&shared, dev);
                    return Err("verification failed".to_string());
                }
            }
            dev.start_emulation().await?;
            put_device(&shared, dev);
            Ok(format!(
                "Wrote {} bytes at 0x{address:06X}{}; emulation started",
                data.len(),
                if verify { " and verified" } else { "" }
            ))
        });
    }

    fn select_file(&mut self) {
        let shared = self.shared.clone();
        let Some(document) = web_sys::window().and_then(|w| w.document()) else {
            return;
        };
        let Ok(element) = document.create_element("input") else {
            return;
        };
        let Ok(input) = element.dyn_into::<HtmlInputElement>() else {
            return;
        };
        input.set_type("file");
        input.set_accept(".bin,.rom,.img,.fs,*");
        let onchange = Closure::wrap(Box::new(move |event: web_sys::Event| {
            let input: HtmlInputElement = event.target().unwrap().dyn_into().unwrap();
            if let Some(file) = input.files().and_then(|files| files.get(0)) {
                let filename = file.name();
                let shared = shared.clone();
                spawn_local(async move {
                    match JsFuture::from(file.array_buffer()).await {
                        Ok(buffer) => {
                            let data = js_sys::Uint8Array::new(&buffer).to_vec();
                            shared.borrow_mut().pending_file = Some((filename, data));
                        }
                        Err(e) => shared
                            .borrow_mut()
                            .log
                            .push_str(&format!("file read failed: {e:?}\n")),
                    }
                });
            }
        }) as Box<dyn FnMut(_)>);
        input.set_onchange(Some(onchange.as_ref().unchecked_ref()));
        onchange.forget();
        input.click();
    }

    fn select_bitstream_file(&mut self) {
        let shared = self.shared.clone();
        let Some(document) = web_sys::window().and_then(|w| w.document()) else {
            return;
        };
        let Ok(element) = document.create_element("input") else {
            return;
        };
        let Ok(input) = element.dyn_into::<HtmlInputElement>() else {
            return;
        };
        input.set_type("file");
        input.set_accept(".fs,*");
        let onchange = Closure::wrap(Box::new(move |event: web_sys::Event| {
            let input: HtmlInputElement = event.target().unwrap().dyn_into().unwrap();
            if let Some(file) = input.files().and_then(|files| files.get(0)) {
                let filename = file.name();
                let shared = shared.clone();
                spawn_local(async move {
                    match JsFuture::from(file.array_buffer()).await {
                        Ok(buffer) => {
                            let data = js_sys::Uint8Array::new(&buffer).to_vec();
                            shared.borrow_mut().pending_bitstream_file = Some((filename, data));
                        }
                        Err(e) => shared
                            .borrow_mut()
                            .log
                            .push_str(&format!("bitstream file read failed: {e:?}\n")),
                    }
                });
            }
        }) as Box<dyn FnMut(_)>);
        input.set_onchange(Some(onchange.as_ref().unchecked_ref()));
        onchange.forget();
        input.click();
    }

    fn fetch_bitstream(&mut self) {
        let url = self.bitstream_url.clone();
        self.run_device_op("Fetching bitstream...", move |shared| async move {
            let window = web_sys::window().ok_or_else(|| "no browser window".to_string())?;
            let response = JsFuture::from(window.fetch_with_str(&url))
                .await
                .map_err(|e| format!("fetch failed: {e:?}"))?;
            let response: Response = response
                .dyn_into()
                .map_err(|_| "fetch did not return a Response".to_string())?;
            if !response.ok() {
                return Err(format!(
                    "fetch failed with HTTP status {}",
                    response.status()
                ));
            }
            let buffer = JsFuture::from(
                response
                    .array_buffer()
                    .map_err(|e| format!("failed to read response body: {e:?}"))?,
            )
            .await
            .map_err(|e| format!("failed to read response body: {e:?}"))?;
            let data = js_sys::Uint8Array::new(&buffer).to_vec();
            shared.borrow_mut().pending_bitstream_file = Some((url.clone(), data.clone()));
            Ok(format!("Fetched {url} ({} bytes)", data.len()))
        });
    }

    fn program_bitstream_sram(&mut self) {
        let Some(data) = self.bitstream_file_data.clone() else {
            self.set_status("choose or fetch a Gowin .fs bitstream first", true);
            return;
        };
        let name = if self.bitstream_file_name.is_empty() {
            "selected bitstream".to_string()
        } else {
            self.bitstream_file_name.clone()
        };
        self.run_device_op("Programming FPGA SRAM...", move |shared| async move {
            let info = gowin::inspect_bitstream(&data)?;
            shared.borrow_mut().log.push_str(&format!(
                "Programming {name} to SRAM: {} bytes, target IDCODE 0x{:08x}\n",
                info.sram_bytes, info.idcode
            ));
            let progress_shared = shared.clone();
            let report = gowin::program_sram_from_webusb(&data, move |p: ProgramProgress| {
                update_fpga_progress(&progress_shared, p);
            })
            .await?;
            Ok(format!(
                "Programmed FPGA SRAM via WebUSB (JTAG IDCODE 0x{:08x}, bitstream IDCODE 0x{:08x}, checksum 0x{:08x}, {} bytes)",
                report.idcode, report.bitstream_idcode, report.checksum, report.bytes_programmed
            ))
        });
    }

    fn program_bitstream_flash(&mut self) {
        let Some(data) = self.bitstream_file_data.clone() else {
            self.set_status("choose or fetch a Gowin .fs bitstream first", true);
            return;
        };
        let offset = match parse_u32(&self.bitstream_flash_offset) {
            Ok(v) => v,
            Err(e) => return self.set_status(e, true),
        };
        let options = FlashOptions {
            offset,
            verify: self.bitstream_verify_flash,
            unprotect: self.bitstream_unprotect_flash,
            chip_erase: self.bitstream_chip_erase,
        };
        let name = if self.bitstream_file_name.is_empty() {
            "selected bitstream".to_string()
        } else {
            self.bitstream_file_name.clone()
        };
        self.run_device_op("Programming FPGA flash...", move |shared| async move {
            let info = gowin::inspect_bitstream(&data)?;
            shared.borrow_mut().log.push_str(&format!(
                "Programming {name} to external flash: {} bytes at 0x{offset:06x}, target IDCODE 0x{:08x}\n",
                info.flash_bytes, info.idcode
            ));
            let progress_shared = shared.clone();
            let report = gowin::program_flash_from_webusb(&data, options, move |p: ProgramProgress| {
                update_fpga_progress(&progress_shared, p);
            })
            .await?;
            Ok(format!(
                "Programmed FPGA flash via WebUSB (JTAG IDCODE 0x{:08x}, JEDEC 0x{:06x}, {} bytes{})",
                report.idcode,
                report.flash_jedec_id.unwrap_or(0),
                report.bytes_programmed,
                if options.verify { ", verified" } else { "" }
            ))
        });
    }

    fn save_download(&mut self) {
        let Some(data) = self.shared.borrow().downloaded.clone() else {
            self.set_status("nothing has been read yet", true);
            return;
        };
        save_file(&data, "norbert-dump.bin");
    }

    fn monitor_start(&mut self) {
        let shared = self.shared.clone();
        if shared.borrow().monitor_running || shared.borrow().busy {
            return;
        }
        shared.borrow_mut().busy = true;
        spawn_local(async move {
            let mut dev = match take_device(&shared) {
                Ok(dev) => dev,
                Err(e) => {
                    shared.borrow_mut().log.push_str(&format!("✗ {e}\n"));
                    shared.borrow_mut().busy = false;
                    return;
                }
            };
            if let Err(e) = dev.log_start().await {
                shared
                    .borrow_mut()
                    .log
                    .push_str(&format!("✗ log start: {e}\n"));
                put_device(&shared, dev);
                shared.borrow_mut().busy = false;
                return;
            }
            shared.borrow_mut().monitor_running = true;
            let mut parser = LogParser::default();
            while shared.borrow().monitor_running {
                match dev.log_poll().await {
                    Ok(data) if data.is_empty() => sleep_ms(5).await,
                    Ok(data) => {
                        let text = parser
                            .push(&data)
                            .iter()
                            .map(format_log_event)
                            .collect::<String>();
                        if !text.is_empty() {
                            shared.borrow_mut().log.push_str(&text);
                        }
                    }
                    Err(e) => {
                        shared
                            .borrow_mut()
                            .log
                            .push_str(&format!("✗ log poll: {e}\n"));
                        break;
                    }
                }
                yield_to_browser().await;
            }
            let _ = dev.log_stop().await;
            shared
                .borrow_mut()
                .log
                .push_str(&format_log_summary(&parser));
            shared.borrow_mut().monitor_running = false;
            shared.borrow_mut().busy = false;
            put_device(&shared, dev);
        });
    }

    fn monitor_stop(&mut self) {
        self.shared.borrow_mut().monitor_running = false;
    }

    fn toctou_action(&mut self, action: &'static str) {
        let index = match parse_u8(&self.toctou_index) {
            Ok(v) if v <= 3 => v,
            _ => return self.set_status("trap index must be 0..3", true),
        };
        let start = parse_u32(&self.toctou_start).unwrap_or(0);
        let mask = parse_u32(&self.toctou_mask).unwrap_or(0);
        let replace = parse_u32(&self.toctou_replace).unwrap_or(0);
        self.run_device_op("TOCTOU...", move |shared| async move {
            let mut dev = take_device(&shared)?;
            match action {
                "set" => dev.toctou_set(index, start, mask, replace).await?,
                "arm" => dev.toctou_simple(TOCTOU_ARM, Some(index)).await?,
                "disarm" => dev.toctou_simple(TOCTOU_DISARM, Some(index)).await?,
                "reset" => dev.toctou_simple(TOCTOU_RESET, Some(index)).await?,
                "reset-all" => dev.toctou_simple(TOCTOU_RESET_ALL, None).await?,
                _ => {}
            }
            put_device(&shared, dev);
            Ok(format!("TOCTOU {action} complete"))
        });
    }

    fn ui_device(&mut self, ui: &mut egui::Ui) {
        ui.heading("Device");
        ui.label("Connect the NORbert UART. The browser will show a WebSerial permission dialog.");
        let connected = self.shared.borrow().connected;
        ui.horizontal(|ui| {
            if ui
                .add_enabled(!connected, egui::Button::new("Connect WebSerial"))
                .clicked()
            {
                self.connect();
            }
            if ui
                .add_enabled(connected, egui::Button::new("Disconnect"))
                .clicked()
            {
                self.disconnect();
            }
            if ui
                .add_enabled(connected, egui::Button::new("Version"))
                .clicked()
            {
                self.version();
            }
            if ui
                .add_enabled(connected, egui::Button::new("Status"))
                .clicked()
            {
                self.start_stop_status("Status");
            }
        });
        ui.horizontal(|ui| {
            if ui
                .add_enabled(connected, egui::Button::new("Start emulation"))
                .clicked()
            {
                self.start_stop_status("Start");
            }
            if ui
                .add_enabled(connected, egui::Button::new("Stop emulation"))
                .clicked()
            {
                self.start_stop_status("Stop");
            }
            if ui
                .add_enabled(connected, egui::Button::new("Assert #HOLD"))
                .clicked()
            {
                self.set_hold(true);
            }
            if ui
                .add_enabled(connected, egui::Button::new("Release #HOLD"))
                .clicked()
            {
                self.set_hold(false);
            }
        });
        ui.separator();
        ui.label("CLI parity: version, start, stop, status, hold, read, write, load, dump, configure, monitor, and TOCTOU are available. Serial port listing is replaced by the browser's WebSerial chooser; FT245 FIFO access still requires the native CLI.");
    }

    fn ui_memory(&mut self, ui: &mut egui::Ui) {
        ui.heading("Memory");
        ui.horizontal(|ui| {
            ui.label("Address");
            ui.text_edit_singleline(&mut self.address);
            ui.label("Length");
            ui.text_edit_singleline(&mut self.length);
        });
        ui.horizontal(|ui| {
            if ui.button("Read / dump").clicked() {
                self.read_memory();
            }
            if ui.button("Save last dump").clicked() {
                self.save_download();
            }
        });
        ui.separator();
        ui.label("Write hex bytes");
        ui.text_edit_multiline(&mut self.write_hex);
        if ui.button("Write hex at address").clicked() {
            self.write_hex();
        }
        ui.separator();
        ui.horizontal(|ui| {
            if ui.button("Choose firmware/image file").clicked() {
                self.select_file();
            }
            ui.label(if self.upload_file_name.is_empty() {
                "No file selected"
            } else {
                &self.upload_file_name
            });
        });
        ui.horizontal(|ui| {
            if ui.button("Load file and start emulation").clicked() {
                self.load_file(false);
            }
            if ui.button("Load + verify and start emulation").clicked() {
                self.load_file(true);
            }
        });
    }

    fn ui_chip(&mut self, ui: &mut egui::Ui) {
        ui.heading("Chip configuration");
        ui.horizontal(|ui| {
            ui.label("Search");
            ui.text_edit_singleline(&mut self.chip_query);
            if ui.button("Configure matching chip").clicked() {
                self.configure_chip();
            }
        });
        ui.label(format!(
            "Embedded rflasher chip database: {} chips",
            self.chips.len()
        ));
        egui::ScrollArea::vertical()
            .max_height(350.0)
            .show(ui, |ui| {
                let query = self.chip_query.to_lowercase();
                let rows: Vec<(usize, String)> = self
                    .chips
                    .iter()
                    .enumerate()
                    .filter(|(_, c)| {
                        query.is_empty()
                            || c.name.to_lowercase().contains(&query)
                            || c.vendor.to_lowercase().contains(&query)
                    })
                    .take(200)
                    .map(|(i, c)| {
                        (
                            i,
                            format!(
                                "{} {}  ({} bytes, JEDEC {:02X?})",
                                c.vendor,
                                c.name,
                                c.total_size,
                                c.jedec_id_bytes()
                            ),
                        )
                    })
                    .collect();
                for (i, label) in rows {
                    if ui
                        .selectable_label(self.selected_chip == Some(i), label)
                        .clicked()
                    {
                        self.selected_chip = Some(i);
                        self.chip_query = self.chips[i].name.clone();
                    }
                }
            });
    }

    fn ui_monitor(&mut self, ui: &mut egui::Ui) {
        ui.heading("SPI bus monitor");
        ui.horizontal(|ui| {
            if ui.button("Start monitor").clicked() {
                self.monitor_start();
            }
            if ui.button("Stop monitor").clicked() {
                self.monitor_stop();
            }
            if ui.button("Clear log").clicked() {
                self.shared.borrow_mut().log.clear();
            }
        });
        ui.label("Monitor decodes SPI commands, addresses, transfer ends, TOCTOU trap events, and double-read candidates.");
    }

    fn ui_toctou(&mut self, ui: &mut egui::Ui) {
        ui.heading("TOCTOU traps");
        ui.horizontal(|ui| {
            ui.label("Index");
            ui.text_edit_singleline(&mut self.toctou_index);
            ui.label("Start");
            ui.text_edit_singleline(&mut self.toctou_start);
        });
        ui.horizontal(|ui| {
            ui.label("Mask");
            ui.text_edit_singleline(&mut self.toctou_mask);
            ui.label("Replacement");
            ui.text_edit_singleline(&mut self.toctou_replace);
        });
        ui.horizontal(|ui| {
            if ui.button("Set").clicked() {
                self.toctou_action("set");
            }
            if ui.button("Arm").clicked() {
                self.toctou_action("arm");
            }
            if ui.button("Disarm").clicked() {
                self.toctou_action("disarm");
            }
            if ui.button("Reset").clicked() {
                self.toctou_action("reset");
            }
            if ui.button("Reset all").clicked() {
                self.toctou_action("reset-all");
            }
        });
    }

    fn ui_bitstream(&mut self, ui: &mut egui::Ui) {
        ui.heading("FPGA bitstream");
        ui.label("CI publishes spi_flash.fs next to this WebUI. You can download it, fetch it into the WebUI, or program the Tang Primer 25K SRAM/flash directly through an FTDI JTAG adapter using WebUSB.");
        ui.label("Tip: close/disconnect the WebSerial UART before programming if the browser refuses to claim the FTDI device.");
        ui.horizontal(|ui| {
            ui.label("Bitstream URL");
            ui.text_edit_singleline(&mut self.bitstream_url);
        });
        ui.horizontal(|ui| {
            if ui.button("Download Pages bitstream").clicked() {
                if let Some(window) = web_sys::window() {
                    let _ = window.open_with_url(&self.bitstream_url);
                }
            }
            if ui.button("Fetch URL for programming").clicked() {
                self.fetch_bitstream();
            }
            if ui.button("Choose .fs file").clicked() {
                self.select_bitstream_file();
            }
        });
        ui.horizontal(|ui| {
            ui.label(if self.bitstream_file_name.is_empty() {
                "No bitstream selected"
            } else {
                &self.bitstream_file_name
            });
            if ui
                .add_enabled(
                    self.bitstream_file_data.is_some() && !self.shared.borrow().busy,
                    egui::Button::new("Program SRAM via WebUSB"),
                )
                .clicked()
            {
                self.program_bitstream_sram();
            }
        });
        ui.separator();
        ui.horizontal(|ui| {
            ui.label("Flash offset");
            ui.text_edit_singleline(&mut self.bitstream_flash_offset);
            ui.checkbox(&mut self.bitstream_verify_flash, "verify");
            ui.checkbox(&mut self.bitstream_unprotect_flash, "unprotect");
            ui.checkbox(&mut self.bitstream_chip_erase, "chip erase");
        });
        ui.horizontal(|ui| {
            if ui
                .add_enabled(
                    self.bitstream_file_data.is_some() && !self.shared.borrow().busy,
                    egui::Button::new("Program external flash via WebUSB"),
                )
                .clicked()
            {
                self.program_bitstream_flash();
            }
        });
        ui.horizontal(|ui| {
            if ui.button("Copy native command to log").clicked() {
                self.shared.borrow_mut().log.push_str("Program SRAM with: openFPGALoader -m -b tangprimer25k spi_flash.fs\nFlash with:        openFPGALoader -b tangprimer25k -f spi_flash.fs\n");
            }
        });
        ui.separator();
        ui.label("After programming the bitstream, reconnect WebSerial and use the Device tab to check protocol version/status.");
    }
}

impl eframe::App for NorbertApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let pending_file = { self.shared.borrow_mut().pending_file.take() };
        if let Some((name, data)) = pending_file {
            self.upload_file_name = name.clone();
            self.upload_file_data = Some(data.clone());
            self.set_status(format!("Loaded {name} ({} bytes)", data.len()), false);
        }
        let pending_bitstream_file = { self.shared.borrow_mut().pending_bitstream_file.take() };
        if let Some((name, data)) = pending_bitstream_file {
            self.bitstream_file_name = name.clone();
            self.bitstream_file_data = Some(data.clone());
            match gowin::inspect_bitstream(&data) {
                Ok(info) => self.set_status(
                    format!(
                        "Loaded {name} ({} bytes, target IDCODE 0x{:08x}, checksum 0x{:08x})",
                        data.len(),
                        info.idcode,
                        info.checksum
                    ),
                    false,
                ),
                Err(err) => {
                    self.set_status(format!("Loaded {name}, but .fs parse failed: {err}"), true)
                }
            }
        }

        egui::TopBottomPanel::top("top").show(ctx, |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.heading("NORbert WebUI");
                for (panel, label) in [
                    (Panel::Device, "Device"),
                    (Panel::Memory, "Memory"),
                    (Panel::Chip, "Chip"),
                    (Panel::Monitor, "Monitor"),
                    (Panel::Toctou, "TOCTOU"),
                    (Panel::Bitstream, "Bitstream"),
                ] {
                    if ui.selectable_label(self.panel == panel, label).clicked() {
                        self.panel = panel;
                    }
                }
            });
            let color = if self.status_error {
                egui::Color32::RED
            } else {
                egui::Color32::LIGHT_GREEN
            };
            ui.colored_label(color, &self.status);
            let s = self.shared.borrow();
            if s.busy {
                ui.add(
                    egui::ProgressBar::new(s.progress)
                        .show_percentage()
                        .text(&s.progress_message),
                );
            } else if !s.progress_message.is_empty() {
                ui.label(&s.progress_message);
            }
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            match self.panel {
                Panel::Device => self.ui_device(ui),
                Panel::Memory => self.ui_memory(ui),
                Panel::Chip => self.ui_chip(ui),
                Panel::Monitor => self.ui_monitor(ui),
                Panel::Toctou => self.ui_toctou(ui),
                Panel::Bitstream => self.ui_bitstream(ui),
            }
            ui.separator();
            ui.heading("Log");
            egui::ScrollArea::vertical()
                .stick_to_bottom(true)
                .show(ui, |ui| {
                    let mut log = self.shared.borrow().log.clone();
                    ui.add(
                        egui::TextEdit::multiline(&mut log)
                            .desired_rows(14)
                            .font(egui::TextStyle::Monospace)
                            .interactive(false),
                    );
                });
        });

        ctx.request_repaint_after(std::time::Duration::from_millis(50));
    }
}

fn take_device(shared: &Rc<RefCell<SharedState>>) -> Result<FlashDevice, String> {
    shared
        .borrow_mut()
        .device
        .take()
        .ok_or_else(|| "not connected".to_string())
}

fn put_device(shared: &Rc<RefCell<SharedState>>, dev: FlashDevice) {
    shared.borrow_mut().device = Some(dev);
}

fn parse_u8(s: &str) -> Result<u8, String> {
    parse_u32(s).and_then(|v| u8::try_from(v).map_err(|e| e.to_string()))
}

fn save_file(data: &[u8], filename: &str) {
    let uint8_array = js_sys::Uint8Array::from(data);
    let array = js_sys::Array::new();
    array.push(&uint8_array.buffer());
    let options = web_sys::BlobPropertyBag::new();
    options.set_type("application/octet-stream");
    let Ok(blob) = web_sys::Blob::new_with_u8_array_sequence_and_options(&array, &options) else {
        return;
    };
    let Ok(url) = web_sys::Url::create_object_url_with_blob(&blob) else {
        return;
    };
    let Some(document) = web_sys::window().and_then(|w| w.document()) else {
        return;
    };
    let Ok(element) = document.create_element("a") else {
        return;
    };
    let Ok(a) = element.dyn_into::<web_sys::HtmlAnchorElement>() else {
        return;
    };
    a.set_href(&url);
    a.set_download(filename);
    let _ = a.style().set_property("display", "none");
    if let Some(body) = document.body() {
        let _ = body.append_child(&a);
        a.click();
        let _ = body.remove_child(&a);
    }
    let _ = web_sys::Url::revoke_object_url(&url);
}

#[allow(clippy::main_recursion)]
fn main() {
    console_error_panic_hook::set_once();
    let _ = console_log::init_with_level(log::Level::Debug);

    spawn_local(async {
        let Some(window) = web_sys::window() else {
            return;
        };
        let Some(document) = window.document() else {
            return;
        };
        let Some(canvas) = document
            .get_element_by_id("norbert_canvas")
            .and_then(|e| e.dyn_into::<web_sys::HtmlCanvasElement>().ok())
        else {
            return;
        };

        let result = eframe::WebRunner::new()
            .start(
                canvas,
                eframe::WebOptions::default(),
                Box::new(|cc| Ok(Box::new(NorbertApp::new(cc)))),
            )
            .await;

        if let Some(loading) = document.get_element_by_id("loading") {
            if let Ok(loading) = loading.dyn_into::<web_sys::HtmlElement>() {
                let _ = loading.style().set_property("display", "none");
            }
        }
        if let Err(e) = result {
            web_sys::console::error_1(&format!("eframe failed: {e:?}").into());
        }
    });
}
