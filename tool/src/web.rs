use crate::device::{
    ConnectionKind, FT2232H_PID, FTDI_VID, FlashDevice, Transport, UART_BAUD_RATE,
};
use crate::protocol::{CMD_VERSION, is_supported_protocol_version};
use anyhow::{Result, anyhow, bail};
use ftdi_nusb::{FtdiDevice, Interface};
use futures_util::{future::Either, pin_mut};
use gloo_timers::future::TimeoutFuture;
use js_sys::{Function, Promise, Reflect, Uint8Array};
use std::collections::VecDeque;
use std::future::Future;
use wasm_bindgen::{JsCast, prelude::*};
use wasm_bindgen_futures::JsFuture;
use web_sys::{
    File, ReadableStreamDefaultReader, SerialOptions, SerialPort, UsbDevice, UsbDeviceFilter,
    UsbDeviceRequestOptions, WritableStreamDefaultWriter,
};

const IO_TIMEOUT_MS: u32 = 5_000;
const SERIAL_QUIET_MS: u32 = 50;
const FILE_CHUNK_SIZE: usize = 1024 * 1024;

fn file_length(file: &File) -> Result<usize> {
    let size = file.size();
    if !size.is_finite() || size <= 0.0 || size > u32::MAX as f64 {
        bail!("file size must be between 1 byte and 4 GiB");
    }
    Ok(size as usize)
}

fn js_error(error: impl std::fmt::Display) -> JsValue {
    JsValue::from_str(&format!("{error:#}"))
}

fn js_value_text(error: &JsValue) -> String {
    error.as_string().unwrap_or_else(|| format!("{error:?}"))
}

fn callback_result(result: JsValue) -> Result<()> {
    if result.as_bool() == Some(false) {
        bail!("operation cancelled");
    }
    Ok(())
}

fn with_cleanup_error(primary: anyhow::Error, cleanup: Result<()>) -> anyhow::Error {
    match cleanup {
        Ok(()) => primary,
        Err(cleanup_error) => {
            anyhow!("{primary:#}; connection cleanup also failed: {cleanup_error:#}")
        }
    }
}

async fn timeout<F, T>(future: F, milliseconds: u32, context: &str) -> Result<T>
where
    F: Future<Output = T>,
{
    let timer = TimeoutFuture::new(milliseconds);
    pin_mut!(future);
    pin_mut!(timer);
    match futures_util::future::select(future, timer).await {
        Either::Left((value, _)) => Ok(value),
        Either::Right(((), _)) => Err(anyhow!("{context} timed out")),
    }
}

struct WebSerialTransport {
    port: SerialPort,
    reader: Option<ReadableStreamDefaultReader>,
    writer: Option<WritableStreamDefaultWriter>,
    pending: VecDeque<u8>,
    pending_read: Option<Promise>,
    open: bool,
}

impl WebSerialTransport {
    async fn request() -> Result<(Self, u8)> {
        let window = web_sys::window().ok_or_else(|| anyhow!("window is unavailable"))?;
        let selected = JsFuture::from(window.navigator().serial().request_port())
            .await
            .map_err(|error| anyhow!("Web Serial request failed: {}", js_value_text(&error)))?;
        let port: SerialPort = selected
            .dyn_into()
            .map_err(|_| anyhow!("browser returned an invalid serial port"))?;

        let options = SerialOptions::new(UART_BAUD_RATE);
        options.set_buffer_size(65_536);
        // Opening promises cannot be cancelled safely: a timed-out open may
        // resolve later and leave a port open without an owner to close it.
        JsFuture::from(port.open(&options)).await.map_err(|error| {
            anyhow!("opening Web Serial port failed: {}", js_value_text(&error))
        })?;

        let mut transport = Self {
            port,
            reader: None,
            writer: None,
            pending: VecDeque::new(),
            pending_read: None,
            open: true,
        };

        let setup = async {
            transport.reader = Some(
                ReadableStreamDefaultReader::new(&transport.port.readable())
                    .map_err(|error| anyhow!(js_value_text(&error)))?,
            );
            transport.writer = Some(
                WritableStreamDefaultWriter::new(&transport.port.writable())
                    .map_err(|error| anyhow!(js_value_text(&error)))?,
            );
            transport.resync().await
        }
        .await;

        match setup {
            Ok(version) => Ok((transport, version)),
            Err(error) => {
                let cleanup = transport.shutdown().await;
                Err(with_cleanup_error(error, cleanup))
            }
        }
    }

    async fn resync(&mut self) -> Result<u8> {
        // Send exactly one VERSION request. Reading until its supported reply
        // arrives avoids the old retry race where a late reply from attempt N
        // was mistaken for attempt N+1 and left another reply queued.
        TimeoutFuture::new(20).await;
        self.write_bytes(&[CMD_VERSION]).await?;

        let mut examined = 0usize;
        let version = loop {
            let bytes = self
                .read_chunk(256, IO_TIMEOUT_MS, true)
                .await?
                .ok_or_else(|| anyhow!("Web Serial synchronization timed out"))?;
            examined += bytes.len();
            if let Some(version) = bytes
                .iter()
                .rev()
                .copied()
                .find(|version| is_supported_protocol_version(*version))
            {
                break version;
            }
            if examined > 1024 {
                bail!("too much startup noise while synchronizing Web Serial");
            }
        };

        // Drain late startup glitches until the line has been quiet for a
        // short period. A timed-out read promise is retained and reused by the
        // next protocol read, because Web Serial read promises are not
        // cancellable without closing the stream.
        loop {
            match self.read_chunk(256, SERIAL_QUIET_MS, false).await? {
                Some(_) => continue,
                None => return Ok(version),
            }
        }
    }

    fn next_read_promise(&mut self) -> Result<Promise> {
        if let Some(promise) = self.pending_read.take() {
            return Ok(promise);
        }
        let reader = self
            .reader
            .as_ref()
            .ok_or_else(|| anyhow!("Web Serial connection is closed"))?;
        Ok(reader.read())
    }

    async fn read_chunk(
        &mut self,
        maximum: usize,
        milliseconds: u32,
        timeout_is_fatal: bool,
    ) -> Result<Option<Vec<u8>>> {
        if !self.pending.is_empty() {
            let count = maximum.min(self.pending.len());
            return Ok(Some(self.pending.drain(..count).collect()));
        }

        let promise = self.next_read_promise()?;
        let read = timeout(
            JsFuture::from(promise.clone()),
            milliseconds,
            "Web Serial read",
        )
        .await;
        let result = match read {
            Ok(Ok(result)) => result,
            Ok(Err(error)) => {
                let primary = anyhow!("Web Serial read failed: {}", js_value_text(&error));
                let cleanup = self.shutdown().await;
                return Err(with_cleanup_error(primary, cleanup));
            }
            Err(_error) if !timeout_is_fatal => {
                self.pending_read = Some(promise);
                return Ok(None);
            }
            Err(error) => {
                let cleanup = self.shutdown().await;
                return Err(with_cleanup_error(error, cleanup));
            }
        };

        let parsed = (|| -> Result<Option<Vec<u8>>> {
            let done = Reflect::get(&result, &JsValue::from_str("done"))
                .map_err(|error| anyhow!(js_value_text(&error)))?
                .as_bool()
                .unwrap_or(false);
            if done {
                bail!("Web Serial stream closed");
            }

            let value = Reflect::get(&result, &JsValue::from_str("value"))
                .map_err(|error| anyhow!(js_value_text(&error)))?;
            if value.is_null() || value.is_undefined() {
                return Ok(Some(Vec::new()));
            }
            let bytes = Uint8Array::new(&value).to_vec();
            let count = maximum.min(bytes.len());
            self.pending.extend(bytes[count..].iter().copied());
            Ok(Some(bytes[..count].to_vec()))
        })();

        match parsed {
            Ok(value) => Ok(value),
            Err(error) => {
                let cleanup = self.shutdown().await;
                Err(with_cleanup_error(error, cleanup))
            }
        }
    }

    async fn write_bytes(&mut self, data: &[u8]) -> Result<()> {
        let bytes = Uint8Array::from(data);
        let write = {
            let writer = self
                .writer
                .as_ref()
                .ok_or_else(|| anyhow!("Web Serial connection is closed"))?;
            timeout(
                JsFuture::from(writer.write_with_chunk(bytes.as_ref())),
                IO_TIMEOUT_MS,
                "Web Serial write",
            )
            .await
        };
        match write {
            Ok(Ok(_)) => Ok(()),
            Ok(Err(error)) => {
                let primary = anyhow!("Web Serial write failed: {}", js_value_text(&error));
                let cleanup = self.shutdown().await;
                Err(with_cleanup_error(primary, cleanup))
            }
            Err(error) => {
                let cleanup = self.shutdown().await;
                Err(with_cleanup_error(error, cleanup))
            }
        }
    }

    async fn shutdown(&mut self) -> Result<()> {
        let mut failures = Vec::new();
        self.pending.clear();
        self.pending_read = None;

        if let Some(reader) = self.reader.take() {
            if let Err(error) = timeout(
                JsFuture::from(reader.cancel()),
                IO_TIMEOUT_MS,
                "cancelling Web Serial reader",
            )
            .await
            .and_then(|result| {
                result
                    .map(|_| ())
                    .map_err(|error| anyhow!(js_value_text(&error)))
            }) {
                failures.push(format!("{error:#}"));
            }
            reader.release_lock();
        }

        if let Some(writer) = self.writer.take() {
            if let Err(error) = timeout(
                JsFuture::from(writer.abort()),
                IO_TIMEOUT_MS,
                "aborting Web Serial writer",
            )
            .await
            .and_then(|result| {
                result
                    .map(|_| ())
                    .map_err(|error| anyhow!(js_value_text(&error)))
            }) {
                failures.push(format!("{error:#}"));
            }
            writer.release_lock();
        }

        if self.open {
            self.open = false;
            if let Err(error) = timeout(
                JsFuture::from(self.port.close()),
                IO_TIMEOUT_MS,
                "closing Web Serial port",
            )
            .await
            .and_then(|result| {
                result
                    .map(|_| ())
                    .map_err(|error| anyhow!(js_value_text(&error)))
            }) {
                failures.push(format!("{error:#}"));
            }
        }

        if failures.is_empty() {
            Ok(())
        } else {
            bail!(failures.join("; "))
        }
    }
}

#[maybe_async::maybe_async(?Send)]
impl Transport for WebSerialTransport {
    async fn write_all(&mut self, data: &[u8]) -> Result<()> {
        self.write_bytes(data).await
    }

    async fn read(&mut self, buffer: &mut [u8]) -> Result<usize> {
        loop {
            let bytes = self
                .read_chunk(buffer.len(), IO_TIMEOUT_MS, true)
                .await?
                .ok_or_else(|| anyhow!("Web Serial read timed out"))?;
            if bytes.is_empty() {
                continue;
            }
            buffer[..bytes.len()].copy_from_slice(&bytes);
            return Ok(bytes.len());
        }
    }

    async fn disconnect(&mut self) -> Result<()> {
        self.shutdown().await
    }

    fn is_connected(&self) -> bool {
        self.open
    }
}

struct WebUsbTransport {
    device: Option<Box<FtdiDevice>>,
}

impl WebUsbTransport {
    async fn request() -> Result<Self> {
        let window = web_sys::window().ok_or_else(|| anyhow!("window is unavailable"))?;
        let filter = UsbDeviceFilter::new();
        filter.set_vendor_id(FTDI_VID);
        filter.set_product_id(FT2232H_PID);
        let options = UsbDeviceRequestOptions::new(&[filter]);
        let selected = JsFuture::from(window.navigator().usb().request_device(&options))
            .await
            .map_err(|error| anyhow!("WebUSB request failed: {}", js_value_text(&error)))?;
        let usb_device: UsbDevice = selected
            .dyn_into()
            .map_err(|_| anyhow!("browser returned an invalid USB device"))?;
        let nusb_device = nusb::Device::from_js(usb_device)
            .await
            .map_err(|error| anyhow!("opening WebUSB device failed: {error}"))?;
        let device = FtdiDevice::open_wasm(nusb_device, Interface::A)
            .await
            .map_err(|error| anyhow!("opening FT2232H channel A failed: {error}"))?;

        let mut transport = Self {
            device: Some(Box::new(device)),
        };
        let setup = transport.initialize().await;
        match setup {
            Ok(()) => Ok(transport),
            Err(error) => {
                let cleanup = transport.shutdown().await;
                Err(with_cleanup_error(error, cleanup))
            }
        }
    }

    fn device_mut(&mut self) -> Result<&mut FtdiDevice> {
        self.device
            .as_deref_mut()
            .ok_or_else(|| anyhow!("WebUSB connection is closed"))
    }

    async fn initialize(&mut self) -> Result<()> {
        let device = self.device_mut()?;
        timeout(device.usb_reset(), IO_TIMEOUT_MS, "FT2232H reset")
            .await?
            .map_err(|error| anyhow!(error))?;
        timeout(device.flush_all(), IO_TIMEOUT_MS, "FT2232H flush")
            .await?
            .map_err(|error| anyhow!(error))?;
        timeout(
            device.set_latency_timer(1),
            IO_TIMEOUT_MS,
            "setting FT2232H latency timer",
        )
        .await?
        .map_err(|error| anyhow!(error))?;
        device.set_read_chunksize(65_536);
        device.set_write_chunksize(65_536);

        TimeoutFuture::new(5).await;
        timeout(device.flush_all(), IO_TIMEOUT_MS, "FT2232H flush")
            .await?
            .map_err(|error| anyhow!(error))?;
        Ok(())
    }

    async fn shutdown(&mut self) -> Result<()> {
        let Some(mut device) = self.device.take() else {
            return Ok(());
        };
        timeout(device.shutdown(), IO_TIMEOUT_MS, "closing WebUSB device").await
    }

    async fn fail<T>(&mut self, primary: anyhow::Error) -> Result<T> {
        let cleanup = self.shutdown().await;
        Err(with_cleanup_error(primary, cleanup))
    }
}

#[maybe_async::maybe_async(?Send)]
impl Transport for WebUsbTransport {
    async fn write_all(&mut self, data: &[u8]) -> Result<()> {
        let result = {
            let device = self.device_mut()?;
            timeout(device.write_all(data), IO_TIMEOUT_MS, "WebUSB write").await
        };
        match result {
            Ok(Ok(())) => Ok(()),
            Ok(Err(error)) => self.fail(anyhow!("WebUSB write failed: {error}")).await,
            Err(error) => self.fail(error).await,
        }
    }

    async fn read(&mut self, buffer: &mut [u8]) -> Result<usize> {
        let result = {
            let device = self.device_mut()?;
            timeout(device.read_data(buffer), IO_TIMEOUT_MS, "WebUSB read").await
        };
        match result {
            Ok(Ok(count)) => Ok(count),
            Ok(Err(error)) => self.fail(anyhow!("WebUSB read failed: {error}")).await,
            Err(error) => self.fail(error).await,
        }
    }

    async fn disconnect(&mut self) -> Result<()> {
        self.shutdown().await
    }

    fn is_connected(&self) -> bool {
        self.device.is_some()
    }
}

/// Browser connection to NORbert over WebUSB or Web Serial.
#[wasm_bindgen]
pub struct WebFlashDevice {
    inner: FlashDevice,
}

#[wasm_bindgen]
impl WebFlashDevice {
    /// Show the WebUSB picker filtered to FT2232H devices and open channel A.
    #[wasm_bindgen(js_name = requestUsb)]
    pub async fn request_usb() -> Result<WebFlashDevice, JsValue> {
        let transport = WebUsbTransport::request().await.map_err(js_error)?;
        let inner = FlashDevice::new(transport, ConnectionKind::Ft245, None).map_err(js_error)?;
        Ok(Self { inner })
    }

    /// Show the Web Serial picker, open at 2 Mbaud, and synchronize with NORbert.
    #[wasm_bindgen(js_name = requestSerial)]
    pub async fn request_serial() -> Result<WebFlashDevice, JsValue> {
        let (transport, version) = WebSerialTransport::request().await.map_err(js_error)?;
        let inner =
            FlashDevice::new(transport, ConnectionKind::Uart, Some(version)).map_err(js_error)?;
        Ok(Self { inner })
    }

    pub fn is_connected(&self) -> bool {
        self.inner.is_connected()
    }

    pub async fn disconnect(&mut self) -> Result<(), JsValue> {
        self.inner.disconnect().await.map_err(js_error)
    }

    pub async fn get_version(&mut self) -> Result<u8, JsValue> {
        self.inner.get_version().await.map_err(js_error)
    }

    pub async fn supports_emulation_control(&mut self) -> Result<bool, JsValue> {
        self.inner
            .capabilities()
            .await
            .map(|capabilities| capabilities.emulation_control)
            .map_err(js_error)
    }

    pub async fn supports_activity_log(&mut self) -> Result<bool, JsValue> {
        self.inner
            .capabilities()
            .await
            .map(|capabilities| capabilities.activity_log)
            .map_err(js_error)
    }

    pub async fn start(&mut self) -> Result<(), JsValue> {
        self.inner.start_emulation().await.map_err(js_error)
    }

    pub async fn stop(&mut self) -> Result<(), JsValue> {
        self.inner.stop_emulation().await.map_err(js_error)
    }

    pub async fn status(&mut self) -> Result<bool, JsValue> {
        self.inner.status().await.map_err(js_error)
    }

    pub async fn read(&mut self, address: u32, length: u32) -> Result<Vec<u8>, JsValue> {
        self.inner.read(address, length).await.map_err(js_error)
    }

    /// Stream read chunks to JavaScript. The callback receives
    /// `(chunk, completed, total)` and may return `false` to cancel.
    pub async fn read_chunks(
        &mut self,
        address: u32,
        length: u32,
        on_chunk: Function,
    ) -> Result<(), JsValue> {
        let mut completed = 0usize;
        self.inner
            .read_chunks(address, length, |_, chunk| {
                completed += chunk.len();
                let bytes = Uint8Array::from(chunk);
                let result = on_chunk
                    .call3(
                        &JsValue::UNDEFINED,
                        bytes.as_ref(),
                        &JsValue::from_f64(completed as f64),
                        &JsValue::from_f64(length as f64),
                    )
                    .map_err(|error| anyhow!(js_value_text(&error)))?;
                callback_result(result)
            })
            .await
            .map_err(js_error)
    }

    pub async fn write(&mut self, address: u32, data: Vec<u8>) -> Result<(), JsValue> {
        self.inner.write(address, &data).await.map_err(js_error)
    }

    /// Read a browser File in bounded chunks and write it through the shared
    /// transfer session. The callback receives `(completed, total)` and may
    /// return `false` to cancel.
    pub async fn write_file(
        &mut self,
        address: u32,
        file: File,
        on_progress: Function,
    ) -> Result<(), JsValue> {
        let length = file_length(&file).map_err(js_error)?;
        let mut session = self
            .inner
            .begin_write(address, length)
            .await
            .map_err(js_error)?;
        let operation = async {
            let mut offset = 0usize;
            let mut completed = 0usize;
            while offset < length {
                let end = (offset + FILE_CHUNK_SIZE).min(length);
                let blob = file
                    .slice_with_f64_and_f64(offset as f64, end as f64)
                    .map_err(|error| anyhow!(js_value_text(&error)))?;
                let buffer = JsFuture::from(blob.array_buffer())
                    .await
                    .map_err(|error| anyhow!(js_value_text(&error)))?;
                let data = Uint8Array::new(&buffer).to_vec();
                session
                    .write_chunk(&data, |written| {
                        completed += written;
                        let result = on_progress
                            .call2(
                                &JsValue::UNDEFINED,
                                &JsValue::from_f64(completed as f64),
                                &JsValue::from_f64(length as f64),
                            )
                            .map_err(|error| anyhow!(js_value_text(&error)))?;
                        callback_result(result)
                    })
                    .await?;
                offset = end;
            }
            Ok(())
        }
        .await;
        session.finish(operation).await.map_err(js_error)
    }

    /// Verify a browser File against SDRAM without retaining either the full
    /// file or a full-device readback in WASM memory.
    pub async fn verify_file(
        &mut self,
        address: u32,
        file: File,
        on_progress: Function,
    ) -> Result<(), JsValue> {
        let length = file_length(&file).map_err(js_error)?;
        let was_running = self.inner.prepare_stopped().await.map_err(js_error)?;
        let operation = async {
            let mut offset = 0usize;
            let mut completed = 0usize;
            while offset < length {
                let end = (offset + FILE_CHUNK_SIZE).min(length);
                let blob = file
                    .slice_with_f64_and_f64(offset as f64, end as f64)
                    .map_err(|error| anyhow!(js_value_text(&error)))?;
                let buffer = JsFuture::from(blob.array_buffer())
                    .await
                    .map_err(|error| anyhow!(js_value_text(&error)))?;
                let expected = Uint8Array::new(&buffer).to_vec();
                if expected.len() != end - offset {
                    bail!("browser returned a truncated file chunk");
                }

                let chunk_address = address
                    .checked_add(offset as u32)
                    .ok_or_else(|| anyhow!("verification address overflow"))?;
                self.inner
                    .read_raw_chunks(chunk_address, expected.len() as u32, |actual_address, actual| {
                        let expected_offset = (actual_address - chunk_address) as usize;
                        let expected_chunk = &expected[expected_offset..expected_offset + actual.len()];
                        if let Some((index, (&wanted, &found))) = expected_chunk
                            .iter()
                            .zip(actual)
                            .enumerate()
                            .find(|(_, (wanted, found))| wanted != found)
                        {
                            bail!(
                                "verification failed at 0x{:08x}: expected 0x{wanted:02x}, got 0x{found:02x}",
                                actual_address + index as u32
                            );
                        }
                        completed += actual.len();
                        let result = on_progress
                            .call2(
                                &JsValue::UNDEFINED,
                                &JsValue::from_f64(completed as f64),
                                &JsValue::from_f64(length as f64),
                            )
                            .map_err(|error| anyhow!(js_value_text(&error)))?;
                        callback_result(result)
                    })
                    .await?;
                offset = end;
            }
            Ok(())
        }
        .await;
        self.inner
            .finish_stopped(was_running, operation)
            .await
            .map_err(js_error)
    }

    pub async fn configure(
        &mut self,
        jedec_id: Vec<u8>,
        total_size: u32,
        supports_4byte: bool,
        sfdp_table: Vec<u8>,
    ) -> Result<(), JsValue> {
        let jedec_id: [u8; 3] = jedec_id
            .try_into()
            .map_err(|_| js_error("JEDEC ID must contain exactly 3 bytes"))?;
        self.inner
            .configure(jedec_id, total_size, supports_4byte, &sfdp_table)
            .await
            .map_err(js_error)
    }

    pub async fn set_hold(&mut self, enabled: bool) -> Result<(), JsValue> {
        self.inner.set_hold(enabled).await.map_err(js_error)
    }

    pub async fn log_start(&mut self) -> Result<(), JsValue> {
        self.inner.log_start().await.map_err(js_error)
    }

    pub async fn log_stop(&mut self) -> Result<(), JsValue> {
        self.inner.log_stop().await.map_err(js_error)
    }

    pub async fn log_poll(&mut self) -> Result<Vec<u8>, JsValue> {
        self.inner.log_poll().await.map_err(js_error)
    }

    pub async fn toctou_set(
        &mut self,
        index: u8,
        start: u32,
        mask: u32,
        replace: u32,
    ) -> Result<(), JsValue> {
        self.inner
            .toctou_set(index, start, mask, replace)
            .await
            .map_err(js_error)
    }

    pub async fn toctou_arm(&mut self, index: u8) -> Result<(), JsValue> {
        self.inner.toctou_arm(index).await.map_err(js_error)
    }

    pub async fn toctou_disarm(&mut self, index: u8) -> Result<(), JsValue> {
        self.inner.toctou_disarm(index).await.map_err(js_error)
    }

    pub async fn toctou_reset(&mut self, index: u8) -> Result<(), JsValue> {
        self.inner.toctou_reset(index).await.map_err(js_error)
    }

    pub async fn toctou_reset_all(&mut self) -> Result<(), JsValue> {
        self.inner.toctou_reset_all().await.map_err(js_error)
    }
}
