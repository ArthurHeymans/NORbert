#![cfg(target_arch = "wasm32")]

use egui::Color32;
use gloo_timers::future::TimeoutFuture;
use js_sys::{Array, Uint8Array};
use spi_flash_tool::WebFlashDevice;
use spi_flash_tool::chip::{FlashChip, FlashChipExt};
use spi_flash_tool::sfdp::generate_sfdp;
use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;
use wasm_bindgen::JsCast;
use wasm_bindgen::closure::Closure;
use wasm_bindgen_futures::{JsFuture, spawn_local};
use web_sys::{Blob, BlobPropertyBag, HtmlAnchorElement, HtmlCanvasElement, HtmlInputElement, Url};

#[derive(Default, PartialEq, Clone, Copy)]
enum Panel {
    #[default]
    Device,
    Activity,
    Toctou,
}

#[derive(Default)]
enum ConnectionState {
    #[default]
    Disconnected,
    Connecting,
    Connected,
    Error(String),
}

struct SharedState {
    device: Option<WebFlashDevice>,
    connection: ConnectionState,
    busy: bool,
    version: Option<u8>,
    running: Option<bool>,
    emulation_control: bool,
    activity_log: bool,
    hold_enabled: Option<bool>,
    status: String,
    status_error: bool,
    pending_file: Option<(String, Vec<u8>)>,
    read_data: Option<Vec<u8>>,
    log_output: String,
    log_pending: Vec<u8>,
    log_txn_count: u32,
    log_opcode: u8,
    log_address: u32,
    log_line_open: bool,
    log_accesses: HashMap<(u32, u8), u32>,
    monitoring: bool,
    configured_chip: Option<Rc<FlashChip>>,
}

impl Default for SharedState {
    fn default() -> Self {
        Self {
            device: None,
            connection: ConnectionState::Disconnected,
            busy: false,
            version: None,
            running: None,
            emulation_control: false,
            activity_log: false,
            hold_enabled: None,
            status: "Choose a transport to connect to NORbert".to_owned(),
            status_error: false,
            pending_file: None,
            read_data: None,
            log_output: String::new(),
            log_pending: Vec::new(),
            log_txn_count: 0,
            log_opcode: 0,
            log_address: 0,
            log_line_open: false,
            log_accesses: HashMap::new(),
            monitoring: false,
            configured_chip: None,
        }
    }
}

struct ChipInfo {
    chip: Rc<FlashChip>,
    display_name: String,
}

struct NorbertWebApp {
    state: Rc<RefCell<SharedState>>,
    available_chips: Vec<ChipInfo>,
    selected_chip: Option<Rc<FlashChip>>,
    chip_search: String,
    panel: Panel,
    read_address: String,
    read_length: String,
    write_address: String,
    upload_name: String,
    upload_data: Option<Vec<u8>>,
    verify_upload: bool,
    toctou_index: String,
    toctou_start: String,
    toctou_mask: String,
    toctou_replace: String,
}

impl NorbertWebApp {
    fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        let available_chips = spi_flash_tool::chip::ChipDatabase::new()
            .iter()
            .cloned()
            .map(|chip| {
                let display_name = format!("{} {}", chip.vendor, chip.name);
                ChipInfo {
                    chip: Rc::new(chip),
                    display_name,
                }
            })
            .collect();

        Self {
            state: Rc::new(RefCell::new(SharedState::default())),
            available_chips,
            selected_chip: None,
            chip_search: String::new(),
            panel: Panel::Device,
            read_address: "0x000000".to_owned(),
            read_length: "256".to_owned(),
            write_address: "0x000000".to_owned(),
            upload_name: String::new(),
            upload_data: None,
            verify_upload: false,
            toctou_index: "0".to_owned(),
            toctou_start: "0x001000".to_owned(),
            toctou_mask: "0xFFF000".to_owned(),
            toctou_replace: "0x101000".to_owned(),
        }
    }

    fn connect(&self, usb: bool, ctx: &egui::Context) {
        {
            let mut state = self.state.borrow_mut();
            state.connection = ConnectionState::Connecting;
            state.busy = true;
            state.status = if usb {
                "Requesting FT245 device access..."
            } else {
                "Requesting UART access..."
            }
            .to_owned();
            state.status_error = false;
        }

        let state = self.state.clone();
        let repaint = ctx.clone();
        spawn_local(async move {
            let result = if usb {
                WebFlashDevice::request_usb().await
            } else {
                WebFlashDevice::request_serial().await
            };

            match result {
                Ok(mut device) => {
                    let details = async {
                        let version = device.get_version().await?;
                        let emulation_control = device.supports_emulation_control().await?;
                        let activity_log = device.supports_activity_log().await?;
                        let running = if emulation_control {
                            Some(device.status().await?)
                        } else {
                            None
                        };
                        Ok::<_, wasm_bindgen::JsValue>((
                            version,
                            emulation_control,
                            activity_log,
                            running,
                        ))
                    }
                    .await;

                    let mut shared = state.borrow_mut();
                    match details {
                        Ok((version, emulation_control, activity_log, running)) => {
                            shared.device = Some(device);
                            shared.connection = ConnectionState::Connected;
                            shared.version = Some(version);
                            shared.emulation_control = emulation_control;
                            shared.activity_log = activity_log;
                            shared.running = running;
                            shared.status = "Connected successfully".to_owned();
                            shared.status_error = false;
                        }
                        Err(error) => set_error(
                            &mut shared,
                            format!("Connection failed: {}", js_error(error)),
                        ),
                    }
                }
                Err(error) => {
                    let mut shared = state.borrow_mut();
                    shared.connection = ConnectionState::Error(js_error(error.clone()));
                    set_error(
                        &mut shared,
                        format!("Connection failed: {}", js_error(error)),
                    );
                }
            }
            state.borrow_mut().busy = false;
            repaint.request_repaint();
        });
    }

    fn disconnect(&self, ctx: &egui::Context) {
        let state = self.state.clone();
        let repaint = ctx.clone();
        state.borrow_mut().busy = true;
        spawn_local(async move {
            let mut device = state.borrow_mut().device.take();
            let result = match device.as_mut() {
                Some(device) => device.disconnect().await,
                None => Ok(()),
            };
            let mut shared = state.borrow_mut();
            shared.busy = false;
            shared.version = None;
            shared.running = None;
            shared.emulation_control = false;
            shared.activity_log = false;
            shared.hold_enabled = None;
            shared.configured_chip = None;
            shared.connection = ConnectionState::Disconnected;
            match result {
                Ok(()) => {
                    shared.status = "Disconnected".to_owned();
                    shared.status_error = false;
                }
                Err(error) => set_error(
                    &mut shared,
                    format!("Disconnect failed: {}", js_error(error)),
                ),
            }
            repaint.request_repaint();
        });
    }

    fn set_running(&self, running: bool, ctx: &egui::Context) {
        let state = self.state.clone();
        let repaint = ctx.clone();
        {
            let mut shared = state.borrow_mut();
            shared.busy = true;
            shared.status = if running {
                "Starting emulation..."
            } else {
                "Stopping emulation..."
            }
            .to_owned();
        }
        spawn_local(async move {
            let mut device = state.borrow_mut().device.take();
            let result = match device.as_mut() {
                Some(device) if running => device.start().await,
                Some(device) => device.stop().await,
                None => Err(wasm_bindgen::JsValue::from_str("No device connected")),
            };
            let mut shared = state.borrow_mut();
            shared.device = device;
            shared.busy = false;
            match result {
                Ok(()) => {
                    shared.running = Some(running);
                    shared.status = if running {
                        "Emulation started"
                    } else {
                        "Emulation stopped"
                    }
                    .to_owned();
                    shared.status_error = false;
                }
                Err(error) => set_error(
                    &mut shared,
                    format!("Operation failed: {}", js_error(error)),
                ),
            }
            repaint.request_repaint();
        });
    }

    fn refresh(&self, ctx: &egui::Context) {
        let state = self.state.clone();
        let repaint = ctx.clone();
        state.borrow_mut().busy = true;
        spawn_local(async move {
            let mut device = state.borrow_mut().device.take();
            let result = match device.as_mut() {
                Some(device) => device.status().await,
                None => Err(wasm_bindgen::JsValue::from_str("No device connected")),
            };
            let mut shared = state.borrow_mut();
            shared.device = device;
            shared.busy = false;
            match result {
                Ok(running) => {
                    shared.running = Some(running);
                    shared.status = "Status refreshed".to_owned();
                    shared.status_error = false;
                }
                Err(error) => set_error(&mut shared, format!("Status failed: {}", js_error(error))),
            }
            repaint.request_repaint();
        });
    }

    fn configure_chip(&self, chip: Rc<FlashChip>, ctx: &egui::Context) {
        let sfdp = match generate_sfdp(&chip) {
            Ok(table) => table.to_vec(),
            Err(error) => {
                set_error(
                    &mut self.state.borrow_mut(),
                    format!("Failed to generate SFDP: {error}"),
                );
                return;
            }
        };
        let jedec_id = chip.jedec_id_bytes().to_vec();
        let total_size = chip.total_size;
        let supports_4byte = chip.supports_4byte();
        let state = self.state.clone();
        let repaint = ctx.clone();
        {
            let mut shared = state.borrow_mut();
            shared.busy = true;
            shared.configured_chip = None;
            shared.status = format!("Configuring {} {}...", chip.vendor, chip.name);
            shared.status_error = false;
        }

        spawn_local(async move {
            let mut device = state.borrow_mut().device.take();
            let result = match device.as_mut() {
                Some(device) => {
                    device
                        .configure(jedec_id, total_size, supports_4byte, sfdp)
                        .await
                }
                None => Err(wasm_bindgen::JsValue::from_str("No device connected")),
            };
            let running = if result.is_ok() {
                match device.as_mut() {
                    Some(device) => device.status().await.ok(),
                    None => None,
                }
            } else {
                None
            };
            let mut shared = state.borrow_mut();
            shared.device = device;
            shared.busy = false;
            match result {
                Ok(()) => {
                    shared.configured_chip = Some(chip.clone());
                    shared.running = running;
                    shared.status = format!(
                        "Configured {} {} ({} MiB, JEDEC {:02X} {:04X})",
                        chip.vendor,
                        chip.name,
                        chip.total_size / (1024 * 1024),
                        chip.jedec_manufacturer,
                        chip.jedec_device,
                    );
                    shared.status_error = false;
                }
                Err(error) => set_error(
                    &mut shared,
                    format!("Chip configuration failed: {}", js_error(error)),
                ),
            }
            repaint.request_repaint();
        });
    }

    fn select_file(&self) {
        let document = web_sys::window().unwrap().document().unwrap();
        let input: HtmlInputElement = document
            .create_element("input")
            .unwrap()
            .dyn_into()
            .unwrap();
        input.set_type("file");
        input.set_accept(".bin,.rom,.img,*");
        let state = self.state.clone();
        let onchange = Closure::wrap(Box::new(move |event: web_sys::Event| {
            let input: HtmlInputElement = event.target().unwrap().dyn_into().unwrap();
            if let Some(file) = input.files().and_then(|files| files.get(0)) {
                let filename = file.name();
                let state = state.clone();
                spawn_local(async move {
                    match JsFuture::from(file.array_buffer()).await {
                        Ok(buffer) => {
                            let data = Uint8Array::new(&buffer).to_vec();
                            let mut shared = state.borrow_mut();
                            shared.pending_file = Some((filename, data));
                            shared.status = "File loaded".to_owned();
                            shared.status_error = false;
                        }
                        Err(error) => set_error(
                            &mut state.borrow_mut(),
                            format!("File read failed: {}", js_error(error)),
                        ),
                    }
                });
            }
        }) as Box<dyn FnMut(_)>);
        input.set_onchange(Some(onchange.as_ref().unchecked_ref()));
        onchange.forget();
        input.click();
    }

    fn write_memory(&self, address: u32, data: Vec<u8>, verify: bool, ctx: &egui::Context) {
        let state = self.state.clone();
        let repaint = ctx.clone();
        state.borrow_mut().busy = true;
        spawn_local(async move {
            let mut device = state.borrow_mut().device.take();
            let result = match device.as_mut() {
                Some(device) => match device.write(address, data.clone()).await {
                    Ok(()) if verify => match device.read(address, data.len() as u32).await {
                        Ok(actual) if actual == data => Ok(()),
                        Ok(_) => Err(wasm_bindgen::JsValue::from_str(
                            "Verification failed: device contents differ from the selected file",
                        )),
                        Err(error) => Err(error),
                    },
                    result => result,
                },
                None => Err(wasm_bindgen::JsValue::from_str("No device connected")),
            };
            let mut shared = state.borrow_mut();
            shared.device = device;
            shared.busy = false;
            match result {
                Ok(()) => {
                    shared.running = Some(true);
                    shared.status = "Upload complete".to_owned();
                    shared.status_error = false;
                }
                Err(error) => set_error(&mut shared, format!("Upload failed: {}", js_error(error))),
            }
            repaint.request_repaint();
        });
    }

    fn read_memory(&self, address: u32, length: u32, ctx: &egui::Context) {
        let state = self.state.clone();
        let repaint = ctx.clone();
        state.borrow_mut().busy = true;
        spawn_local(async move {
            let mut device = state.borrow_mut().device.take();
            let result = match device.as_mut() {
                Some(device) => device.read(address, length).await,
                None => Err(wasm_bindgen::JsValue::from_str("No device connected")),
            };
            let mut shared = state.borrow_mut();
            shared.device = device;
            shared.busy = false;
            match result {
                Ok(data) => {
                    shared.read_data = Some(data);
                    shared.running = Some(true);
                    shared.status = "Download complete".to_owned();
                    shared.status_error = false;
                }
                Err(error) => {
                    set_error(&mut shared, format!("Download failed: {}", js_error(error)))
                }
            }
            repaint.request_repaint();
        });
    }

    fn set_hold(&self, enabled: bool, ctx: &egui::Context) {
        let state = self.state.clone();
        let repaint = ctx.clone();
        state.borrow_mut().busy = true;
        spawn_local(async move {
            let mut device = state.borrow_mut().device.take();
            let result = match device.as_mut() {
                Some(device) => device.set_hold(enabled).await,
                None => Err(wasm_bindgen::JsValue::from_str("No device connected")),
            };
            let mut shared = state.borrow_mut();
            shared.device = device;
            shared.busy = false;
            match result {
                Ok(()) => {
                    shared.hold_enabled = Some(enabled);
                    shared.status = if enabled {
                        "Target flash #HOLD asserted"
                    } else {
                        "Target flash #HOLD released"
                    }
                    .to_owned();
                    shared.status_error = false;
                }
                Err(error) => set_error(
                    &mut shared,
                    format!("Hold control failed: {}", js_error(error)),
                ),
            }
            repaint.request_repaint();
        });
    }

    fn toctou_command(&self, command: ToctouCommand, ctx: &egui::Context) {
        let state = self.state.clone();
        let repaint = ctx.clone();
        state.borrow_mut().busy = true;
        spawn_local(async move {
            let mut device = state.borrow_mut().device.take();
            let result = match (device.as_mut(), command) {
                (
                    Some(device),
                    ToctouCommand::Set {
                        index,
                        start,
                        mask,
                        replace,
                    },
                ) => device.toctou_set(index, start, mask, replace).await,
                (Some(device), ToctouCommand::Arm(index)) => device.toctou_arm(index).await,
                (Some(device), ToctouCommand::Disarm(index)) => device.toctou_disarm(index).await,
                (Some(device), ToctouCommand::Reset(index)) => device.toctou_reset(index).await,
                (Some(device), ToctouCommand::ResetAll) => device.toctou_reset_all().await,
                (None, _) => Err(wasm_bindgen::JsValue::from_str("No device connected")),
            };
            let mut shared = state.borrow_mut();
            shared.device = device;
            shared.busy = false;
            match result {
                Ok(()) => {
                    shared.status = "TOCTOU configuration updated".to_owned();
                    shared.status_error = false;
                }
                Err(error) => set_error(
                    &mut shared,
                    format!("TOCTOU command failed: {}", js_error(error)),
                ),
            }
            repaint.request_repaint();
        });
    }

    fn start_monitor(&self, ctx: &egui::Context) {
        let state = self.state.clone();
        let repaint = ctx.clone();
        {
            let mut shared = state.borrow_mut();
            shared.busy = true;
            shared.monitoring = true;
            shared.log_pending.clear();
            shared.log_accesses.clear();
            shared.log_txn_count = 0;
            shared.log_line_open = false;
            shared.status = "Starting SPI activity capture...".to_owned();
            shared.status_error = false;
        }
        spawn_local(async move {
            let Some(mut device) = state.borrow_mut().device.take() else {
                let mut shared = state.borrow_mut();
                shared.busy = false;
                shared.monitoring = false;
                set_error(&mut shared, "No device connected".to_owned());
                return;
            };
            if let Err(error) = device.log_start().await {
                let mut shared = state.borrow_mut();
                shared.device = Some(device);
                shared.busy = false;
                shared.monitoring = false;
                set_error(
                    &mut shared,
                    format!("Failed to start activity capture: {}", js_error(error)),
                );
                repaint.request_repaint();
                return;
            }
            state.borrow_mut().status = "SPI activity capture running".to_owned();

            while state.borrow().monitoring {
                match device.log_poll().await {
                    Ok(data) if !data.is_empty() => {
                        decode_log_packets(&mut state.borrow_mut(), &data)
                    }
                    Ok(_) => {}
                    Err(error) => {
                        let mut shared = state.borrow_mut();
                        set_error(
                            &mut shared,
                            format!("Activity capture failed: {}", js_error(error)),
                        );
                        shared.monitoring = false;
                    }
                }
                repaint.request_repaint();
                TimeoutFuture::new(10).await;
            }

            let stop_result = device.log_stop().await;
            let mut shared = state.borrow_mut();
            shared.device = Some(device);
            shared.busy = false;
            shared.monitoring = false;
            match stop_result {
                Ok(()) if !shared.status_error => {
                    shared.status = "SPI activity capture stopped".to_owned();
                }
                Ok(()) => {}
                Err(error) => set_error(
                    &mut shared,
                    format!("Failed to stop activity capture: {}", js_error(error)),
                ),
            }
            repaint.request_repaint();
        });
    }

    fn stop_monitor(&self) {
        let mut state = self.state.borrow_mut();
        state.monitoring = false;
        state.status = "Stopping SPI activity capture...".to_owned();
    }

    fn device_panel(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
        ui.heading("Device");
        ui.separator();

        let state = self.state.borrow();
        let connected = matches!(state.connection, ConnectionState::Connected);
        let connecting = matches!(state.connection, ConnectionState::Connecting);
        let busy = state.busy;
        let version = state.version;
        let running = state.running;
        let emulation_control = state.emulation_control;
        let configured_chip = state.configured_chip.clone();
        drop(state);

        ui.horizontal(|ui| {
            if ui
                .add_enabled(
                    !connected && !connecting,
                    egui::Button::new("Connect FT245 (WebUSB)"),
                )
                .clicked()
            {
                self.connect(true, ctx);
            }
            if ui
                .add_enabled(
                    !connected && !connecting,
                    egui::Button::new("Connect UART (Web Serial)"),
                )
                .clicked()
            {
                self.connect(false, ctx);
            }
            if ui
                .add_enabled(connected && !busy, egui::Button::new("Disconnect"))
                .clicked()
            {
                self.disconnect(ctx);
            }
            if connecting {
                ui.spinner();
            }
        });

        match &self.state.borrow().connection {
            ConnectionState::Disconnected => {
                ui.label("No device connected");
            }
            ConnectionState::Connecting => {
                ui.label("Requesting browser device access...");
            }
            ConnectionState::Connected => {
                ui.label(egui::RichText::new("Connected").color(Color32::GREEN));
            }
            ConnectionState::Error(error) => {
                ui.label(egui::RichText::new(error).color(Color32::RED));
            }
        }

        if connected {
            ui.add_space(16.0);
            ui.separator();
            ui.heading("Device Information");
            egui::Grid::new("device_info")
                .num_columns(2)
                .spacing([20.0, 4.0])
                .show(ui, |ui| {
                    ui.label("Protocol version:");
                    ui.label(version.map_or_else(|| "Unknown".to_owned(), |v| v.to_string()));
                    ui.end_row();
                    ui.label("Emulation:");
                    ui.label(match running {
                        Some(true) => "Running",
                        Some(false) => "Stopped",
                        None => "Unavailable",
                    });
                    ui.end_row();
                    ui.label("Emulated chip:");
                    ui.label(configured_chip.as_ref().map_or_else(
                        || "Not configured".to_owned(),
                        |chip| format!("{} {}", chip.vendor, chip.name),
                    ));
                    ui.end_row();
                });

            ui.add_space(16.0);
            ui.separator();
            ui.heading("Flash Chip");
            ui.label("Select and configure the flash chip NORbert should emulate.");

            let mut chip_to_configure = None;
            let popup_id = ui.make_persistent_id("chip_selector_popup");
            let selected_text = self.selected_chip.as_ref().map_or_else(
                || "Select chip...".to_owned(),
                |chip| {
                    format!(
                        "{} {} ({} MiB)",
                        chip.vendor,
                        chip.name,
                        chip.total_size / (1024 * 1024),
                    )
                },
            );
            let response = ui.add_enabled(
                !busy,
                egui::Button::new(format!("{selected_text} ▼")).min_size(egui::vec2(500.0, 0.0)),
            );
            if response.clicked() {
                ui.memory_mut(|memory| memory.toggle_popup(popup_id));
            }
            let search_id = ui.make_persistent_id("chip_search_field");
            egui::popup::popup_below_widget(
                ui,
                popup_id,
                &response,
                egui::popup::PopupCloseBehavior::CloseOnClickOutside,
                |ui| {
                    ui.set_min_width(500.0);
                    ui.add(
                        egui::TextEdit::singleline(&mut self.chip_search)
                            .id(search_id)
                            .hint_text("Search chips..."),
                    )
                    .request_focus();
                    ui.separator();
                    let search = self.chip_search.to_lowercase();
                    egui::ScrollArea::vertical()
                        .max_height(500.0)
                        .show(ui, |ui| {
                            for chip_info in &self.available_chips {
                                if (search.is_empty()
                                    || chip_info.display_name.to_lowercase().contains(&search))
                                    && ui
                                        .selectable_label(
                                            self.selected_chip.as_ref().is_some_and(|selected| {
                                                Rc::ptr_eq(selected, &chip_info.chip)
                                            }),
                                            &chip_info.display_name,
                                        )
                                        .clicked()
                                {
                                    chip_to_configure = Some(chip_info.chip.clone());
                                    ui.memory_mut(|memory| memory.close_popup());
                                }
                            }
                        });
                },
            );
            if let Some(chip) = chip_to_configure {
                self.configure_chip(chip, ctx);
            }

            if let Some(chip) = configured_chip.as_ref() {
                ui.label(format!(
                    "JEDEC {:02X} {:04X} · {} KiB · {}-byte addressing",
                    chip.jedec_manufacturer,
                    chip.jedec_device,
                    chip.total_size / 1024,
                    if chip.supports_4byte() { 4 } else { 3 },
                ));
            }

            ui.add_space(16.0);
            ui.separator();
            ui.heading("Control");
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(
                        emulation_control && running != Some(true) && !busy,
                        egui::Button::new("Start"),
                    )
                    .clicked()
                {
                    self.set_running(true, ctx);
                }
                if ui
                    .add_enabled(
                        emulation_control && running == Some(true) && !busy,
                        egui::Button::new("Stop"),
                    )
                    .clicked()
                {
                    self.set_running(false, ctx);
                }
                if ui
                    .add_enabled(emulation_control && !busy, egui::Button::new("Refresh"))
                    .clicked()
                {
                    self.refresh(ctx);
                }
            });
            ui.horizontal(|ui| {
                ui.label("Target flash #HOLD:");
                let hold_enabled = self.state.borrow().hold_enabled;
                if ui
                    .add_enabled(
                        !busy && hold_enabled != Some(true),
                        egui::Button::new("Assert"),
                    )
                    .clicked()
                {
                    self.set_hold(true, ctx);
                }
                if ui
                    .add_enabled(
                        !busy && hold_enabled != Some(false),
                        egui::Button::new("Release"),
                    )
                    .clicked()
                {
                    self.set_hold(false, ctx);
                }
            });

            ui.add_space(16.0);
            ui.separator();
            ui.heading("Memory");
            ui.label("Upload to Device:");
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(!busy, egui::Button::new("Select File..."))
                    .clicked()
                {
                    self.select_file();
                }
                if !self.upload_name.is_empty() {
                    ui.label(format!(
                        "{} ({} bytes)",
                        self.upload_name,
                        self.upload_data.as_ref().map_or(0, Vec::len)
                    ));
                }
            });
            ui.horizontal(|ui| {
                ui.label("Start Address:");
                ui.add(egui::TextEdit::singleline(&mut self.write_address).desired_width(120.0));
                let parsed = parse_number(&self.write_address);
                if ui
                    .add_enabled(
                        self.upload_data.is_some() && parsed.is_some() && !busy,
                        egui::Button::new("Upload"),
                    )
                    .clicked()
                {
                    self.write_memory(
                        parsed.unwrap(),
                        self.upload_data.clone().unwrap(),
                        self.verify_upload,
                        ctx,
                    );
                }
            });
            ui.checkbox(&mut self.verify_upload, "Verify after upload");

            ui.add_space(8.0);
            ui.separator();
            ui.label("Download from Device:");
            ui.horizontal(|ui| {
                ui.label("Address:");
                ui.add(egui::TextEdit::singleline(&mut self.read_address).desired_width(120.0));
                ui.label("Length:");
                ui.add(egui::TextEdit::singleline(&mut self.read_length).desired_width(120.0));
                let address = parse_number(&self.read_address);
                let length = parse_number(&self.read_length);
                if ui
                    .add_enabled(
                        address.is_some() && length.is_some() && !busy,
                        egui::Button::new("Download"),
                    )
                    .clicked()
                {
                    self.read_memory(address.unwrap(), length.unwrap(), ctx);
                }
            });
            if let Some(data) = self.state.borrow().read_data.as_ref() {
                ui.horizontal(|ui| {
                    ui.label(format!("{} bytes ready", data.len()));
                    if ui.button("Save As...").clicked() {
                        save_file(data, "norbert-dump.bin");
                    }
                });
                egui::ScrollArea::vertical()
                    .max_height(180.0)
                    .show(ui, |ui| {
                        ui.code(hex_dump(&data[..data.len().min(4096)], 0));
                    });
            }
        }
    }

    fn toctou_panel(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
        ui.heading("TOCTOU Traps");
        ui.separator();
        let state = self.state.borrow();
        let connected = matches!(state.connection, ConnectionState::Connected);
        let busy = state.busy;
        drop(state);
        if !connected {
            ui.label("Connect to a device first.");
            return;
        }

        ui.label("Configure one of four address-match traps. The first matching read passes through; subsequent reads are redirected.");
        egui::Grid::new("toctou_fields")
            .num_columns(2)
            .spacing([20.0, 8.0])
            .show(ui, |ui| {
                ui.label("Trap index (0-3):");
                ui.text_edit_singleline(&mut self.toctou_index);
                ui.end_row();
                ui.label("Start address:");
                ui.text_edit_singleline(&mut self.toctou_start);
                ui.end_row();
                ui.label("Match mask:");
                ui.text_edit_singleline(&mut self.toctou_mask);
                ui.end_row();
                ui.label("Replacement base:");
                ui.text_edit_singleline(&mut self.toctou_replace);
                ui.end_row();
            });

        let index = parse_number(&self.toctou_index)
            .and_then(|value| u8::try_from(value).ok())
            .filter(|value| *value < 4);
        let start = parse_number(&self.toctou_start);
        let mask = parse_number(&self.toctou_mask);
        let replace = parse_number(&self.toctou_replace);
        ui.horizontal(|ui| {
            if ui
                .add_enabled(
                    index.is_some()
                        && start.is_some()
                        && mask.is_some()
                        && replace.is_some()
                        && !busy,
                    egui::Button::new("Set"),
                )
                .clicked()
            {
                self.toctou_command(
                    ToctouCommand::Set {
                        index: index.unwrap(),
                        start: start.unwrap(),
                        mask: mask.unwrap(),
                        replace: replace.unwrap(),
                    },
                    ctx,
                );
            }
            if ui
                .add_enabled(index.is_some() && !busy, egui::Button::new("Arm"))
                .clicked()
            {
                self.toctou_command(ToctouCommand::Arm(index.unwrap()), ctx);
            }
            if ui
                .add_enabled(index.is_some() && !busy, egui::Button::new("Disarm"))
                .clicked()
            {
                self.toctou_command(ToctouCommand::Disarm(index.unwrap()), ctx);
            }
            if ui
                .add_enabled(index.is_some() && !busy, egui::Button::new("Reset trigger"))
                .clicked()
            {
                self.toctou_command(ToctouCommand::Reset(index.unwrap()), ctx);
            }
            if ui
                .add_enabled(!busy, egui::Button::new("Reset all"))
                .clicked()
            {
                self.toctou_command(ToctouCommand::ResetAll, ctx);
            }
        });
        if index.is_none() {
            ui.label(
                egui::RichText::new("Trap index must be between 0 and 3.").color(Color32::RED),
            );
        }
    }

    fn activity_panel(&self, ui: &mut egui::Ui, ctx: &egui::Context) {
        ui.heading("SPI Activity");
        ui.separator();
        let state = self.state.borrow();
        let connected = matches!(state.connection, ConnectionState::Connected);
        let supported = state.activity_log;
        let monitoring = state.monitoring;
        let busy = state.busy;
        drop(state);
        if !connected {
            ui.label("Connect to a device first.");
            return;
        }
        ui.horizontal(|ui| {
            if ui
                .add_enabled(supported && !busy, egui::Button::new("Start Capture"))
                .clicked()
            {
                self.start_monitor(ctx);
            }
            if ui
                .add_enabled(monitoring, egui::Button::new("Stop Capture"))
                .clicked()
            {
                self.stop_monitor();
            }
            if ui.button("Clear").clicked() {
                self.state.borrow_mut().log_output.clear();
            }
        });
        egui::ScrollArea::vertical()
            .stick_to_bottom(true)
            .show(ui, |ui| {
                let mut state = self.state.borrow_mut();
                ui.add(
                    egui::TextEdit::multiline(&mut state.log_output)
                        .font(egui::TextStyle::Monospace)
                        .desired_width(f32::INFINITY)
                        .interactive(false),
                );
            });
    }
}

#[derive(Clone, Copy)]
enum ToctouCommand {
    Set {
        index: u8,
        start: u32,
        mask: u32,
        replace: u32,
    },
    Arm(u8),
    Disarm(u8),
    Reset(u8),
    ResetAll,
}

impl eframe::App for NorbertWebApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.selected_chip = self.state.borrow().configured_chip.clone();

        if let Some((name, data)) = self.state.borrow_mut().pending_file.take() {
            self.upload_name = name;
            self.upload_data = Some(data);
        }

        egui::TopBottomPanel::top("top_panel").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.heading("NORbert Web Interface");
                ui.separator();
                ui.selectable_value(&mut self.panel, Panel::Device, "Device");
                ui.selectable_value(&mut self.panel, Panel::Activity, "Activity");
                ui.selectable_value(&mut self.panel, Panel::Toctou, "TOCTOU");
            });
        });
        egui::TopBottomPanel::bottom("bottom_panel").show(ctx, |ui| {
            let state = self.state.borrow();
            let color = if state.status_error {
                Color32::RED
            } else {
                Color32::GREEN
            };
            ui.label(egui::RichText::new(&state.status).color(color));
        });
        egui::CentralPanel::default().show(ctx, |ui| match self.panel {
            Panel::Device => self.device_panel(ui, ctx),
            Panel::Activity => self.activity_panel(ui, ctx),
            Panel::Toctou => self.toctou_panel(ui, ctx),
        });
        if self.state.borrow().busy
            || matches!(self.state.borrow().connection, ConnectionState::Connecting)
        {
            ctx.request_repaint_after(std::time::Duration::from_millis(50));
        }
    }
}

fn decode_log_packets(state: &mut SharedState, data: &[u8]) {
    const LOG_CMD: u8 = 0xA1;
    const LOG_ADDR: u8 = 0xA2;
    const LOG_END: u8 = 0xA3;
    const LOG_TRAP: u8 = 0xA4;

    state.log_pending.extend_from_slice(data);
    let mut position = 0;
    while position < state.log_pending.len() {
        let remaining = state.log_pending.len() - position;
        match state.log_pending[position] {
            LOG_CMD if remaining >= 2 => {
                if state.log_line_open {
                    state.log_output.push('\n');
                }
                state.log_opcode = state.log_pending[position + 1];
                state.log_txn_count += 1;
                state.log_output.push_str(&format!(
                    "{:<6} 0x{:02X} {:<18}",
                    state.log_txn_count,
                    state.log_opcode,
                    spi_opcode_name(state.log_opcode),
                ));
                state.log_line_open = true;
                position += 2;
            }
            LOG_ADDR if remaining >= 5 => {
                let address = u32::from_be_bytes([
                    state.log_pending[position + 1],
                    state.log_pending[position + 2],
                    state.log_pending[position + 3],
                    state.log_pending[position + 4],
                ]);
                state.log_address = address;
                state.log_output.push_str(&format!(" 0x{address:08X}"));
                if is_read_opcode(state.log_opcode) {
                    let count = state
                        .log_accesses
                        .entry((address, state.log_opcode))
                        .or_insert(0);
                    *count += 1;
                    if *count == 2 {
                        state
                            .log_output
                            .push_str("  ** DOUBLE READ (TOCTOU candidate)");
                    } else if *count > 2 {
                        state.log_output.push_str(&format!("  ** READ #{count}"));
                    }
                }
                state.log_output.push('\n');
                state.log_line_open = false;
                position += 5;
            }
            LOG_END if remaining >= 4 => {
                let count = u32::from_be_bytes([
                    0,
                    state.log_pending[position + 1],
                    state.log_pending[position + 2],
                    state.log_pending[position + 3],
                ]);
                if state.log_line_open {
                    state.log_output.push('\n');
                    state.log_line_open = false;
                }
                if count > 1 {
                    state.log_output.push_str(&format!(
                        "       end: {count} bytes from 0x{:08X}\n",
                        state.log_address,
                    ));
                }
                position += 4;
            }
            LOG_TRAP if remaining >= 6 => {
                let index = state.log_pending[position + 1];
                let address = u32::from_be_bytes([
                    0,
                    state.log_pending[position + 2],
                    state.log_pending[position + 3],
                    state.log_pending[position + 4],
                ]);
                state.log_output.push_str(&format!(
                    "!! TOCTOU TRAP #{index} FIRED at 0x{address:06X}\n",
                ));
                position += 6;
            }
            0xA1..=0xAF => break,
            _ => position += 1,
        }
    }
    state.log_pending.drain(..position);
    const MAX_LOG: usize = 2 * 1024 * 1024;
    if state.log_output.len() > MAX_LOG {
        let excess = state.log_output.len() - MAX_LOG;
        let boundary = state.log_output[excess..]
            .find('\n')
            .map_or(excess, |offset| excess + offset + 1);
        state.log_output.drain(..boundary);
    }
}

fn spi_opcode_name(opcode: u8) -> &'static str {
    match opcode {
        0x01 => "WRITE_STATUS",
        0x02 => "PAGE_PROGRAM",
        0x03 => "READ",
        0x04 => "WRITE_DISABLE",
        0x05 => "READ_STATUS",
        0x06 => "WRITE_ENABLE",
        0x0B => "FAST_READ",
        0x0C => "FAST_READ_4B",
        0x12 => "PAGE_PROGRAM_4B",
        0x13 => "READ_4B",
        0x20 => "SECTOR_ERASE_4K",
        0x21 => "SECTOR_ERASE_4K_4B",
        0x35 => "READ_STATUS2",
        0x3B => "DUAL_READ",
        0x3C => "DUAL_READ_4B",
        0x52 => "BLOCK_ERASE_32K",
        0x5A => "READ_SFDP",
        0x60 | 0xC7 => "CHIP_ERASE",
        0x6B => "QUAD_READ",
        0x6C => "QUAD_READ_4B",
        0x9E | 0x9F => "READ_JEDEC_ID",
        0xB7 => "4BYTE_ENABLE",
        0xBB => "DUAL_IO_READ",
        0xBC => "DUAL_IO_READ_4B",
        0xD8 => "BLOCK_ERASE_64K",
        0xDC => "BLOCK_ERASE_64K_4B",
        0xE9 => "4BYTE_DISABLE",
        0xEB => "QUAD_IO_READ",
        0xEC => "QUAD_IO_READ_4B",
        _ => "UNKNOWN",
    }
}

fn is_read_opcode(opcode: u8) -> bool {
    matches!(
        opcode,
        0x03 | 0x0B | 0x0C | 0x13 | 0x3B | 0x3C | 0x5A | 0x6B | 0x6C | 0xBB | 0xBC | 0xEB | 0xEC
    )
}

fn set_error(state: &mut SharedState, message: String) {
    state.status = message;
    state.status_error = true;
}

fn js_error(error: wasm_bindgen::JsValue) -> String {
    error.as_string().unwrap_or_else(|| format!("{error:?}"))
}

fn parse_number(value: &str) -> Option<u32> {
    let value = value.trim();
    value
        .strip_prefix("0x")
        .or_else(|| value.strip_prefix("0X"))
        .map_or_else(
            || value.parse().ok(),
            |hex| u32::from_str_radix(hex, 16).ok(),
        )
}

fn hex_dump(data: &[u8], base: u32) -> String {
    data.chunks(16)
        .enumerate()
        .map(|(line, chunk)| {
            let hex = chunk
                .iter()
                .map(|byte| format!("{byte:02x}"))
                .collect::<Vec<_>>()
                .join(" ");
            let ascii: String = chunk
                .iter()
                .map(|byte| {
                    if (32..=126).contains(byte) {
                        char::from(*byte)
                    } else {
                        '.'
                    }
                })
                .collect();
            format!("{:08x}: {:47} |{}|", base + (line * 16) as u32, hex, ascii)
        })
        .collect::<Vec<_>>()
        .join("\n")
}

fn save_file(data: &[u8], filename: &str) {
    let bytes = Uint8Array::from(data);
    let parts = Array::new();
    parts.push(&bytes.buffer());
    let options = BlobPropertyBag::new();
    options.set_type("application/octet-stream");
    let blob = Blob::new_with_u8_array_sequence_and_options(&parts, &options).unwrap();
    let url = Url::create_object_url_with_blob(&blob).unwrap();
    let document = web_sys::window().unwrap().document().unwrap();
    let link: HtmlAnchorElement = document.create_element("a").unwrap().dyn_into().unwrap();
    link.set_href(&url);
    link.set_download(filename);
    link.click();
    Url::revoke_object_url(&url).unwrap();
}

fn main() {
    console_error_panic_hook::set_once();
    tracing_wasm::set_as_global_default();
    let options = eframe::WebOptions::default();
    spawn_local(async move {
        let canvas = web_sys::window()
            .unwrap()
            .document()
            .unwrap()
            .get_element_by_id("norbert_canvas")
            .unwrap()
            .dyn_into::<HtmlCanvasElement>()
            .unwrap();
        eframe::WebRunner::new()
            .start(
                canvas,
                options,
                Box::new(|cc| Ok(Box::new(NorbertWebApp::new(cc)))),
            )
            .await
            .expect("failed to start NORbert web UI");
    });
}
