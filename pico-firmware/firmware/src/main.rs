#![no_std]
#![no_main]

mod bridge;
mod ft245_bus;

use bridge::Bridge;
use defmt::{info, unwrap, warn};
use embassy_executor::Spawner;
#[cfg(feature = "ethernet")]
use embassy_futures::yield_now;
#[cfg(feature = "ethernet")]
use embassy_net::tcp::TcpSocket;
#[cfg(feature = "ethernet")]
use embassy_net::{Stack, StackResources};
#[cfg(feature = "ethernet")]
use embassy_net_wiznet::chip::W5500;
#[cfg(feature = "ethernet")]
use embassy_net_wiznet::{Device, Runner, State};
#[cfg(feature = "ethernet")]
use embassy_rp::clocks::RoscRng;
#[cfg(feature = "ethernet")]
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, DMA_CH2, DMA_CH3, PIO0, USB};
#[cfg(feature = "ethernet")]
use embassy_rp::peripherals::{PIN_16, PIN_17, PIN_18, PIN_19, PIN_20, PIN_21, SPI0};
use embassy_rp::pio::InterruptHandler as PioInterruptHandler;
#[cfg(feature = "ethernet")]
use embassy_rp::spi::{Async, Config as SpiConfig, Spi};
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_rp::{bind_interrupts, dma};
#[cfg(feature = "ethernet")]
use embassy_time::{Delay, Duration};
use embassy_usb::driver::{Endpoint, EndpointIn, EndpointOut};
use embassy_usb::msos::{self, windows_version};
use embassy_usb::{Builder, Config as UsbConfig, UsbDevice};
#[cfg(feature = "ethernet")]
use embedded_hal_bus::spi::ExclusiveDevice;
#[cfg(feature = "ethernet")]
use embedded_io_async::Write;
use ft245_bus::Ft245Bus;
#[cfg(feature = "ethernet")]
use norbert_pico_protocol::TCP_PORT;
use norbert_pico_protocol::{
    DeviceFrame, ErrorCode, MAX_FRAME_BODY, Response, decode_host_body, encode_device_frame,
    frame_len,
};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>, dma::InterruptHandler<DMA_CH2>, dma::InterruptHandler<DMA_CH3>;
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});

const USB_VID: u16 = 0x1209;
const USB_PID: u16 = 0x4e42;
static DEVICE_INTERFACE_GUIDS: [&str; 1] = ["{9d32d44e-5f3d-4ef1-9213-4c574f4e4242}"];

#[embassy_executor::task]
async fn usb_task(mut usb: UsbDevice<'static, Driver<'static, USB>>) -> ! {
    usb.run().await
}

#[cfg(feature = "ethernet")]
#[embassy_executor::task]
async fn ethernet_task(
    runner: Runner<
        'static,
        W5500,
        ExclusiveDevice<Spi<'static, SPI0, Async>, Output<'static>, Delay>,
        Input<'static>,
        Output<'static>,
    >,
) -> ! {
    runner.run().await
}

#[cfg(feature = "ethernet")]
#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, Device<'static>>) -> ! {
    runner.run().await
}

#[cfg(feature = "ethernet")]
#[embassy_executor::task]
#[allow(clippy::too_many_arguments)]
async fn ethernet_init_task(
    spawner: Spawner,
    bridge: &'static Bridge<'static>,
    spi0: embassy_rp::Peri<'static, SPI0>,
    dma0: embassy_rp::Peri<'static, DMA_CH0>,
    dma1: embassy_rp::Peri<'static, DMA_CH1>,
    miso: embassy_rp::Peri<'static, PIN_16>,
    cs: embassy_rp::Peri<'static, PIN_17>,
    clk: embassy_rp::Peri<'static, PIN_18>,
    mosi: embassy_rp::Peri<'static, PIN_19>,
    reset: embassy_rp::Peri<'static, PIN_20>,
    int: embassy_rp::Peri<'static, PIN_21>,
    seed: u64,
) -> ! {
    match init_w5500_stack(
        &spawner, spi0, dma0, dma1, miso, cs, clk, mosi, reset, int, seed,
    )
    .await
    {
        Some(stack) => tcp_rpc_server(stack, bridge).await,
        None => loop {
            warn!("W5500 unavailable; USB RPC remains active");
            embassy_time::Timer::after(Duration::from_secs(5)).await;
        },
    }
}

#[cfg(feature = "ethernet")]
async fn tcp_rpc_server(stack: Stack<'static>, bridge: &'static Bridge<'static>) -> ! {
    let mut rx_buffer = [0u8; 4096];
    let mut tx_buffer = [0u8; 4096];
    let mut body = [0u8; MAX_FRAME_BODY];
    let mut out = [0u8; norbert_pico_protocol::MAX_FRAME];

    loop {
        let mut socket = TcpSocket::new(stack, rx_buffer.as_mut_slice(), tx_buffer.as_mut_slice());
        socket.set_timeout(Some(Duration::from_secs(30)));

        info!("Listening for NORbert RPC on TCP:{}", TCP_PORT);
        if let Err(e) = socket.accept(TCP_PORT).await {
            warn!("TCP accept failed: {:?}", e);
            continue;
        }
        info!("TCP RPC client connected: {:?}", socket.remote_endpoint());

        loop {
            let len = match read_tcp_frame_body(&mut socket, body.as_mut_slice()).await {
                Ok(0) => break,
                Ok(len) => len,
                Err(error) => {
                    warn!("TCP frame read failed: {:?}", error);
                    break;
                }
            };

            let Some(body) = body.get(..len) else {
                break;
            };
            let n = handle_rpc_body(bridge, body, out.as_mut_slice()).await;
            let Some(response) = out.get(..n) else {
                break;
            };
            if let Err(e) = socket.write_all(response).await {
                warn!("TCP response write failed: {:?}", e);
                break;
            }
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("NORbert Pico bridge starting");
    let p = embassy_rp::init(Default::default());
    #[cfg(feature = "ethernet")]
    let mut rng = RoscRng;

    let ft245 = Ft245Bus::new(
        p.PIN_0, p.PIN_1, p.PIN_2, p.PIN_3, p.PIN_4, p.PIN_5, p.PIN_6, p.PIN_7, p.PIN_8, p.PIN_9,
        p.PIN_10, p.PIN_11, p.PIO0, p.DMA_CH2, p.DMA_CH3,
    );
    static BRIDGE: StaticCell<Bridge<'static>> = StaticCell::new();
    let bridge = BRIDGE.init(Bridge::new(ft245));

    #[cfg(feature = "ethernet")]
    spawner.spawn(unwrap!(ethernet_init_task(
        spawner,
        bridge,
        p.SPI0,
        p.DMA_CH0,
        p.DMA_CH1,
        p.PIN_16,
        p.PIN_17,
        p.PIN_18,
        p.PIN_19,
        p.PIN_20,
        p.PIN_21,
        rng.next_u64(),
    )));

    let driver = Driver::new(p.USB, Irqs);
    let mut usb_config = UsbConfig::new(USB_VID, USB_PID);
    usb_config.manufacturer = Some("9elements");
    usb_config.product = Some("NORbert Pico Bridge");
    usb_config.serial_number = Some("NORBERT-PICO-0001");
    usb_config.max_power = 100;
    usb_config.max_packet_size_0 = 64;

    static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

    let config_descriptor = CONFIG_DESCRIPTOR.init([0u8; 256]);
    let bos_descriptor = BOS_DESCRIPTOR.init([0u8; 256]);
    let msos_descriptor = MSOS_DESCRIPTOR.init([0u8; 256]);
    let control_buf = CONTROL_BUF.init([0u8; 64]);
    let mut builder = Builder::new(
        driver,
        usb_config,
        config_descriptor.as_mut_slice(),
        bos_descriptor.as_mut_slice(),
        msos_descriptor.as_mut_slice(),
        control_buf.as_mut_slice(),
    );
    builder.msos_descriptor(windows_version::WIN8_1, 0);
    builder.msos_feature(msos::CompatibleIdFeatureDescriptor::new("WINUSB", ""));
    builder.msos_feature(msos::RegistryPropertyFeatureDescriptor::new(
        "DeviceInterfaceGUIDs",
        msos::PropertyData::RegMultiSz(DEVICE_INTERFACE_GUIDS.as_slice()),
    ));

    let mut function = builder.function(0xFF, 0, 0);
    let mut interface = function.interface();
    let mut alt = interface.alt_setting(0xFF, 0, 0, None);
    let mut usb_out = alt.endpoint_bulk_out(None, 64);
    let mut usb_in = alt.endpoint_bulk_in(None, 64);
    drop(function);

    let usb = builder.build();
    spawner.spawn(unwrap!(usb_task(usb)));

    static USB_RX: StaticCell<UsbFrameRx> = StaticCell::new();
    static USB_OUT: StaticCell<[u8; norbert_pico_protocol::MAX_FRAME]> = StaticCell::new();
    let rx = USB_RX.init(UsbFrameRx::new());
    let out = USB_OUT.init([0u8; norbert_pico_protocol::MAX_FRAME]);
    loop {
        usb_out.wait_enabled().await;
        info!("USB RPC connected");
        rx.clear();

        loop {
            let mut packet = [0u8; 64];
            let n = match usb_out.read(packet.as_mut_slice()).await {
                Ok(n) => n,
                Err(e) => {
                    warn!("USB OUT failed: {:?}", e);
                    break;
                }
            };

            let Some(packet) = packet.get(..n) else {
                break;
            };
            if let Err(error) = rx.push(packet) {
                let len = encode_error(0, error, out.as_mut_slice());
                if let Some(response) = out.get(..len) {
                    let _ = write_usb_chunks(&mut usb_in, response).await;
                }
                rx.clear();
                continue;
            }

            loop {
                let frame = match rx.frame() {
                    Ok(frame) => frame,
                    Err(error) => {
                        let len = encode_error(0, error, out.as_mut_slice());
                        if let Some(response) = out.get(..len) {
                            let _ = write_usb_chunks(&mut usb_in, response).await;
                        }
                        rx.clear();
                        break;
                    }
                };
                let Some(body) = frame else {
                    break;
                };
                let len = handle_rpc_body(bridge, body, out.as_mut_slice()).await;
                let Some(response) = out.get(..len) else {
                    break;
                };
                if let Err(e) = write_usb_chunks(&mut usb_in, response).await {
                    warn!("USB IN failed: {:?}", e);
                    break;
                }
                rx.consume();
            }
        }
    }
}

#[cfg(feature = "ethernet")]
#[allow(clippy::too_many_arguments)]
async fn init_w5500_stack(
    spawner: &Spawner,
    spi0: embassy_rp::Peri<'static, SPI0>,
    dma0: embassy_rp::Peri<'static, DMA_CH0>,
    dma1: embassy_rp::Peri<'static, DMA_CH1>,
    miso: embassy_rp::Peri<'static, PIN_16>,
    cs: embassy_rp::Peri<'static, PIN_17>,
    clk: embassy_rp::Peri<'static, PIN_18>,
    mosi: embassy_rp::Peri<'static, PIN_19>,
    reset: embassy_rp::Peri<'static, PIN_20>,
    int: embassy_rp::Peri<'static, PIN_21>,
    seed: u64,
) -> Option<Stack<'static>> {
    let mut spi_cfg = SpiConfig::default();
    spi_cfg.frequency = 50_000_000;
    let spi = Spi::new(spi0, clk, mosi, miso, dma0, dma1, Irqs, spi_cfg);
    let cs = Output::new(cs, Level::High);
    let w5500_int = Input::new(int, Pull::Up);
    let w5500_reset = Output::new(reset, Level::High);

    static WIZ_STATE: StaticCell<State<8, 8>> = StaticCell::new();
    let state = WIZ_STATE.init(State::<8, 8>::new());
    let mac_addr = [0x02, 0x4e, 0x4f, 0x52, 0x42, 0x54];
    let spi_device = match ExclusiveDevice::new(spi, cs, Delay) {
        Ok(device) => device,
        Err(_) => {
            warn!("Failed to create W5500 SPI device");
            return None;
        }
    };

    let (device, runner) =
        match embassy_net_wiznet::new(mac_addr, state, spi_device, w5500_int, w5500_reset).await {
            Ok(result) => result,
            Err(_) => {
                warn!("W5500 init failed");
                return None;
            }
        };
    spawner.spawn(unwrap!(ethernet_task(runner)));

    static RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        device,
        embassy_net::Config::dhcpv4(Default::default()),
        RESOURCES.init(StackResources::new()),
        seed,
    );
    spawner.spawn(unwrap!(net_task(runner)));

    info!("Waiting for DHCP");
    let cfg = wait_for_config(stack).await;
    info!("W5500 IPv4 address: {:?}", cfg.address.address());
    Some(stack)
}

#[cfg(feature = "ethernet")]
async fn wait_for_config(stack: Stack<'static>) -> embassy_net::StaticConfigV4 {
    loop {
        if let Some(config) = stack.config_v4() {
            return config.clone();
        }
        yield_now().await;
    }
}

async fn handle_rpc_body(bridge: &'static Bridge<'static>, body: &[u8], out: &mut [u8]) -> usize {
    let response = match decode_host_body(body) {
        Ok(frame) => bridge.handle(frame).await,
        Err(error) => DeviceFrame {
            seq: 0,
            response: Response::Error(error),
        },
    };

    match encode_device_frame(&response, out) {
        Ok(len) => len,
        Err(_) => encode_error(response.seq, ErrorCode::EncodeFailed, out),
    }
}

fn encode_error(seq: u32, error: ErrorCode, out: &mut [u8]) -> usize {
    let frame = DeviceFrame {
        seq,
        response: Response::Error(error),
    };
    encode_device_frame(&frame, out).unwrap_or(0)
}

#[cfg(feature = "ethernet")]
async fn read_tcp_frame_body(
    socket: &mut TcpSocket<'_>,
    body: &mut [u8],
) -> Result<usize, ErrorCode> {
    let mut prefix = [0u8; 2];
    if read_exact_tcp(socket, &mut prefix).await? == 0 {
        return Ok(0);
    }
    let len = frame_len(prefix);
    if len > body.len() {
        return Err(ErrorCode::FrameTooLarge);
    }
    let Some(body) = body.get_mut(..len) else {
        return Err(ErrorCode::FrameTooLarge);
    };
    read_exact_tcp(socket, body).await?;
    Ok(len)
}

#[cfg(feature = "ethernet")]
async fn read_exact_tcp(
    socket: &mut TcpSocket<'_>,
    mut out: &mut [u8],
) -> Result<usize, ErrorCode> {
    let original_len = out.len();
    while !out.is_empty() {
        match socket.read(out).await {
            Ok(0) if original_len == out.len() => return Ok(0),
            Ok(0) => return Err(ErrorCode::FpgaTimeout),
            Ok(n) => {
                let Some(rest) = out.get_mut(n..) else {
                    return Err(ErrorCode::FpgaTimeout);
                };
                out = rest;
            }
            Err(_) => return Err(ErrorCode::FpgaTimeout),
        }
    }
    Ok(original_len)
}

async fn write_usb_chunks<E: EndpointIn>(
    ep: &mut E,
    mut data: &[u8],
) -> Result<(), embassy_usb::driver::EndpointError> {
    while !data.is_empty() {
        let n = core::cmp::min(64, data.len());
        let Some(chunk) = data.get(..n) else {
            break;
        };
        ep.write(chunk).await?;
        let Some(rest) = data.get(n..) else {
            break;
        };
        data = rest;
    }
    Ok(())
}

struct UsbFrameRx {
    buf: [u8; norbert_pico_protocol::MAX_FRAME],
    used: usize,
    expected: Option<usize>,
}

impl UsbFrameRx {
    fn new() -> Self {
        Self {
            buf: [0u8; norbert_pico_protocol::MAX_FRAME],
            used: 0,
            expected: None,
        }
    }

    fn clear(&mut self) {
        self.used = 0;
        self.expected = None;
    }

    fn push(&mut self, data: &[u8]) -> Result<(), ErrorCode> {
        if self.used + data.len() > self.buf.len() {
            return Err(ErrorCode::FrameTooLarge);
        }
        let end = self.used + data.len();
        let Some(dst) = self.buf.get_mut(self.used..end) else {
            return Err(ErrorCode::FrameTooLarge);
        };
        dst.copy_from_slice(data);
        self.used = end;
        self.update_expected()
    }

    fn frame(&mut self) -> Result<Option<&[u8]>, ErrorCode> {
        self.update_expected()?;
        match self.expected {
            Some(expected) if self.used >= expected => Ok(self.buf.get(2..expected)),
            _ => Ok(None),
        }
    }

    fn consume(&mut self) {
        let Some(expected) = self.expected else {
            self.clear();
            return;
        };
        if self.used < expected {
            self.clear();
            return;
        }
        self.buf.copy_within(expected..self.used, 0);
        self.used -= expected;
        self.expected = None;
    }

    fn update_expected(&mut self) -> Result<(), ErrorCode> {
        if self.expected.is_none() && self.used >= 2 {
            let Some(prefix) = self.buf.get(..2) else {
                return Err(ErrorCode::FrameTooLarge);
            };
            let body_len = match prefix {
                [lo, hi] => frame_len([*lo, *hi]),
                _ => return Err(ErrorCode::FrameTooLarge),
            };
            if body_len > MAX_FRAME_BODY {
                return Err(ErrorCode::FrameTooLarge);
            }
            self.expected = Some(body_len + 2);
        }
        Ok(())
    }
}
