use embassy_rp::Peri;
use embassy_rp::dma;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH2, DMA_CH3, PIO0};
use embassy_rp::pio::{
    self, Config, Direction, FifoJoin, Pio, PioPin, ShiftConfig, ShiftDirection, StateMachine,
};
use embassy_time::{Duration, Timer, with_timeout};
use norbert_pico_protocol::{ErrorCode, FPGA_PROTOCOL_VERSION};

const STROBE_TIMEOUT_MS: u64 = 100;
const RXF_IDLE: u8 = 1;
const RXF_BUSY: u8 = 0;

/// Timing-compatible FT245-device-side bus for the FPGA connector.
///
/// The data strobes are implemented by RP2040 PIO state machines and use DMA
/// for bulk byte movement between SRAM and the PIO FIFOs.  The public API stays
/// byte-oriented because the higher-level FPGA command protocol is
/// request/response and needs bounded timeout/error handling around every bus
/// transaction.
pub struct Ft245Bus<'d> {
    tx_sm: StateMachine<'d, PIO0, 0>,
    rx_sm: StateMachine<'d, PIO0, 1>,
    tx_dma: dma::Channel<'d>,
    rx_dma: dma::Channel<'d>,
    data: [pio::Pin<'d, PIO0>; 8],
    txe_n: Output<'d>,
    stats: BusStats,
}

/// Low-level FIFO-bus counters.
#[derive(Clone, Copy, Debug, Default)]
pub struct BusStats {
    /// Bytes driven from the Pico to the FPGA.
    pub tx_bytes: u32,
    /// Bytes sampled from the FPGA to the Pico.
    pub rx_bytes: u32,
}

impl<'d> Ft245Bus<'d> {
    /// Create an FT245-compatible device-side bus.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        d0: Peri<'d, impl PioPin>,
        d1: Peri<'d, impl PioPin>,
        d2: Peri<'d, impl PioPin>,
        d3: Peri<'d, impl PioPin>,
        d4: Peri<'d, impl PioPin>,
        d5: Peri<'d, impl PioPin>,
        d6: Peri<'d, impl PioPin>,
        d7: Peri<'d, impl PioPin>,
        rxf_n: Peri<'d, impl PioPin>,
        txe_n: Peri<'d, impl embassy_rp::gpio::Pin>,
        rd_n: Peri<'d, impl PioPin>,
        wr_n: Peri<'d, impl PioPin>,
        pio0: Peri<'d, PIO0>,
        tx_dma: Peri<'d, DMA_CH2>,
        rx_dma: Peri<'d, DMA_CH3>,
    ) -> Self {
        let mut pio = Pio::new(pio0, crate::Irqs);
        let tx_asm = pio::program::pio_asm!(
            r#"
                .wrap_target
                    pull block
                    set pins, 1
                    out pins, 8
                    set pins, 0
                    wait 0 gpio 10
                    wait 1 gpio 10
                    set pins, 1 [4]
                .wrap
            "#,
        );
        let rx_asm = pio::program::pio_asm!(
            r#"
                .wrap_target
                    wait 0 gpio 11
                    nop [7]
                    in pins, 8
                    wait 1 gpio 11
                    push block
                .wrap
            "#,
        );
        let tx_program = pio.common.load_program(&tx_asm.program);
        let rx_program = pio.common.load_program(&rx_asm.program);

        let d0 = pio.common.make_pio_pin(d0);
        let d1 = pio.common.make_pio_pin(d1);
        let d2 = pio.common.make_pio_pin(d2);
        let d3 = pio.common.make_pio_pin(d3);
        let d4 = pio.common.make_pio_pin(d4);
        let d5 = pio.common.make_pio_pin(d5);
        let d6 = pio.common.make_pio_pin(d6);
        let d7 = pio.common.make_pio_pin(d7);
        let rxf_n = pio.common.make_pio_pin(rxf_n);
        let txe_n = Output::new(txe_n, Level::High);
        let mut rd_n = pio.common.make_pio_pin(rd_n);
        let mut wr_n = pio.common.make_pio_pin(wr_n);
        rd_n.set_pull(embassy_rp::gpio::Pull::Up);
        wr_n.set_pull(embassy_rp::gpio::Pull::Up);

        let data = [d0, d1, d2, d3, d4, d5, d6, d7];
        let control = [rxf_n];
        let data_refs = data.each_ref();
        let control_refs = control.each_ref();

        let mut tx_sm = pio.sm0;
        configure_tx_sm(
            &mut tx_sm,
            &tx_program,
            data_refs.as_slice(),
            control_refs.as_slice(),
        );
        let mut rx_sm = pio.sm1;
        configure_rx_sm(&mut rx_sm, &rx_program, data_refs.as_slice());

        let mut bus = Self {
            tx_sm,
            rx_sm,
            tx_dma: dma::Channel::new(tx_dma, crate::Irqs),
            rx_dma: dma::Channel::new(rx_dma, crate::Irqs),
            data,
            txe_n,
            stats: BusStats::default(),
        };
        bus.idle_bus();
        bus
    }

    /// Return low-level bus counters.
    pub fn stats(&self) -> BusStats {
        self.stats
    }

    /// Write all bytes to the FPGA byte protocol.
    pub async fn write_all(&mut self, data: &[u8]) -> Result<(), ErrorCode> {
        if data.is_empty() {
            return Ok(());
        }

        self.rx_sm.set_enable(false);
        self.tx_sm.set_enable(false);
        self.tx_sm.clear_fifos();
        let _ = self.tx_sm.tx().stalled();
        self.set_tx_data_dir(Direction::Out);
        self.set_tx_control(RXF_BUSY);

        let Some((&first, rest)) = data.split_first() else {
            return Ok(());
        };
        self.tx_sm.tx().push(u32::from(first));
        self.tx_sm.restart();
        self.tx_sm.set_enable(true);

        let timeout = transfer_timeout(data.len());
        if !rest.is_empty() {
            let transfer = self.tx_sm.tx().dma_push(&mut self.tx_dma, rest, false);
            if with_timeout(timeout, transfer).await.is_err() {
                self.tx_sm.set_enable(false);
                self.idle_bus();
                return Err(ErrorCode::FpgaTimeout);
            }
        }

        Timer::after(tx_settle_time(data.len())).await;
        self.tx_sm.set_enable(false);
        self.idle_bus();
        self.stats.tx_bytes = self.stats.tx_bytes.saturating_add(data.len() as u32);
        Ok(())
    }

    /// Drain any stale FPGA-to-Pico bytes that were already queued.
    pub async fn drain_rx(&mut self) {
        let mut byte = [0u8; 1];
        for _ in 0..16 {
            if self
                .read_exact_timeout(&mut byte, Duration::from_millis(10))
                .await
                .is_err()
            {
                break;
            }
        }
    }

    /// Read exactly `out.len()` bytes from the FPGA byte protocol.
    pub async fn read_exact(&mut self, out: &mut [u8]) -> Result<(), ErrorCode> {
        self.read_exact_timeout(out, transfer_timeout(out.len()))
            .await?;
        self.stats.rx_bytes = self.stats.rx_bytes.saturating_add(out.len() as u32);
        Ok(())
    }

    async fn read_exact_timeout(
        &mut self,
        out: &mut [u8],
        timeout: Duration,
    ) -> Result<(), ErrorCode> {
        if out.is_empty() {
            return Ok(());
        }

        self.tx_sm.set_enable(false);
        self.rx_sm.set_enable(false);
        self.rx_sm.clear_fifos();
        self.set_rx_data_dir(Direction::In);
        self.rx_sm.restart();
        self.rx_sm.set_enable(true);
        self.txe_n.set_low();

        let transfer = self.rx_sm.rx().dma_pull(&mut self.rx_dma, out, false);
        if with_timeout(timeout, transfer).await.is_err() {
            self.txe_n.set_high();
            self.rx_sm.set_enable(false);
            self.idle_bus();
            return Err(ErrorCode::FpgaTimeout);
        }

        self.txe_n.set_high();
        self.rx_sm.set_enable(false);
        self.idle_bus();
        Ok(())
    }

    /// Write a command and wait for one ACK-like byte.
    pub async fn command_ack(&mut self, command: &[u8]) -> Result<(), ErrorCode> {
        const ATTEMPTS: usize = 3;
        for attempt in 0..ATTEMPTS {
            self.drain_rx().await;
            self.write_all(command).await?;
            for _ in 0..8 {
                match self.read_nonzero_ack().await {
                    Ok(0x01) => {
                        self.drain_rx().await;
                        return Ok(());
                    }
                    Ok(ack) if ack == FPGA_PROTOCOL_VERSION || ack == 0x02 => continue,
                    Ok(ack) => {
                        self.drain_rx().await;
                        return Err(ErrorCode::FpgaRejected(ack));
                    }
                    Err(ErrorCode::FpgaTimeout) if attempt + 1 < ATTEMPTS => break,
                    Err(error) => {
                        self.drain_rx().await;
                        return Err(error);
                    }
                }
            }
        }
        self.drain_rx().await;
        Err(ErrorCode::FpgaTimeout)
    }

    /// Read an ACK byte, skipping the same stray zeros tolerated by the host tool.
    pub async fn read_nonzero_ack(&mut self) -> Result<u8, ErrorCode> {
        let mut skipped_zero = 0u8;
        loop {
            let mut byte = [0u8; 1];
            self.read_exact(&mut byte).await?;
            let [byte] = byte;
            if byte == 0 {
                skipped_zero = skipped_zero.saturating_add(1);
                if skipped_zero > 8 {
                    return Err(ErrorCode::FpgaRejected(0));
                }
                continue;
            }
            return Ok(byte);
        }
    }

    fn idle_bus(&mut self) {
        self.set_tx_data_dir(Direction::In);
        self.set_rx_data_dir(Direction::In);
        self.txe_n.set_high();
        self.set_tx_control(RXF_IDLE);
    }

    fn set_tx_data_dir(&mut self, direction: Direction) {
        let data = self.data.each_ref();
        self.tx_sm.set_pin_dirs(direction, data.as_slice());
    }

    fn set_rx_data_dir(&mut self, direction: Direction) {
        let data = self.data.each_ref();
        self.rx_sm.set_pin_dirs(direction, data.as_slice());
    }

    fn set_tx_control(&mut self, value: u8) {
        set_control(&mut self.tx_sm, value);
    }
}

fn configure_tx_sm<'d>(
    sm: &mut StateMachine<'d, PIO0, 0>,
    program: &pio::LoadedProgram<'d, PIO0>,
    data: &[&pio::Pin<'d, PIO0>],
    control: &[&pio::Pin<'d, PIO0>],
) {
    sm.set_pin_dirs(Direction::In, data);
    sm.set_pin_dirs(Direction::Out, control);

    let mut cfg = Config::default();
    cfg.use_program(program, &[]);
    cfg.set_out_pins(data);
    cfg.set_set_pins(control);
    cfg.fifo_join = FifoJoin::TxOnly;
    cfg.shift_out = ShiftConfig {
        auto_fill: false,
        direction: ShiftDirection::Right,
        threshold: 8,
    };
    sm.set_config(&cfg);
}

fn configure_rx_sm<'d>(
    sm: &mut StateMachine<'d, PIO0, 1>,
    program: &pio::LoadedProgram<'d, PIO0>,
    data: &[&pio::Pin<'d, PIO0>],
) {
    sm.set_pin_dirs(Direction::In, data);

    let mut cfg = Config::default();
    cfg.use_program(program, &[]);
    cfg.set_in_pins(data);
    cfg.fifo_join = FifoJoin::RxOnly;
    cfg.shift_in = ShiftConfig {
        auto_fill: false,
        direction: ShiftDirection::Left,
        threshold: 8,
    };
    sm.set_config(&cfg);
}

fn set_control<'d, const SM: usize>(sm: &mut StateMachine<'d, PIO0, SM>, value: u8) {
    let instr = pio_set_pins_instr(value & 0x01);
    // SAFETY: The TX state machine maps SET pins only to GP8 (RXF#).
    // Executing a SET while disabled only updates that output latch and does
    // not advance the bus protocol state machine.
    unsafe { sm.exec_instr(instr) };
}

const fn pio_set_pins_instr(value: u8) -> u16 {
    0xE000 | value as u16
}

fn tx_settle_time(_bytes: usize) -> Duration {
    Duration::from_micros(500)
}

fn transfer_timeout(bytes: usize) -> Duration {
    Duration::from_millis(STROBE_TIMEOUT_MS + bytes as u64)
}
