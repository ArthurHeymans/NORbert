use cortex_m::asm;
use embassy_rp::gpio::{Flex, Input, Level, Output, Pin, Pull};
use embassy_rp::Peri;
use embassy_time::{with_timeout, Duration};
use norbert_pico_protocol::ErrorCode;

const STROBE_TIMEOUT: Duration = Duration::from_millis(100);

/// Timing-compatible FT245-device-side bus for the FPGA connector.
///
/// This backend deliberately presents the same signals as an FT2232H configured
/// in asynchronous 245 FIFO mode.  It is written as a conservative SIO backend
/// for bring-up; the public API is the seam where the PIO/DMA implementation
/// can replace the byte strobes without changing the RPC layer.
pub struct Ft245Bus<'d> {
    data: [Flex<'d>; 8],
    rxf_n: Output<'d>,
    txe_n: Output<'d>,
    rd_n: Input<'d>,
    wr_n: Input<'d>,
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
        d0: Peri<'d, impl Pin>,
        d1: Peri<'d, impl Pin>,
        d2: Peri<'d, impl Pin>,
        d3: Peri<'d, impl Pin>,
        d4: Peri<'d, impl Pin>,
        d5: Peri<'d, impl Pin>,
        d6: Peri<'d, impl Pin>,
        d7: Peri<'d, impl Pin>,
        rxf_n: Peri<'d, impl Pin>,
        txe_n: Peri<'d, impl Pin>,
        rd_n: Peri<'d, impl Pin>,
        wr_n: Peri<'d, impl Pin>,
    ) -> Self {
        let mut data = [
            Flex::new(d0),
            Flex::new(d1),
            Flex::new(d2),
            Flex::new(d3),
            Flex::new(d4),
            Flex::new(d5),
            Flex::new(d6),
            Flex::new(d7),
        ];
        for pin in &mut data {
            pin.set_as_input();
        }

        // Active-low FT245 status signals.  Idle means: no data for FPGA
        // (RXF# high), but the Pico can accept FPGA writes (TXE# low).
        let mut rxf_n = Output::new(rxf_n, Level::High);
        let mut txe_n = Output::new(txe_n, Level::Low);
        rxf_n.set_high();
        txe_n.set_low();

        Self {
            data,
            rxf_n,
            txe_n,
            rd_n: Input::new(rd_n, Pull::Up),
            wr_n: Input::new(wr_n, Pull::Up),
            stats: BusStats::default(),
        }
    }

    /// Return low-level bus counters.
    pub fn stats(&self) -> BusStats {
        self.stats
    }

    /// Write all bytes to the FPGA byte protocol.
    pub async fn write_all(&mut self, data: &[u8]) -> Result<(), ErrorCode> {
        for byte in data {
            self.write_byte(*byte).await?;
        }
        Ok(())
    }

    /// Read exactly `out.len()` bytes from the FPGA byte protocol.
    pub async fn read_exact(&mut self, out: &mut [u8]) -> Result<(), ErrorCode> {
        for byte in out {
            *byte = self.read_byte().await?;
        }
        Ok(())
    }

    /// Write a command and wait for one ACK-like byte.
    pub async fn command_ack(&mut self, command: &[u8]) -> Result<(), ErrorCode> {
        self.write_all(command).await?;
        let ack = self.read_nonzero_ack().await?;
        if ack == 0x01 {
            Ok(())
        } else {
            Err(ErrorCode::FpgaRejected(ack))
        }
    }

    /// Read an ACK byte, skipping the same stray zero bytes tolerated by the host tool.
    pub async fn read_nonzero_ack(&mut self) -> Result<u8, ErrorCode> {
        let mut skipped = 0u8;
        loop {
            let byte = self.read_byte().await?;
            if byte != 0 {
                return Ok(byte);
            }
            skipped = skipped.saturating_add(1);
            if skipped > 8 {
                return Err(ErrorCode::FpgaRejected(0));
            }
        }
    }

    async fn write_byte(&mut self, byte: u8) -> Result<(), ErrorCode> {
        // Prevent the FPGA from starting a write while the Pico owns the
        // bidirectional bus for a host-to-FPGA byte.
        self.txe_n.set_high();
        self.drive_data(byte);

        // Tell the FPGA a byte is available, then wait for its RD# pulse.
        self.rxf_n.set_low();
        with_timeout(STROBE_TIMEOUT, self.rd_n.wait_for_low())
            .await
            .map_err(|_| ErrorCode::FpgaTimeout)?;
        short_bus_delay();
        with_timeout(STROBE_TIMEOUT, self.rd_n.wait_for_high())
            .await
            .map_err(|_| ErrorCode::FpgaTimeout)?;

        self.rxf_n.set_high();
        self.release_data();
        self.txe_n.set_low();
        self.stats.tx_bytes = self.stats.tx_bytes.saturating_add(1);
        Ok(())
    }

    async fn read_byte(&mut self) -> Result<u8, ErrorCode> {
        self.release_data();
        self.txe_n.set_low();

        with_timeout(STROBE_TIMEOUT, self.wr_n.wait_for_low())
            .await
            .map_err(|_| ErrorCode::FpgaTimeout)?;
        short_bus_delay();
        let byte = self.sample_data();
        with_timeout(STROBE_TIMEOUT, self.wr_n.wait_for_high())
            .await
            .map_err(|_| ErrorCode::FpgaTimeout)?;

        self.stats.rx_bytes = self.stats.rx_bytes.saturating_add(1);
        Ok(byte)
    }

    fn drive_data(&mut self, byte: u8) {
        for (bit, pin) in self.data.iter_mut().enumerate() {
            if (byte & (1 << bit)) != 0 {
                pin.set_high();
            } else {
                pin.set_low();
            }
            pin.set_as_output();
        }
    }

    fn release_data(&mut self) {
        for pin in &mut self.data {
            pin.set_as_input();
        }
    }

    fn sample_data(&self) -> u8 {
        self.data.iter().enumerate().fold(0u8, |acc, (bit, pin)| {
            if pin.is_high() {
                acc | (1 << bit)
            } else {
                acc
            }
        })
    }
}

fn short_bus_delay() {
    // At 125 MHz this is about 128 ns, above the FT245 async data-valid and
    // setup/hold windows used by the FPGA-side Verilog state machine.
    asm::delay(16);
}
