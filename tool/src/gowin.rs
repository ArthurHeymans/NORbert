//! Experimental Gowin GW5A FPGA programmer for the browser WebUSB path.
//!
//! This module intentionally implements the small openFPGALoader subset needed
//! by NORbert's Tang Primer 25K target: FT2232H MPSSE/JTAG, Gowin `.fs` files,
//! volatile SRAM programming, and external SPI flash programming through the
//! GW5A JTAG-SPI bridge.

use ftdi_usb::constants::mpsse;
use ftdi_usb::mpsse::MpsseContext;
use ftdi_usb::{FtdiDevice, Interface};

const FTDI_VID: u16 = 0x0403;
const FT2232H_PID: u16 = 0x6010;
const TANG_PRIMER_25K_IDCODE: u32 = 0x0001_281b;
const GOWIN_IR_LEN: usize = 8;

const NOOP: u8 = 0x02;
const ERASE_SRAM: u8 = 0x05;
const XFER_DONE: u8 = 0x09;
const INIT_ADDR: u8 = 0x12;
const CONFIG_ENABLE: u8 = 0x15;
const XFER_WRITE: u8 = 0x17;
const CONFIG_DISABLE: u8 = 0x3a;
const RELOAD: u8 = 0x3c;
const STATUS_REGISTER: u8 = 0x41;

const STATUS_BAD_COMMAND: u32 = 1 << 1;
const STATUS_TIMEOUT: u32 = 1 << 3;
const STATUS_MEMORY_ERASE: u32 = 1 << 5;
const STATUS_SYSTEM_EDIT_MODE: u32 = 1 << 7;
const STATUS_DONE_FINAL: u32 = 1 << 13;

const FLASH_WRSR: u8 = 0x01;
const FLASH_PP: u8 = 0x02;
const FLASH_READ: u8 = 0x03;
const FLASH_RDSR: u8 = 0x05;
const FLASH_WREN: u8 = 0x06;
const FLASH_SE: u8 = 0x20;
const FLASH_POWER_UP: u8 = 0xab;
const FLASH_CE: u8 = 0xc7;
const FLASH_RSTEN: u8 = 0x66;
const FLASH_RST: u8 = 0x99;
const FLASH_RDID: u8 = 0x9f;
const FLASH_RDSR_WIP: u8 = 0x01;
const FLASH_RDSR_WEL: u8 = 0x02;
const FLASH_BP_MASK: u8 = 0x7c;

/// Describes a programming progress update.
#[derive(Debug, Clone)]
pub struct ProgramProgress {
    /// Human-readable stage name.
    pub stage: &'static str,
    /// Bytes or bits completed in the current stage.
    pub done: usize,
    /// Total bytes or bits in the current stage.
    pub total: usize,
}

/// Summarizes metadata parsed from a Gowin `.fs` file.
#[derive(Debug, Clone)]
pub struct BitstreamInfo {
    /// Gowin target IDCODE embedded in the bitstream.
    pub idcode: u32,
    /// Parsed checksum used by the SRAM programming flow.
    pub checksum: u32,
    /// Number of bytes shifted for SRAM programming.
    pub sram_bytes: usize,
    /// Number of bytes written for external flash programming.
    pub flash_bytes: usize,
}

/// Parses Gowin `.fs` metadata without opening WebUSB.
pub fn inspect_bitstream(fs: &[u8]) -> Result<BitstreamInfo, String> {
    let bitstream = GowinBitstream::parse(fs, true)?;
    Ok(BitstreamInfo {
        idcode: bitstream.idcode,
        checksum: bitstream.checksum,
        sram_bytes: bitstream.sram_data.len(),
        flash_bytes: bitstream.flash_data.len(),
    })
}

/// Controls external flash programming behavior.
#[derive(Debug, Clone, Copy)]
pub struct FlashOptions {
    /// Start address in the external SPI flash.
    pub offset: u32,
    /// Verify flash contents after programming.
    pub verify: bool,
    /// Allow clearing SPI flash block-protect bits before erase/program.
    pub unprotect: bool,
    /// Erase the entire flash chip instead of only touched 4 KiB sectors.
    pub chip_erase: bool,
}

/// Summarizes a successful programming operation.
#[derive(Debug, Clone)]
pub struct ProgramReport {
    /// JTAG IDCODE read from the FPGA.
    pub idcode: u32,
    /// Gowin `.fs` IDCODE embedded in the bitstream.
    pub bitstream_idcode: u32,
    /// Parsed bitstream checksum.
    pub checksum: u32,
    /// Number of bytes written to SRAM or flash.
    pub bytes_programmed: usize,
    /// Optional external SPI flash JEDEC ID.
    pub flash_jedec_id: Option<u32>,
}

/// Parses and programs a Gowin `.fs` bitstream into volatile FPGA SRAM.
pub async fn program_sram_from_webusb(
    fs: &[u8],
    mut progress: impl FnMut(ProgramProgress),
) -> Result<ProgramReport, String> {
    let bitstream = GowinBitstream::parse(fs, true)?;
    let mut programmer = GowinProgrammer::open().await?;
    let idcode = programmer.detect_idcode().await?;
    validate_idcode(idcode, bitstream.idcode)?;
    programmer.program_sram(&bitstream, &mut progress).await?;
    Ok(ProgramReport {
        idcode,
        bitstream_idcode: bitstream.idcode,
        checksum: bitstream.checksum,
        bytes_programmed: bitstream.sram_data.len(),
        flash_jedec_id: None,
    })
}

/// Parses and programs a Gowin `.fs` bitstream into the external SPI flash.
pub async fn program_flash_from_webusb(
    fs: &[u8],
    options: FlashOptions,
    mut progress: impl FnMut(ProgramProgress),
) -> Result<ProgramReport, String> {
    let bitstream = GowinBitstream::parse(fs, false)?;
    let mut programmer = GowinProgrammer::open().await?;
    let idcode = programmer.detect_idcode().await?;
    validate_idcode(idcode, bitstream.idcode)?;
    let jedec = programmer
        .program_external_flash(&bitstream, options, &mut progress)
        .await?;
    Ok(ProgramReport {
        idcode,
        bitstream_idcode: bitstream.idcode,
        checksum: bitstream.checksum,
        bytes_programmed: bitstream.flash_data.len(),
        flash_jedec_id: Some(jedec),
    })
}

fn validate_idcode(jtag_idcode: u32, bitstream_idcode: u32) -> Result<(), String> {
    if (jtag_idcode & 0x0fff_ffff) != (TANG_PRIMER_25K_IDCODE & 0x0fff_ffff) {
        return Err(format!(
            "unsupported FPGA IDCODE 0x{jtag_idcode:08x}; expected Tang Primer 25K/GW5A-25 0x{TANG_PRIMER_25K_IDCODE:08x}"
        ));
    }
    if (bitstream_idcode & 0x0fff_ffff) != (jtag_idcode & 0x0fff_ffff) {
        return Err(format!(
            "bitstream IDCODE 0x{bitstream_idcode:08x} does not match FPGA IDCODE 0x{jtag_idcode:08x}"
        ));
    }
    Ok(())
}

#[derive(Debug, Clone)]
struct GowinBitstream {
    idcode: u32,
    checksum: u32,
    sram_data: Vec<u8>,
    flash_data: Vec<u8>,
}

impl GowinBitstream {
    fn parse(input: &[u8], sram_reverse_bytes: bool) -> Result<Self, String> {
        let text =
            std::str::from_utf8(input).map_err(|e| format!("bitstream is not UTF-8: {e}"))?;
        let mut lines = Vec::new();
        let mut in_header = true;
        let mut end_header = None;
        let mut idcode = 0u32;
        let mut checksum_from_header = 0u32;
        let mut conf_data_len = None;
        let mut compressed = false;
        let mut crc_check = false;
        let mut zero8 = 0xffu8;
        let mut zero4 = 0xffu8;
        let mut zero2 = 0xffu8;

        for raw in text.lines() {
            let line = raw.trim_end_matches('\r');
            if line.is_empty() {
                break;
            }
            if line.starts_with('/') {
                continue;
            }
            if !line.bytes().all(|b| b == b'0' || b == b'1') {
                return Err("bitstream contains a non-binary data line".to_string());
            }
            if line.len() % 8 != 0 {
                return Err("bitstream line is not byte aligned".to_string());
            }

            let line_index = lines.len();
            lines.push(line.to_string());

            if !in_header {
                continue;
            }
            if line.len() < 8 {
                return Err("truncated Gowin .fs header line".to_string());
            }
            let key = bit_to_val(&line.as_bytes()[..8]) as u8 & 0x7f;
            let val = bit_to_val(line.as_bytes());
            match key {
                0x06 => {
                    if line.len() != 64 {
                        return Err("invalid Gowin .fs IDCODE header length".to_string());
                    }
                    idcode = val as u32;
                }
                0x0a => checksum_from_header = val as u32,
                0x10 => {
                    if line.len() != 64 {
                        return Err("invalid Gowin .fs options header length".to_string());
                    }
                    compressed = ((val >> 13) & 1) != 0;
                }
                0x51 => {
                    if line.len() != 64 {
                        return Err("invalid Gowin .fs compression header length".to_string());
                    }
                    zero8 = ((val >> 16) & 0xff) as u8;
                    zero4 = ((val >> 8) & 0xff) as u8;
                    zero2 = (val & 0xff) as u8;
                }
                0x3b => {
                    if line.len() != 32 {
                        return Err("invalid Gowin .fs configuration length header".to_string());
                    }
                    crc_check = ((val >> 23) & 1) != 0;
                    conf_data_len = Some((val & 0xffff) as usize);
                    end_header = Some(line_index);
                    in_header = false;
                }
                _ => {}
            }
        }

        if idcode == 0 {
            return Err("Gowin .fs file does not contain an IDCODE header".to_string());
        }
        let end_header = end_header
            .ok_or("Gowin .fs file does not contain a configuration header terminator")?;
        let conf_data_len =
            conf_data_len.ok_or("Gowin .fs file does not contain ConfDataLength")?;

        let sram_data = pack_lines(&lines, sram_reverse_bytes)?;
        let flash_data = if sram_reverse_bytes {
            pack_lines(&lines, false)?
        } else {
            sram_data.clone()
        };
        let checksum = compute_checksum(
            &lines,
            ChecksumOptions {
                end_header,
                conf_data_len,
                compressed,
                crc_check,
                zero8,
                zero4,
                zero2,
            },
        )?;

        Ok(Self {
            idcode,
            checksum: if checksum == 0 {
                checksum_from_header
            } else {
                checksum
            },
            sram_data,
            flash_data,
        })
    }
}

fn bit_to_val(bits: &[u8]) -> u64 {
    bits.iter()
        .fold(0u64, |acc, &bit| (acc << 1) | u64::from(bit == b'1'))
}

fn pack_lines(lines: &[String], reverse_bytes: bool) -> Result<Vec<u8>, String> {
    let mut out = Vec::new();
    for line in lines {
        if line.len() % 8 != 0 {
            return Err("bitstream line is not byte aligned".to_string());
        }
        for chunk in line.as_bytes().chunks_exact(8) {
            let byte = bit_to_val(chunk) as u8;
            out.push(if reverse_bytes {
                reverse_byte(byte)
            } else {
                byte
            });
        }
    }
    Ok(out)
}

struct ChecksumOptions {
    end_header: usize,
    conf_data_len: usize,
    compressed: bool,
    crc_check: bool,
    zero8: u8,
    zero4: u8,
    zero2: u8,
}

fn compute_checksum(lines: &[String], options: ChecksumOptions) -> Result<u32, String> {
    let nb_line = options.conf_data_len.min(65_535);
    let data_lines = lines
        .get(options.end_header + 1..)
        .ok_or("invalid Gowin .fs header/data split")?;
    let data_lines = &data_lines[..data_lines.len().min(nb_line)];
    let drop_bits = if options.crc_check { 8 * 8 } else { 6 * 8 };
    let mut checksum_bits = String::new();

    for line in data_lines {
        if line.len() < drop_bits {
            return Err("truncated Gowin .fs configuration line".to_string());
        }
        if options.compressed {
            let payload = &line[..line.len() - drop_bits];
            if payload.len() % 8 != 0 {
                return Err("compressed Gowin .fs line is not byte aligned".to_string());
            }
            for chunk in payload.as_bytes().chunks_exact(8) {
                let byte = bit_to_val(chunk) as u8;
                if byte == options.zero8 {
                    checksum_bits.push_str(
                        "0000000000000000000000000000000000000000000000000000000000000000",
                    );
                } else if byte == options.zero4 {
                    checksum_bits.push_str("00000000000000000000000000000000");
                } else if byte == options.zero2 {
                    checksum_bits.push_str("0000000000000000");
                } else {
                    checksum_bits.push_str(std::str::from_utf8(chunk).map_err(|e| e.to_string())?);
                }
            }
        } else {
            checksum_bits.push_str(&line[..line.len() - drop_bits]);
        }
    }

    if checksum_bits.is_empty() {
        return Ok(0);
    }
    if !checksum_bits.len().is_multiple_of(16) {
        return Err("Gowin .fs checksum data is not 16-bit aligned".to_string());
    }
    let mut checksum = 0u32;
    for chunk in checksum_bits.as_bytes().chunks_exact(16) {
        checksum = checksum.wrapping_add(bit_to_val(chunk) as u32 & 0xffff);
    }
    Ok(checksum)
}

fn reverse_byte(byte: u8) -> u8 {
    byte.reverse_bits()
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TapState {
    TestLogicReset,
    RunTestIdle,
    SelectDrScan,
    CaptureDr,
    ShiftDr,
    Exit1Dr,
    PauseDr,
    Exit2Dr,
    UpdateDr,
    SelectIrScan,
    CaptureIr,
    ShiftIr,
    Exit1Ir,
    PauseIr,
    Exit2Ir,
    UpdateIr,
}

struct GowinProgrammer {
    jtag: FtdiJtag,
}

impl GowinProgrammer {
    async fn open() -> Result<Self, String> {
        let jtag = FtdiJtag::open_tang_primer_25k().await?;
        Ok(Self { jtag })
    }

    async fn detect_idcode(&mut self) -> Result<u32, String> {
        self.jtag.go_test_logic_reset().await?;
        self.jtag.set_state(TapState::ShiftDr, 1).await?;
        let rx = self.jtag.shift_bits(&[0xff; 4], 32, true, true).await?;
        self.jtag.set_state(TapState::RunTestIdle, 1).await?;
        let bytes: [u8; 4] = rx
            .get(..4)
            .ok_or("short JTAG IDCODE read")?
            .try_into()
            .map_err(|_| "invalid JTAG IDCODE read".to_string())?;
        Ok(u32::from_le_bytes(bytes))
    }

    async fn program_sram(
        &mut self,
        bitstream: &GowinBitstream,
        progress: &mut impl FnMut(ProgramProgress),
    ) -> Result<(), String> {
        self.reset_gowin().await?;
        self.jtag.set_state(TapState::RunTestIdle, 1).await?;
        self.jtag.toggle_clk(1_000_000).await?;
        self.erase_sram().await?;
        self.write_sram(bitstream, progress).await
    }

    async fn program_external_flash(
        &mut self,
        bitstream: &GowinBitstream,
        options: FlashOptions,
        progress: &mut impl FnMut(ProgramProgress),
    ) -> Result<u32, String> {
        self.prepare_flash_access().await?;
        let result = self
            .program_spi_flash(&bitstream.flash_data, options, progress)
            .await;
        let post_result = self.post_flash_access().await;
        match (result, post_result) {
            (Ok(jedec), Ok(())) => Ok(jedec),
            (Err(err), _) => Err(err),
            (Ok(_), Err(err)) => Err(err),
        }
    }

    async fn send_command(&mut self, cmd: u8) -> Result<(), String> {
        self.jtag.shift_ir(&[cmd], GOWIN_IR_LEN).await?;
        self.jtag.toggle_clk(6).await
    }

    async fn read_reg32(&mut self, cmd: u8) -> Result<u32, String> {
        self.send_command(cmd).await?;
        let rx = self.jtag.shift_dr(&[0xff; 4], 32, true).await?;
        let bytes: [u8; 4] = rx
            .get(..4)
            .ok_or("short Gowin register read")?
            .try_into()
            .map_err(|_| "invalid Gowin register read".to_string())?;
        Ok(u32::from_le_bytes(bytes))
    }

    async fn read_status(&mut self) -> Result<u32, String> {
        self.read_reg32(STATUS_REGISTER).await
    }

    async fn reset_gowin(&mut self) -> Result<(), String> {
        self.send_command(RELOAD).await?;
        self.send_command(NOOP).await
    }

    async fn enable_cfg(&mut self) -> Result<(), String> {
        self.send_command(CONFIG_ENABLE).await?;
        self.poll_status(
            STATUS_SYSTEM_EDIT_MODE,
            STATUS_SYSTEM_EDIT_MODE,
            "enable configuration",
        )
        .await
    }

    async fn disable_cfg(&mut self) -> Result<(), String> {
        self.send_command(CONFIG_DISABLE).await?;
        self.send_command(NOOP).await?;
        self.poll_status(STATUS_SYSTEM_EDIT_MODE, 0, "disable configuration")
            .await
    }

    async fn poll_status(&mut self, mask: u32, value: u32, context: &str) -> Result<(), String> {
        for attempt in 0..100_000u32 {
            let status = self.read_status().await?;
            if (status & mask) == value {
                return Ok(());
            }
            if attempt % 128 == 0 {
                sleep_ms(0).await;
            }
        }
        Err(format!("timeout waiting for {context}"))
    }

    async fn erase_sram(&mut self) -> Result<(), String> {
        let status = self.read_status().await?;
        if (status & ((1 << 4) | STATUS_TIMEOUT | STATUS_BAD_COMMAND)) != 0 {
            self.send_command(CONFIG_ENABLE).await?;
            self.send_command(0x3f).await?;
            self.send_command(CONFIG_DISABLE).await?;
            self.send_command(NOOP).await?;
            self.send_command(0x11).await?;
            self.send_command(NOOP).await?;
            self.jtag.toggle_clk(125 * 8).await?;
        }

        let mut must_loop = true;
        let mut loop_count = 0u8;
        while must_loop {
            self.enable_cfg().await?;
            self.send_command(ERASE_SRAM).await?;
            self.send_command(NOOP).await?;
            self.poll_status(STATUS_MEMORY_ERASE, STATUS_MEMORY_ERASE, "SRAM erase")
                .await?;
            self.send_command(XFER_DONE).await?;
            self.send_command(NOOP).await?;
            self.disable_cfg().await?;

            let status = self.read_status().await?;
            if loop_count >= 1 && (status & STATUS_DONE_FINAL) == 0 {
                must_loop = false;
            }
            loop_count = loop_count.saturating_add(1);
        }
        Ok(())
    }

    async fn write_sram(
        &mut self,
        bitstream: &GowinBitstream,
        progress: &mut impl FnMut(ProgramProgress),
    ) -> Result<(), String> {
        self.send_command(CONFIG_ENABLE).await?;
        self.send_command(INIT_ADDR).await?;
        self.send_command(XFER_WRITE).await?;

        let total_bits = bitstream.sram_data.len() * 8;
        let mut bit_offset = 0usize;
        const CHUNK_BITS: usize = 0x80000;
        while bit_offset < total_bits {
            let remaining = total_bits - bit_offset;
            let chunk_bits = remaining.min(CHUNK_BITS);
            let byte_offset = bit_offset / 8;
            let byte_len = chunk_bits.div_ceil(8);
            let end_state = remaining == chunk_bits;
            self.jtag
                .shift_dr_keep_state(
                    &bitstream.sram_data[byte_offset..byte_offset + byte_len],
                    chunk_bits,
                    end_state,
                )
                .await?;
            bit_offset += chunk_bits;
            progress(ProgramProgress {
                stage: "Writing SRAM",
                done: bit_offset,
                total: total_bits,
            });
            sleep_ms(0).await;
        }

        self.send_command(0x0a).await?;
        let checksum = bitstream.checksum.to_le_bytes();
        self.jtag.shift_dr(&checksum, 32, false).await?;
        self.send_command(0x08).await?;
        self.send_command(CONFIG_DISABLE).await?;
        self.send_command(NOOP).await?;

        let status = self.read_status().await?;
        if (status & STATUS_DONE_FINAL) == 0 {
            return Err(format!(
                "FPGA did not assert DONE after SRAM program; status=0x{status:08x}"
            ));
        }
        Ok(())
    }

    async fn prepare_flash_access(&mut self) -> Result<(), String> {
        self.reset_gowin().await?;
        self.jtag.set_state(TapState::RunTestIdle, 1).await?;
        self.jtag.toggle_clk(1_000_000).await?;
        self.erase_sram().await?;
        sleep_ms(100).await;
        self.gw5a_enable_spi().await?;
        sleep_ms(100).await;
        self.jtag.set_clock(10_000_000).await
    }

    async fn post_flash_access(&mut self) -> Result<(), String> {
        self.gw5a_disable_spi().await?;
        self.reset_gowin().await
    }

    async fn gw5a_enable_spi(&mut self) -> Result<(), String> {
        self.enable_cfg().await?;
        self.send_command(0x3f).await?;
        self.disable_cfg().await?;
        self.send_command(NOOP).await?;
        self.jtag.set_state(TapState::RunTestIdle, 1).await?;
        self.jtag.toggle_clk(126 * 8).await?;
        self.jtag.set_state(TapState::RunTestIdle, 1).await?;
        self.send_command(0x16).await?;
        self.send_command(0x00).await?;
        self.jtag.set_state(TapState::RunTestIdle, 1).await?;
        self.jtag.toggle_clk(625 * 8).await?;
        self.jtag.set_state(TapState::TestLogicReset, 1).await
    }

    async fn gw5a_disable_spi(&mut self) -> Result<(), String> {
        self.jtag.set_state(TapState::SelectDrScan, 1).await?;
        self.jtag.set_state(TapState::CaptureDr, 1).await?;
        self.jtag.set_state(TapState::Exit1Dr, 1).await?;
        self.jtag.set_state(TapState::Exit2Dr, 1).await?;
        for _ in 0..6 {
            self.jtag.set_state(TapState::PauseDr, 1).await?;
            self.jtag.set_state(TapState::Exit2Dr, 1).await?;
        }
        self.jtag.set_state(TapState::Exit1Dr, 1).await?;
        self.jtag.set_state(TapState::PauseDr, 1).await?;
        self.jtag.set_state(TapState::TestLogicReset, 1).await?;
        self.jtag.toggle_clk(5).await
    }

    async fn spi_transfer(
        &mut self,
        cmd: u8,
        write_after_cmd: &[u8],
        read_len: usize,
    ) -> Result<Vec<u8>, String> {
        let transfer_len = write_after_cmd.len() + read_len;
        let rx_enabled = read_len > 0;
        let k_len = transfer_len + usize::from(rx_enabled);
        let bit_len = transfer_len * 8 + if rx_enabled { 3 } else { 0 };
        let mut jtx = vec![cmd & 1; k_len.max(1)];
        let mut plain = Vec::with_capacity(transfer_len);
        plain.extend_from_slice(write_after_cmd);
        plain.resize(transfer_len, 0);
        let curr_tdi = plain.last().copied().unwrap_or(cmd) & 1;
        for (dst, src) in jtx.iter_mut().zip(plain.iter().copied()) {
            *dst = reverse_byte(src);
        }

        let mut shifted_cmd = reverse_byte(cmd);
        self.jtag
            .set_state(TapState::RunTestIdle, shifted_cmd & 1)
            .await?;
        shifted_cmd >>= 1;
        self.jtag
            .read_write_bits(&[shifted_cmd], 7, false, false)
            .await?;
        let jrx = self
            .jtag
            .read_write_bits(&jtx, bit_len, rx_enabled, false)
            .await?;
        self.jtag
            .set_state(TapState::TestLogicReset, curr_tdi)
            .await?;
        self.jtag.toggle_clk(5).await?;

        if !rx_enabled {
            return Ok(Vec::new());
        }
        let mut rx = vec![0u8; transfer_len];
        for (i, byte) in rx.iter_mut().enumerate() {
            let a = *jrx.get(i).unwrap_or(&0);
            let b = *jrx.get(i + 1).unwrap_or(&0);
            *byte = reverse_byte((a >> 3) | ((b & 0x07) << 5));
        }
        Ok(rx[write_after_cmd.len()..].to_vec())
    }

    async fn spi_wait(
        &mut self,
        cmd: u8,
        mask: u8,
        value: u8,
        timeout: u32,
        context: &str,
    ) -> Result<(), String> {
        for attempt in 0..timeout {
            let status = self.spi_transfer(cmd, &[], 1).await?[0];
            if (status & mask) == value {
                return Ok(());
            }
            if attempt % 32 == 0 {
                sleep_ms(0).await;
            }
        }
        Err(format!("timeout waiting for SPI flash {context}"))
    }

    async fn program_spi_flash(
        &mut self,
        data: &[u8],
        options: FlashOptions,
        progress: &mut impl FnMut(ProgramProgress),
    ) -> Result<u32, String> {
        self.spi_reset().await?;
        self.spi_transfer(FLASH_POWER_UP, &[], 0).await?;
        let id = self.spi_transfer(FLASH_RDID, &[], 4).await?;
        if id.len() < 3 || id[..3] == [0, 0, 0] || id[..3] == [0xff, 0xff, 0xff] {
            return Err(format!("failed to read SPI flash JEDEC ID: {id:02x?}"));
        }
        let jedec = (u32::from(id[0]) << 16) | (u32::from(id[1]) << 8) | u32::from(id[2]);

        let status = self.spi_read_status().await?;
        if (status & FLASH_BP_MASK) != 0 {
            if !options.unprotect {
                return Err(format!(
                    "SPI flash block protection is set (status=0x{status:02x}); enable unprotect to clear it"
                ));
            }
            self.spi_write_status(status & !FLASH_BP_MASK).await?;
        }

        if options.chip_erase {
            self.spi_write_enable().await?;
            self.spi_transfer(FLASH_CE, &[], 0).await?;
            self.spi_wait(FLASH_RDSR, FLASH_RDSR_WIP, 0, 2_000_000, "chip erase")
                .await?;
        } else {
            let start = options.offset & !0xfff;
            let end = (options.offset + data.len() as u32 + 0xfff) & !0xfff;
            let total = (end - start) as usize;
            let mut erased = 0usize;
            for addr in (start..end).step_by(0x1000) {
                self.spi_write_enable().await?;
                self.spi_transfer(FLASH_SE, &addr24(addr), 0).await?;
                self.spi_wait(FLASH_RDSR, FLASH_RDSR_WIP, 0, 200_000, "sector erase")
                    .await?;
                erased += 0x1000;
                progress(ProgramProgress {
                    stage: "Erasing flash",
                    done: erased.min(total),
                    total,
                });
                sleep_ms(0).await;
            }
        }

        for (page_index, chunk) in data.chunks(256).enumerate() {
            let addr = options.offset + (page_index * 256) as u32;
            let mut payload = Vec::with_capacity(3 + chunk.len());
            payload.extend_from_slice(&addr24(addr));
            payload.extend_from_slice(chunk);
            self.spi_write_enable().await?;
            self.spi_transfer(FLASH_PP, &payload, 0).await?;
            self.spi_wait(FLASH_RDSR, FLASH_RDSR_WIP, 0, 1000, "page program")
                .await?;
            let done = ((page_index + 1) * 256).min(data.len());
            progress(ProgramProgress {
                stage: "Writing flash",
                done,
                total: data.len(),
            });
            sleep_ms(0).await;
        }

        if options.verify {
            let mut verified = 0usize;
            for (chunk_index, expected) in data.chunks(256).enumerate() {
                let addr = options.offset + (chunk_index * 256) as u32;
                let actual = self
                    .spi_transfer(FLASH_READ, &addr24(addr), expected.len())
                    .await?;
                if actual != expected {
                    return Err(format!(
                        "flash verification failed at address 0x{addr:06x}: expected {:02x?}, got {:02x?}",
                        &expected[..expected.len().min(16)],
                        &actual[..actual.len().min(16)]
                    ));
                }
                verified += expected.len();
                progress(ProgramProgress {
                    stage: "Verifying flash",
                    done: verified,
                    total: data.len(),
                });
                sleep_ms(0).await;
            }
        }

        Ok(jedec)
    }

    async fn spi_reset(&mut self) -> Result<(), String> {
        self.spi_transfer(0xff, &[0xff; 8], 0).await?;
        self.spi_transfer(FLASH_RSTEN, &[], 0).await?;
        self.spi_transfer(FLASH_RST, &[], 0).await?;
        sleep_ms(1).await;
        Ok(())
    }

    async fn spi_read_status(&mut self) -> Result<u8, String> {
        Ok(self.spi_transfer(FLASH_RDSR, &[], 1).await?[0])
    }

    async fn spi_write_enable(&mut self) -> Result<(), String> {
        self.spi_transfer(FLASH_WREN, &[], 0).await?;
        self.spi_wait(
            FLASH_RDSR,
            FLASH_RDSR_WEL,
            FLASH_RDSR_WEL,
            1000,
            "write enable",
        )
        .await
    }

    async fn spi_write_status(&mut self, status: u8) -> Result<(), String> {
        self.spi_write_enable().await?;
        self.spi_transfer(FLASH_WRSR, &[status], 0).await?;
        self.spi_wait(FLASH_RDSR, FLASH_RDSR_WIP, 0, 1000, "write status")
            .await
    }
}

fn addr24(addr: u32) -> [u8; 3] {
    [
        ((addr >> 16) & 0xff) as u8,
        ((addr >> 8) & 0xff) as u8,
        (addr & 0xff) as u8,
    ]
}

struct FtdiJtag {
    dev: FtdiDevice,
    ctx: MpsseContext,
    state: TapState,
}

impl FtdiJtag {
    async fn open_tang_primer_25k() -> Result<Self, String> {
        let info = FtdiDevice::request_device()
            .await
            .map_err(|e| format!("WebUSB FTDI request failed: {e}"))?;
        let mut dev = FtdiDevice::open_wasm(info, Interface::A)
            .await
            .map_err(|e| format!("failed to open FT2232H interface A: {e}"))?;
        if dev.chip_type().is_h_type() && dev.max_packet_size() == 0 {
            return Err(format!(
                "invalid FTDI device {FTDI_VID:04x}:{FT2232H_PID:04x}: zero max packet size"
            ));
        }
        let mut ctx = MpsseContext::init(&mut dev, 2_500_000)
            .await
            .map_err(|e| format!("failed to initialize FTDI MPSSE: {e}"))?;
        // openFPGALoader's `ft2232` cable config for Tang Primer boards.
        ctx.set_gpio_low(&mut dev, 0x08, 0x0b)
            .await
            .map_err(|e| format!("failed to configure FTDI low GPIO: {e}"))?;
        ctx.set_gpio_high(&mut dev, 0x08, 0x0b)
            .await
            .map_err(|e| format!("failed to configure FTDI high GPIO: {e}"))?;
        let mut this = Self {
            dev,
            ctx,
            state: TapState::TestLogicReset,
        };
        this.go_test_logic_reset().await?;
        Ok(this)
    }

    async fn set_clock(&mut self, hz: u32) -> Result<(), String> {
        self.ctx
            .set_clock(&mut self.dev, hz)
            .await
            .map_err(|e| format!("failed to set JTAG clock: {e}"))
    }

    async fn go_test_logic_reset(&mut self) -> Result<(), String> {
        self.write_tms_bits(&[true; 6], 1).await?;
        self.state = TapState::TestLogicReset;
        Ok(())
    }

    async fn shift_ir(&mut self, tdi: &[u8], bit_len: usize) -> Result<Vec<u8>, String> {
        self.set_state(TapState::ShiftIr, 1).await?;
        let tdo = self.shift_bits(tdi, bit_len, true, false).await?;
        self.set_state(TapState::RunTestIdle, 1).await?;
        Ok(tdo)
    }

    async fn shift_dr(
        &mut self,
        tdi: &[u8],
        bit_len: usize,
        read: bool,
    ) -> Result<Vec<u8>, String> {
        if self.state != TapState::ShiftDr {
            self.set_state(TapState::ShiftDr, 1).await?;
        }
        let tdo = self.shift_bits(tdi, bit_len, true, read).await?;
        self.set_state(TapState::RunTestIdle, 1).await?;
        Ok(tdo)
    }

    async fn shift_dr_keep_state(
        &mut self,
        tdi: &[u8],
        bit_len: usize,
        exit_to_idle: bool,
    ) -> Result<(), String> {
        if self.state != TapState::ShiftDr {
            self.set_state(TapState::ShiftDr, 1).await?;
        }
        self.shift_bits(tdi, bit_len, exit_to_idle, false).await?;
        if exit_to_idle {
            self.set_state(TapState::RunTestIdle, 1).await?;
        }
        Ok(())
    }

    async fn read_write_bits(
        &mut self,
        tdi: &[u8],
        bit_len: usize,
        read: bool,
        last: bool,
    ) -> Result<Vec<u8>, String> {
        self.shift_bits(tdi, bit_len, last, read).await
    }

    async fn shift_bits(
        &mut self,
        tdi_data: &[u8],
        bit_count: usize,
        exit_shift: bool,
        read: bool,
    ) -> Result<Vec<u8>, String> {
        if bit_count == 0 {
            return Ok(Vec::new());
        }
        let byte_count = bit_count.div_ceil(8);
        let mut tdi = vec![0u8; byte_count];
        let copy_len = tdi_data.len().min(byte_count);
        tdi[..copy_len].copy_from_slice(&tdi_data[..copy_len]);
        let normal_bits = if exit_shift { bit_count - 1 } else { bit_count };
        let full_bytes = normal_bits / 8;
        let remaining_bits = normal_bits % 8;
        let rw_cmd =
            mpsse::DO_WRITE | if read { mpsse::DO_READ } else { 0 } | mpsse::WRITE_NEG | mpsse::LSB;
        let mut cmd = Vec::with_capacity(byte_count + 32);

        let mut offset = 0usize;
        while offset < full_bytes {
            let chunk = (full_bytes - offset).min(65_536);
            let len_field = (chunk - 1) as u16;
            cmd.push(rw_cmd);
            cmd.push(len_field as u8);
            cmd.push((len_field >> 8) as u8);
            cmd.extend_from_slice(&tdi[offset..offset + chunk]);
            offset += chunk;
        }
        if remaining_bits > 0 {
            cmd.push(rw_cmd | mpsse::BITMODE);
            cmd.push((remaining_bits - 1) as u8);
            cmd.push(tdi[full_bytes]);
        }
        if exit_shift {
            let last_bit_idx = bit_count - 1;
            let last_tdi = (tdi[last_bit_idx / 8] >> (last_bit_idx % 8)) & 1;
            cmd.push(
                mpsse::WRITE_TMS
                    | if read { mpsse::DO_READ } else { 0 }
                    | mpsse::WRITE_NEG
                    | mpsse::BITMODE
                    | mpsse::LSB,
            );
            cmd.push(0);
            cmd.push(0x01 | (last_tdi << 7));
            self.state = match self.state {
                TapState::ShiftDr => TapState::Exit1Dr,
                TapState::ShiftIr => TapState::Exit1Ir,
                other => other,
            };
        }
        if read {
            cmd.push(mpsse::SEND_IMMEDIATE);
        }
        self.dev
            .write_all(&cmd)
            .await
            .map_err(|e| format!("JTAG shift write failed: {e}"))?;

        if !read {
            return Ok(Vec::new());
        }
        let mut expected = full_bytes;
        if remaining_bits > 0 {
            expected += 1;
        }
        if exit_shift {
            expected += 1;
        }
        let response = self.read_exact(expected).await?;
        let mut tdo = vec![0u8; byte_count];
        let mut tdo_bit = 0usize;
        let mut resp_idx = 0usize;
        if full_bytes > 0 {
            tdo[..full_bytes].copy_from_slice(&response[..full_bytes]);
            tdo_bit += full_bytes * 8;
            resp_idx += full_bytes;
        }
        if remaining_bits > 0 {
            let shifted = response[resp_idx] >> (8 - remaining_bits);
            for bit in 0..remaining_bits {
                if (shifted & (1 << bit)) != 0 {
                    tdo[tdo_bit / 8] |= 1 << (tdo_bit % 8);
                }
                tdo_bit += 1;
            }
            resp_idx += 1;
        }
        if exit_shift && (response[resp_idx] & 0x80) != 0 {
            tdo[tdo_bit / 8] |= 1 << (tdo_bit % 8);
        }
        Ok(tdo)
    }

    async fn set_state(&mut self, new_state: TapState, tdi: u8) -> Result<(), String> {
        let mut bits = Vec::new();
        while self.state != new_state {
            let tms = match self.state {
                TapState::TestLogicReset => {
                    if new_state == TapState::TestLogicReset {
                        true
                    } else {
                        self.state = TapState::RunTestIdle;
                        false
                    }
                }
                TapState::RunTestIdle => {
                    if new_state == TapState::RunTestIdle {
                        false
                    } else {
                        self.state = TapState::SelectDrScan;
                        true
                    }
                }
                TapState::SelectDrScan => match new_state {
                    TapState::CaptureDr
                    | TapState::ShiftDr
                    | TapState::Exit1Dr
                    | TapState::PauseDr
                    | TapState::Exit2Dr
                    | TapState::UpdateDr => {
                        self.state = TapState::CaptureDr;
                        false
                    }
                    _ => {
                        self.state = TapState::SelectIrScan;
                        true
                    }
                },
                TapState::SelectIrScan => match new_state {
                    TapState::CaptureIr
                    | TapState::ShiftIr
                    | TapState::Exit1Ir
                    | TapState::PauseIr
                    | TapState::Exit2Ir
                    | TapState::UpdateIr => {
                        self.state = TapState::CaptureIr;
                        false
                    }
                    _ => {
                        self.state = TapState::TestLogicReset;
                        true
                    }
                },
                TapState::CaptureDr => {
                    if new_state == TapState::ShiftDr {
                        self.state = TapState::ShiftDr;
                        false
                    } else {
                        self.state = TapState::Exit1Dr;
                        true
                    }
                }
                TapState::ShiftDr => {
                    if new_state == TapState::ShiftDr {
                        false
                    } else {
                        self.state = TapState::Exit1Dr;
                        true
                    }
                }
                TapState::Exit1Dr => match new_state {
                    TapState::PauseDr
                    | TapState::Exit2Dr
                    | TapState::ShiftDr
                    | TapState::Exit1Dr => {
                        self.state = TapState::PauseDr;
                        false
                    }
                    _ => {
                        self.state = TapState::UpdateDr;
                        true
                    }
                },
                TapState::PauseDr => {
                    if new_state == TapState::PauseDr {
                        false
                    } else {
                        self.state = TapState::Exit2Dr;
                        true
                    }
                }
                TapState::Exit2Dr => match new_state {
                    TapState::ShiftDr | TapState::Exit1Dr | TapState::PauseDr => {
                        self.state = TapState::ShiftDr;
                        false
                    }
                    _ => {
                        self.state = TapState::UpdateDr;
                        true
                    }
                },
                TapState::UpdateDr | TapState::UpdateIr => {
                    if new_state == TapState::RunTestIdle {
                        self.state = TapState::RunTestIdle;
                        false
                    } else {
                        self.state = TapState::SelectDrScan;
                        true
                    }
                }
                TapState::CaptureIr => {
                    if new_state == TapState::ShiftIr {
                        self.state = TapState::ShiftIr;
                        false
                    } else {
                        self.state = TapState::Exit1Ir;
                        true
                    }
                }
                TapState::ShiftIr => {
                    if new_state == TapState::ShiftIr {
                        false
                    } else {
                        self.state = TapState::Exit1Ir;
                        true
                    }
                }
                TapState::Exit1Ir => match new_state {
                    TapState::PauseIr
                    | TapState::Exit2Ir
                    | TapState::ShiftIr
                    | TapState::Exit1Ir => {
                        self.state = TapState::PauseIr;
                        false
                    }
                    _ => {
                        self.state = TapState::UpdateIr;
                        true
                    }
                },
                TapState::PauseIr => {
                    if new_state == TapState::PauseIr {
                        false
                    } else {
                        self.state = TapState::Exit2Ir;
                        true
                    }
                }
                TapState::Exit2Ir => match new_state {
                    TapState::ShiftIr | TapState::Exit1Ir | TapState::PauseIr => {
                        self.state = TapState::ShiftIr;
                        false
                    }
                    _ => {
                        self.state = TapState::UpdateIr;
                        true
                    }
                },
            };
            bits.push(tms);
        }
        if !bits.is_empty() {
            self.write_tms_bits(&bits, tdi).await?;
        }
        Ok(())
    }

    async fn write_tms_bits(&mut self, bits: &[bool], tdi: u8) -> Result<(), String> {
        if bits.is_empty() {
            return Ok(());
        }
        let mut cmd = Vec::with_capacity(bits.len().div_ceil(6) * 3);
        let mut offset = 0usize;
        while offset < bits.len() {
            let chunk_len = (bits.len() - offset).min(6);
            let mut data = (tdi & 1) << 7;
            let mut last_tms = false;
            for i in 0..chunk_len {
                last_tms = bits[offset + i];
                if last_tms {
                    data |= 1 << i;
                }
            }
            if last_tms {
                data |= 1 << chunk_len;
            }
            cmd.push(mpsse::WRITE_TMS | mpsse::WRITE_NEG | mpsse::BITMODE | mpsse::LSB);
            cmd.push((chunk_len - 1) as u8);
            cmd.push(data);
            offset += chunk_len;
        }
        self.dev
            .write_all(&cmd)
            .await
            .map_err(|e| format!("JTAG TMS write failed: {e}"))
    }

    async fn toggle_clk(&mut self, cycles: u32) -> Result<(), String> {
        let mut remaining = cycles;
        let mut cmd = Vec::new();
        while remaining >= 8 {
            let bytes = (remaining / 8).min(65_536) as u16;
            let field = bytes - 1;
            cmd.push(mpsse::CLK_BYTES);
            cmd.push(field as u8);
            cmd.push((field >> 8) as u8);
            remaining -= u32::from(bytes) * 8;
        }
        if remaining > 0 {
            cmd.push(mpsse::CLK_BITS);
            cmd.push((remaining - 1) as u8);
        }
        self.dev
            .write_all(&cmd)
            .await
            .map_err(|e| format!("JTAG clock toggle failed: {e}"))
    }

    async fn read_exact(&mut self, len: usize) -> Result<Vec<u8>, String> {
        let mut buf = vec![0u8; len];
        let mut offset = 0usize;
        while offset < len {
            let n = self
                .dev
                .read_data(&mut buf[offset..])
                .await
                .map_err(|e| format!("JTAG read failed: {e}"))?;
            if n == 0 {
                return Err("JTAG read returned no data".to_string());
            }
            offset += n;
        }
        Ok(buf)
    }
}

async fn sleep_ms(ms: i32) {
    let promise = js_sys::Promise::new(&mut |resolve, _| {
        if let Some(window) = web_sys::window() {
            let _ = window.set_timeout_with_callback_and_timeout_and_arguments_0(&resolve, ms);
        }
    });
    let _ = wasm_bindgen_futures::JsFuture::from(promise).await;
}
