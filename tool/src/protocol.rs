//! NORbert host protocol helpers shared by the CLI and WebUI.
//!
//! This module intentionally contains only transport-independent logic:
//! command constants, packet builders, validation, SPI-log parsing, and small
//! formatting/parsing helpers.  Native and WASM frontends provide their own I/O
//! transports but use these routines so the wire protocol stays in one place.

use crate::chip::FlashChip;
use crate::sfdp::generate_sfdp;

/// UART baud rate used by NORbert.
pub const BAUD_RATE: u32 = 2_000_000;
/// Maximum write/read burst size for the FPGA protocol.
pub const BLOCK_SIZE: usize = 16384;
/// Safe UART read block size that respects SDRAM refresh timing.
pub const UART_READ_BLOCK_SIZE: usize = 4096;
/// SDRAM capacity available as the flash backing store.
pub const SDRAM_SIZE_BYTES: u64 = 64 * 1024 * 1024;
/// Protocol version implemented by this tool.
pub const PROTOCOL_VERSION: u8 = 5;

/// Protocol commands shared between UART and FT245 transports.
pub const CMD_VERSION: u8 = 0x30;
pub const CMD_READ: u8 = 0x31;
pub const CMD_WRITE: u8 = 0x32;
pub const CMD_CHIPCONFIG: u8 = 0x33;
pub const CMD_START: u8 = 0x34;
pub const CMD_STOP: u8 = 0x35;
pub const CMD_STATUS: u8 = 0x36;
pub const CMD_HOLDCTL: u8 = 0x37;
pub const CMD_LOGCTL: u8 = 0x38;
pub const CMD_TOCTOU: u8 = 0x39;
pub const CMD_LOGPOLL: u8 = 0x3A;

/// CMD_LOGPOLL response terminator.
pub const LOG_POLL_TERMINATOR: u8 = 0xA0;
/// CMD_LOGPOLL escape marker for payload bytes that collide with framing.
pub const LOG_POLL_ESCAPE: u8 = 0xA5;
/// Log packet type markers.
pub const LOG_CMD: u8 = 0xA1;
pub const LOG_ADDR: u8 = 0xA2;
pub const LOG_END: u8 = 0xA3;
pub const LOG_TRAP: u8 = 0xA4;

/// TOCTOU sub-commands.
pub const TOCTOU_SET: u8 = 0x01;
pub const TOCTOU_ARM: u8 = 0x02;
pub const TOCTOU_DISARM: u8 = 0x03;
pub const TOCTOU_RESET: u8 = 0x04;
pub const TOCTOU_RESET_ALL: u8 = 0x05;

/// Return whether an opcode is a read-like SPI command.
pub fn is_read_opcode(opcode: u8) -> bool {
    const READ_OPCODES: &[u8] = &[
        0x03, 0x0B, 0x0C, 0x13, 0x3B, 0x3C, 0x5A, 0x6B, 0x6C, 0xBB, 0xBC, 0xEB, 0xEC,
    ];
    READ_OPCODES.contains(&opcode)
}

/// Return a human-readable SPI opcode name.
pub fn spi_opcode_name(opcode: u8) -> &'static str {
    match opcode {
        0x02 => "PAGE_PROGRAM",
        0x03 => "READ",
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
        0x5C => "BLOCK_ERASE_32K_4B",
        0x5A => "READ_SFDP",
        0x60 => "CHIP_ERASE",
        0x6B => "QUAD_READ",
        0x6C => "QUAD_READ_4B",
        0x9E | 0x9F => "READ_JEDEC_ID",
        0xB7 => "4BYTE_ENABLE",
        0xBB => "DUAL_IO_READ",
        0xBC => "DUAL_IO_READ_4B",
        0xC7 => "CHIP_ERASE",
        0xD8 => "BLOCK_ERASE_64K",
        0xDC => "BLOCK_ERASE_64K_4B",
        0xE9 => "4BYTE_DISABLE",
        0xEB => "QUAD_IO_READ",
        0xEC => "QUAD_IO_READ_4B",
        _ => "UNKNOWN",
    }
}

/// Validate an aligned block read/write transfer.
pub fn validate_block(
    address: u32,
    length: usize,
    max_len: usize,
    kind: &str,
) -> Result<(), String> {
    if length == 0 || length > max_len {
        return Err(format!("invalid {kind} length: must be 1-{max_len}"));
    }
    if !address.is_multiple_of(8) {
        return Err(format!("{kind} address must be 8-byte aligned"));
    }
    if !length.is_multiple_of(8) {
        return Err(format!("{kind} length must be a multiple of 8"));
    }
    let end = (address as u64)
        .checked_add(length as u64)
        .ok_or_else(|| format!("{kind} range overflows address arithmetic"))?;
    if end > SDRAM_SIZE_BYTES {
        return Err(format!(
            "{kind} range 0x{address:08x}..0x{end:08x} exceeds {} MiB SDRAM backing store",
            SDRAM_SIZE_BYTES / (1024 * 1024)
        ));
    }
    Ok(())
}

/// Build a CMD_READ packet for an aligned block.
pub fn read_command(address: u32, length: usize) -> Result<[u8; 6], String> {
    validate_block(address, length, BLOCK_SIZE, "read")?;
    let addr_units = address / 8;
    let len_units = length / 8;
    Ok([
        CMD_READ,
        ((addr_units >> 16) & 0xFF) as u8,
        ((addr_units >> 8) & 0xFF) as u8,
        (addr_units & 0xFF) as u8,
        ((len_units >> 8) & 0xFF) as u8,
        (len_units & 0xFF) as u8,
    ])
}

/// Return a copy padded to an 8-byte write boundary with erased bytes.
pub fn padded_write_data(data: &[u8]) -> Vec<u8> {
    let mut padded = data.to_vec();
    if !padded.len().is_multiple_of(8) {
        padded.resize(padded.len().div_ceil(8) * 8, 0xFF);
    }
    padded
}

/// Build a CMD_WRITE packet and payload for an aligned block.
pub fn write_command(address: u32, data: &[u8]) -> Result<Vec<u8>, String> {
    validate_block(address, data.len(), BLOCK_SIZE, "write")?;
    let addr_units = address / 8;
    let len_units = data.len() / 8;
    let mut buf = Vec::with_capacity(6 + data.len());
    buf.extend_from_slice(&[
        CMD_WRITE,
        ((addr_units >> 16) & 0xFF) as u8,
        ((addr_units >> 8) & 0xFF) as u8,
        (addr_units & 0xFF) as u8,
        ((len_units >> 8) & 0xFF) as u8,
        (len_units & 0xFF) as u8,
    ]);
    buf.extend_from_slice(data);
    Ok(buf)
}

/// Build a CMD_CHIPCONFIG packet for a chip.
pub fn chip_config_command(chip: &FlashChip) -> Result<Vec<u8>, String> {
    let sfdp_table = generate_sfdp(chip);
    let sfdp_len = sfdp_table.len();
    if sfdp_len > 128 {
        return Err(format!("SFDP table too large: {sfdp_len} bytes (max 128)"));
    }
    if chip.total_size as u64 > SDRAM_SIZE_BYTES {
        return Err(format!(
            "chip size {} bytes exceeds {} MiB SDRAM backing store",
            chip.total_size,
            SDRAM_SIZE_BYTES / (1024 * 1024)
        ));
    }

    let jedec = chip.jedec_id_bytes();
    let erase_bursts = chip.chip_erase_bursts();
    let mut buf = Vec::with_capacity(9 + sfdp_len);
    buf.push(CMD_CHIPCONFIG);
    buf.push(jedec[0]);
    buf.push(jedec[1]);
    buf.push(jedec[2]);
    buf.push(u8::from(chip.supports_4byte));
    buf.push(((erase_bursts >> 16) & 0x7F) as u8);
    buf.push(((erase_bursts >> 8) & 0xFF) as u8);
    buf.push((erase_bursts & 0xFF) as u8);
    buf.push(sfdp_len as u8);
    buf.extend_from_slice(&sfdp_table);
    Ok(buf)
}

/// Build a HOLD control packet.
pub fn hold_command(enable: bool) -> [u8; 2] {
    [CMD_HOLDCTL, u8::from(enable)]
}

/// Build a LOGCTL packet.
pub fn logctl_command(enable: bool) -> [u8; 2] {
    [CMD_LOGCTL, u8::from(enable)]
}

/// Build a TOCTOU set packet.
pub fn toctou_set_command(index: u8, start: u32, mask: u32, replace: u32) -> [u8; 12] {
    [
        CMD_TOCTOU,
        TOCTOU_SET,
        index & 0x03,
        ((start >> 16) & 0xFF) as u8,
        ((start >> 8) & 0xFF) as u8,
        (start & 0xFF) as u8,
        ((mask >> 16) & 0xFF) as u8,
        ((mask >> 8) & 0xFF) as u8,
        (mask & 0xFF) as u8,
        ((replace >> 16) & 0xFF) as u8,
        ((replace >> 8) & 0xFF) as u8,
        (replace & 0xFF) as u8,
    ]
}

/// Build a TOCTOU one-index packet.
pub fn toctou_index_command(op: u8, index: u8) -> [u8; 3] {
    [CMD_TOCTOU, op, index & 0x03]
}

/// Parse an address/length value in decimal or `0x` hexadecimal form.
pub fn parse_u32(s: &str) -> Result<u32, String> {
    let s = s.trim();
    if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u32::from_str_radix(hex, 16).map_err(|e| e.to_string())
    } else {
        s.parse::<u32>().map_err(|e| e.to_string())
    }
}

/// Parse a byte string such as `deadbeef` or `de ad be ef`.
pub fn parse_hex_bytes(s: &str) -> Result<Vec<u8>, String> {
    let clean: String = s
        .chars()
        .filter(|c| !c.is_whitespace() && *c != '_')
        .collect();
    if !clean.len().is_multiple_of(2) {
        return Err("hex string must have an even number of digits".to_string());
    }
    (0..clean.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&clean[i..i + 2], 16).map_err(|e| e.to_string()))
        .collect()
}

/// Render a compact hex dump.
pub fn hexdump(base: u32, data: &[u8]) -> String {
    let mut out = String::new();
    for (line, chunk) in data.chunks(16).enumerate() {
        out.push_str(&format!("{:08X}: ", base + (line * 16) as u32));
        for byte in chunk {
            out.push_str(&format!("{byte:02X} "));
        }
        out.push('\n');
    }
    out
}

/// Structured events decoded from logger packets.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LogEvent {
    /// A new SPI transaction command.
    Command { txn: u32, opcode: u8 },
    /// An address was captured.
    Address {
        txn: u32,
        opcode: u8,
        address: u32,
        read_count: Option<u32>,
        double_read: bool,
    },
    /// A transfer ended with a byte count.
    End { address: u32, byte_count: u32 },
    /// A TOCTOU trap fired.
    Trap { index: u8, address: u32 },
}

/// Stateful parser for the FPGA SPI logger stream.
#[derive(Default)]
pub struct LogParser {
    pending: Vec<u8>,
    txn_count: u32,
    current_opcode: u8,
    current_addr: u32,
    access_map: std::collections::HashMap<(u32, u8), u32>,
    double_reads: Vec<(u32, u8)>,
}

impl LogParser {
    /// Push raw logger bytes and return all complete decoded events.
    pub fn push(&mut self, data: &[u8]) -> Vec<LogEvent> {
        let mut events = Vec::new();
        self.pending.extend_from_slice(data);
        let mut pos = 0;
        while pos < self.pending.len() {
            let remaining = self.pending.len() - pos;
            match self.pending[pos] {
                LOG_CMD if remaining >= 2 => {
                    let opcode = self.pending[pos + 1];
                    self.current_opcode = opcode;
                    self.txn_count += 1;
                    events.push(LogEvent::Command {
                        txn: self.txn_count,
                        opcode,
                    });
                    pos += 2;
                }
                LOG_ADDR if remaining >= 5 => {
                    let addr = ((self.pending[pos + 1] as u32) << 24)
                        | ((self.pending[pos + 2] as u32) << 16)
                        | ((self.pending[pos + 3] as u32) << 8)
                        | self.pending[pos + 4] as u32;
                    self.current_addr = addr;
                    let mut read_count = None;
                    let mut double_read = false;
                    if is_read_opcode(self.current_opcode) {
                        let key = (addr, self.current_opcode);
                        let count = self.access_map.entry(key).or_insert(0);
                        *count += 1;
                        read_count = Some(*count);
                        if *count == 2 {
                            self.double_reads.push(key);
                            double_read = true;
                        }
                    }
                    events.push(LogEvent::Address {
                        txn: self.txn_count,
                        opcode: self.current_opcode,
                        address: addr,
                        read_count,
                        double_read,
                    });
                    pos += 5;
                }
                LOG_END if remaining >= 4 => {
                    let count = ((self.pending[pos + 1] as u32) << 16)
                        | ((self.pending[pos + 2] as u32) << 8)
                        | self.pending[pos + 3] as u32;
                    events.push(LogEvent::End {
                        address: self.current_addr,
                        byte_count: count + 1,
                    });
                    pos += 4;
                }
                LOG_TRAP if remaining >= 6 => {
                    let index = self.pending[pos + 1];
                    let addr = ((self.pending[pos + 2] as u32) << 16)
                        | ((self.pending[pos + 3] as u32) << 8)
                        | self.pending[pos + 4] as u32;
                    events.push(LogEvent::Trap {
                        index,
                        address: addr,
                    });
                    pos += 6;
                }
                0xA1..=0xAF => break,
                _ => pos += 1,
            }
        }
        self.pending.drain(..pos);
        events
    }

    /// Return addresses that were read more than once and their total counts.
    pub fn double_reads(&self) -> Vec<(u32, u8, u32)> {
        self.double_reads
            .iter()
            .map(|(addr, opcode)| {
                let count = self.access_map.get(&(*addr, *opcode)).copied().unwrap_or(0);
                (*addr, *opcode, count)
            })
            .collect()
    }
}

/// Format a parser summary like the CLI monitor output.
pub fn format_log_summary(parser: &LogParser) -> String {
    let double_reads = parser.double_reads();
    if double_reads.is_empty() {
        "\nNo double-reads detected.\n".to_string()
    } else {
        let mut out = format!(
            "\n--- TOCTOU Detection Summary ---\n{} address(es) read more than once:\n",
            double_reads.len()
        );
        for (addr, opcode, count) in double_reads {
            out.push_str(&format!(
                "  0x{addr:06X} ({}) read {count} times\n",
                spi_opcode_name(opcode)
            ));
        }
        out
    }
}

/// Format log events like the CLI monitor output.
pub fn format_log_event(event: &LogEvent) -> String {
    match event {
        LogEvent::Command { txn, opcode } => {
            format!("{txn:<6} 0x{opcode:02X} {:<13}", spi_opcode_name(*opcode))
        }
        LogEvent::Address {
            address,
            read_count,
            double_read,
            ..
        } => {
            let mut out = format!(" 0x{address:06X}");
            if *double_read {
                out.push_str("  ** DOUBLE READ (TOCTOU candidate)");
            } else if let Some(count) = read_count.filter(|count| *count > 2) {
                out.push_str(&format!("  ** READ #{count}"));
            }
            out.push('\n');
            out
        }
        LogEvent::End {
            address,
            byte_count,
        } => {
            if *byte_count > 1 {
                format!("       end: {byte_count} bytes from 0x{address:06X}\n")
            } else {
                String::new()
            }
        }
        LogEvent::Trap { index, address } => {
            format!(
                "  !! TOCTOU TRAP #{index} FIRED at 0x{address:06X} -- serving replacement data\n"
            )
        }
    }
}
