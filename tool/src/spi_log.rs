//! Decoding and rendering of the SPI activity log drained by `CMD_LOGPOLL`.
//!
//! Shared by the CLI `monitor` command and the browser UI so both show the
//! same opcode names, packet decoding and double-read (TOCTOU) detection.

use crate::protocol::{LOG_ADDR, LOG_CMD, LOG_END, LOG_TRAP};
use std::collections::HashMap;
use std::fmt::Write;

/// Column header matching the rows produced by [`ActivityLog::feed`].
pub const HEADER: &str = "TXN#   COMMAND            ADDRESS    INFO";

/// Human-readable name of an SPI flash opcode decoded by `spi_trx.v`.
pub fn opcode_name(opcode: u8) -> &'static str {
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
        0x50 => "EWSR",
        0x52 => "BLOCK_ERASE_32K",
        0x5A => "READ_SFDP",
        0x5C => "BLOCK_ERASE_32K_4B",
        0x60 | 0xC7 => "CHIP_ERASE",
        0x6B => "QUAD_READ",
        0x6C => "QUAD_READ_4B",
        0x9E | 0x9F => "READ_JEDEC_ID",
        0xAD => "AAI_WORD_PROGRAM",
        0xB7 => "4BYTE_ENABLE",
        0xBB => "DUAL_IO_READ",
        0xBC => "DUAL_IO_READ_4B",
        0xD8 => "BLOCK_ERASE_64K",
        0xDC => "BLOCK_ERASE_64K_4B",
        0xE9 => "4BYTE_DISABLE",
        0xEB => "QUAD_IO_READ",
        0xEC => "QUAD_IO_READ_4B",
        0xF2 => "LOG",
        _ => "UNKNOWN",
    }
}

/// Opcodes that read the flash array or SFDP table.
pub fn is_read_opcode(opcode: u8) -> bool {
    matches!(
        opcode,
        0x03 | 0x0B | 0x0C | 0x13 | 0x3B | 0x3C | 0x5A | 0x6B | 0x6C | 0xBB | 0xBC | 0xEB | 0xEC
    )
}

/// One decoded log packet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LogEvent {
    Command(u8),
    Address(u32),
    End { bytes: u32 },
    Trap { index: u8, address: u32 },
}

/// Reassembles log packets from arbitrarily split poll responses.
#[derive(Default)]
pub struct LogDecoder {
    pending: Vec<u8>,
}

impl LogDecoder {
    pub fn feed(&mut self, data: &[u8]) -> Vec<LogEvent> {
        self.pending.extend_from_slice(data);
        let mut events = Vec::new();
        let mut pos = 0;
        while let Some(&kind) = self.pending.get(pos) {
            let packet = &self.pending[pos..];
            let (event, len) = match kind {
                LOG_CMD if packet.len() >= 2 => (Some(LogEvent::Command(packet[1])), 2),
                LOG_ADDR if packet.len() >= 5 => (
                    Some(LogEvent::Address(u32::from_be_bytes([
                        packet[1], packet[2], packet[3], packet[4],
                    ]))),
                    5,
                ),
                LOG_END if packet.len() >= 4 => (
                    Some(LogEvent::End {
                        bytes: u32::from_be_bytes([0, packet[1], packet[2], packet[3]]),
                    }),
                    4,
                ),
                // type(1) + index(1) + addr(3) + pad(1)
                LOG_TRAP if packet.len() >= 6 => (
                    Some(LogEvent::Trap {
                        index: packet[1],
                        address: u32::from_be_bytes([0, packet[2], packet[3], packet[4]]),
                    }),
                    6,
                ),
                // Known packet type but incomplete: wait for more data.
                0xA1..=0xAF => break,
                // Unknown byte: resynchronize on the next one.
                _ => (None, 1),
            };
            events.extend(event);
            pos += len;
        }
        self.pending.drain(..pos);
        events
    }
}

/// Renders log packets as text rows and tracks repeated reads.
#[derive(Default)]
pub struct ActivityLog {
    decoder: LogDecoder,
    txn_count: u32,
    opcode: u8,
    address: u32,
    line_open: bool,
    reads: HashMap<(u32, u8), u32>,
    double_reads: Vec<(u32, u8)>,
}

impl ActivityLog {
    /// Decode `data` and return the text it adds to the log. A command row
    /// stays open (no trailing newline) until its address or end arrives.
    pub fn feed(&mut self, data: &[u8]) -> String {
        let mut out = String::new();
        for event in self.decoder.feed(data) {
            self.render(event, &mut out);
        }
        out
    }

    /// `(address, opcode, read count)` for every read seen more than once,
    /// in order of first repetition.
    pub fn double_reads(&self) -> impl Iterator<Item = (u32, u8, u32)> + '_ {
        self.double_reads
            .iter()
            .map(|&(address, opcode)| (address, opcode, self.reads[&(address, opcode)]))
    }

    fn close_line(&mut self, out: &mut String) {
        if self.line_open {
            out.push('\n');
            self.line_open = false;
        }
    }

    fn render(&mut self, event: LogEvent, out: &mut String) {
        match event {
            LogEvent::Command(opcode) => {
                self.close_line(out);
                self.opcode = opcode;
                self.txn_count += 1;
                let _ = write!(
                    out,
                    "{:<6} 0x{:02X} {:<13}",
                    self.txn_count,
                    opcode,
                    opcode_name(opcode)
                );
                self.line_open = true;
            }
            LogEvent::Address(address) => {
                self.address = address;
                let _ = write!(out, " {}", format_address(address));
                if is_read_opcode(self.opcode) {
                    let key = (address, self.opcode);
                    let count = self.reads.entry(key).or_insert(0);
                    *count += 1;
                    match *count {
                        2 => {
                            self.double_reads.push(key);
                            out.push_str("  ** DOUBLE READ (TOCTOU candidate)");
                        }
                        n if n > 2 => {
                            let _ = write!(out, "  ** READ #{n}");
                        }
                        _ => {}
                    }
                }
                out.push('\n');
                self.line_open = false;
            }
            LogEvent::End { bytes } => {
                // Addressless commands get no LOG_ADDR packet, so terminate
                // their row when the transaction ends.
                self.close_line(out);
                if bytes > 1 {
                    let _ = writeln!(
                        out,
                        "       end: {bytes} bytes from {}",
                        format_address(self.address)
                    );
                }
            }
            LogEvent::Trap { index, address } => {
                self.close_line(out);
                let _ = writeln!(
                    out,
                    "  !! TOCTOU TRAP #{index} FIRED at 0x{address:06X} -- serving replacement data"
                );
            }
        }
    }
}

fn format_address(address: u32) -> String {
    if address > 0xFF_FFFF {
        format!("0x{address:08X}")
    } else {
        format!("0x{address:06X}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decoder_reassembles_split_packets_and_skips_noise() {
        let mut decoder = LogDecoder::default();
        assert_eq!(decoder.feed(&[0x00, LOG_CMD]), []);
        assert_eq!(
            decoder.feed(&[0x03, LOG_ADDR, 0x00, 0x00, 0x10]),
            [LogEvent::Command(0x03)]
        );
        assert_eq!(
            decoder.feed(&[
                0x00, LOG_END, 0x00, 0x10, 0x01, LOG_TRAP, 2, 0x12, 0x34, 0x56, 0
            ]),
            [
                LogEvent::Address(0x1000),
                LogEvent::End { bytes: 0x1001 },
                LogEvent::Trap {
                    index: 2,
                    address: 0x12_3456
                },
            ]
        );
    }

    #[test]
    fn activity_log_renders_rows_and_flags_double_reads() {
        let read = [
            LOG_CMD, 0xBB, LOG_ADDR, 0, 0, 0x10, 0x00, LOG_END, 0, 0x10, 0x01,
        ];
        let mut log = ActivityLog::default();
        assert_eq!(log.feed(&[LOG_CMD, 0x9F, LOG_END, 0, 0, 3]), {
            "1      0x9F READ_JEDEC_ID\n       end: 3 bytes from 0x000000\n"
        });
        assert_eq!(
            log.feed(&read),
            "2      0xBB DUAL_IO_READ  0x001000\n       end: 4097 bytes from 0x001000\n"
        );
        assert!(log.feed(&read).contains("DOUBLE READ"));
        assert!(log.feed(&read).contains("READ #3"));
        assert_eq!(log.double_reads().collect::<Vec<_>>(), [(0x1000, 0xBB, 3)]);
    }
}
