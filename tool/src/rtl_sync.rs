//! Checks that the host tool agrees with the constants in the RTL headers.
//!
//! `src/host_protocol.vh` and `src/spi_flash_cmds.vh` are the single source
//! for the wire protocol and the emulated SPI commands; these tests fail when
//! `protocol.rs`, `spi_log.rs` or the generated SFDP table drift from them.

use crate::chip::{ChipDatabase, Features, FlashChipExt};
use crate::protocol::*;
use crate::sfdp::generate_sfdp;
use crate::spi_log::{is_read_opcode, opcode_name};
use std::collections::BTreeMap;

const HOST_PROTOCOL_VH: &str = include_str!("../../src/host_protocol.vh");
const SPI_FLASH_CMDS_VH: &str = include_str!("../../src/spi_flash_cmds.vh");

/// Parse `NAME = value` localparams (sized hex like `8'hA0` or decimal).
fn localparams(source: &str) -> BTreeMap<String, u32> {
    let code: String = source
        .lines()
        .map(|line| line.split("//").next().unwrap_or_default())
        .collect::<Vec<_>>()
        .join("\n");
    code.split([',', ';'])
        .filter_map(|item| {
            let (lhs, rhs) = item.split_once('=')?;
            let name = lhs.split_whitespace().last()?.to_owned();
            let value = rhs.trim();
            let value = match value.split_once('\'') {
                Some((_, based)) => match based.split_at(1) {
                    ("h", digits) => u32::from_str_radix(digits, 16).ok()?,
                    ("d", digits) => digits.parse().ok()?,
                    _ => return None,
                },
                None => value.parse().ok()?,
            };
            Some((name, value))
        })
        .collect()
}

fn spi_opcodes() -> BTreeMap<String, u8> {
    localparams(SPI_FLASH_CMDS_VH)
        .into_iter()
        .filter(|(name, _)| name.starts_with("CMD_"))
        .map(|(name, value)| (name, value as u8))
        .collect()
}

#[test]
fn host_protocol_matches_rtl() {
    let expected: BTreeMap<&str, u8> = [
        ("VERSION", PROTOCOL_VERSION),
        ("CMD_NOP", 0x00), // Ignored by the FPGA; never sent by the host
        ("CMD_VERSION", CMD_VERSION),
        ("CMD_RAMREAD", CMD_READ),
        ("CMD_RAMWRITE", CMD_WRITE),
        ("CMD_CHIPCONFIG", CMD_CHIPCONFIG),
        ("CMD_START", CMD_START),
        ("CMD_STOP", CMD_STOP),
        ("CMD_STATUS", CMD_STATUS),
        ("CMD_HOLDCTL", CMD_HOLDCTL),
        ("CMD_LOGCTL", CMD_LOGCTL),
        ("CMD_TOCTOU", CMD_TOCTOU),
        ("CMD_LOGPOLL", CMD_LOGPOLL),
        ("TOCTOU_SET", TOCTOU_SET),
        ("TOCTOU_ARM", TOCTOU_ARM),
        ("TOCTOU_DISARM", TOCTOU_DISARM),
        ("TOCTOU_RESET", TOCTOU_RESET),
        ("TOCTOU_RESET_ALL", TOCTOU_RESET_ALL),
        ("LOG_POLL_TERMINATOR", LOG_POLL_TERMINATOR),
        ("LOG_POLL_ESCAPE", LOG_POLL_ESCAPE),
        ("LOG_PKT_CMD", LOG_CMD),
        ("LOG_PKT_ADDR", LOG_ADDR),
        ("LOG_PKT_END", LOG_END),
        ("LOG_PKT_TRAP", LOG_TRAP),
    ]
    .into_iter()
    .collect();
    let rtl: BTreeMap<String, u8> = localparams(HOST_PROTOCOL_VH)
        .into_iter()
        .map(|(name, value)| (name, value as u8))
        .collect();

    assert_eq!(
        rtl.keys().map(String::as_str).collect::<Vec<_>>(),
        expected.keys().copied().collect::<Vec<_>>(),
        "host_protocol.vh and protocol.rs define different constants"
    );
    for (name, value) in expected {
        assert_eq!(rtl[name], value, "{name}");
    }
}

#[test]
fn every_spi_opcode_has_a_log_name() {
    let rtl: Vec<u8> = spi_opcodes().into_values().collect();
    let named: Vec<u8> = (0..=u8::MAX)
        .filter(|&opcode| opcode_name(opcode) != "UNKNOWN")
        .collect();
    let mut rtl_sorted = rtl.clone();
    rtl_sorted.sort_unstable();
    assert_eq!(
        named, rtl_sorted,
        "spi_log names differ from spi_flash_cmds.vh"
    );
}

#[test]
fn read_opcodes_match_rtl() {
    for (name, opcode) in spi_opcodes() {
        let array_read = name.contains("READ") && !name.contains("STATUS") && !name.contains("ID");
        assert_eq!(is_read_opcode(opcode), array_read, "{name}");
    }
}

#[test]
fn sfdp_wait_states_match_rtl() {
    let rtl = localparams(SPI_FLASH_CMDS_VH);
    let db = ChipDatabase::new();
    let mut chip = db
        .iter()
        .find(|chip| chip.supports_dual() && chip.supports_quad())
        .expect("a dual/quad chip in the chip database")
        .clone();
    chip.features |= Features::SFDP;
    let table = generate_sfdp(&chip).unwrap();
    let dword = |index: usize| {
        let offset = 0x10 + 4 * index;
        u32::from_le_bytes(table[offset..offset + 4].try_into().unwrap())
    };
    // BFPT DWORD 3/4 halves: [4:0] wait states, [7:5] mode clocks, [15:8] opcode
    let fields = |half: u32| (half & 0x1F, (half >> 5) & 0x7, (half >> 8) & 0xFF);
    let (dw3, dw4) = (dword(2), dword(3));

    assert_eq!(
        fields(dw3),
        (
            rtl["QUAD_IO_DUMMY_CLKS"],
            rtl["QUAD_IO_MODE_CLKS"],
            rtl["CMD_QUADIOREAD"]
        ),
        "1-4-4"
    );
    assert_eq!(
        fields(dw3 >> 16),
        (rtl["FAST_READ_DUMMY_CLKS"], 0, rtl["CMD_QUADREAD"]),
        "1-1-4"
    );
    assert_eq!(
        fields(dw4),
        (0, rtl["DUAL_IO_MODE_CLKS"], rtl["CMD_DUALIOREAD"]),
        "1-2-2"
    );
    assert_eq!(
        fields(dw4 >> 16),
        (rtl["FAST_READ_DUMMY_CLKS"], 0, rtl["CMD_DUALREAD"]),
        "1-1-2"
    );
}

#[test]
fn localparams_parses_lists_and_comments() {
    let params =
        localparams("localparam\n  A = 8'hA0, // x = 1\n  B = 8'd7;\nlocalparam [7:0] C = 12;\n");
    assert_eq!(
        params.into_iter().collect::<Vec<_>>(),
        [("A".into(), 0xA0), ("B".into(), 7), ("C".into(), 12)]
    );
}
