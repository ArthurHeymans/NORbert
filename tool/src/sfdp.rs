//! SFDP (Serial Flash Discoverable Parameters) table generation.
//!
//! Generates a JESD216B-compatible SFDP table from a chip definition.
//! Only populates fields relevant for read, erase, and write operations.
//!
//! Table layout (80 bytes total):
//!   0x00..0x07  SFDP Header (8 bytes)
//!   0x08..0x0F  Parameter Header 0 / BFPT (8 bytes)
//!   0x10..0x4F  Basic Flash Parameter Table (16 DWORDs = 64 bytes)

use crate::chip::FlashChip;

/// Size of the generated SFDP table in bytes.
pub const SFDP_TABLE_SIZE: usize = 80;

const BFPT_OFFSET: u32 = 0x10;
const BFPT_DWORDS: u8 = 16; // JESD216A/B

/// Generate the full SFDP table for a given chip.
pub fn generate_sfdp(chip: &FlashChip) -> [u8; SFDP_TABLE_SIZE] {
    let mut table = [0xFFu8; SFDP_TABLE_SIZE];

    write_sfdp_header(&mut table);
    write_param_header(&mut table);
    write_bfpt(chip, &mut table);

    table
}

/// SFDP Header (8 bytes at offset 0x00).
fn write_sfdp_header(table: &mut [u8; SFDP_TABLE_SIZE]) {
    // Signature "SFDP"
    table[0] = 0x53; // 'S'
    table[1] = 0x46; // 'F'
    table[2] = 0x44; // 'D'
    table[3] = 0x50; // 'P'
                     // SFDP revision 1.6 (JESD216B)
    table[4] = 0x06; // Minor revision
    table[5] = 0x01; // Major revision
                     // Number of parameter headers minus 1 (just BFPT)
    table[6] = 0x00;
    table[7] = 0xFF; // Unused
}

/// Parameter Header 0 for BFPT (8 bytes at offset 0x08).
fn write_param_header(table: &mut [u8; SFDP_TABLE_SIZE]) {
    table[0x08] = 0x00; // Parameter ID LSB (BFPT = 0xFF00)
    table[0x09] = 0x06; // Parameter minor revision
    table[0x0A] = BFPT_DWORDS; // Length in DWORDs
    table[0x0B] = 0x01; // Parameter major revision
                        // Table pointer (24-bit little-endian)
    table[0x0C] = BFPT_OFFSET as u8;
    table[0x0D] = (BFPT_OFFSET >> 8) as u8;
    table[0x0E] = (BFPT_OFFSET >> 16) as u8;
    table[0x0F] = 0xFF; // Parameter ID MSB
}

/// Write a little-endian DWORD at a byte offset into the table.
fn put_dword(table: &mut [u8; SFDP_TABLE_SIZE], offset: usize, value: u32) {
    table[offset] = value as u8;
    table[offset + 1] = (value >> 8) as u8;
    table[offset + 2] = (value >> 16) as u8;
    table[offset + 3] = (value >> 24) as u8;
}

/// Write the Basic Flash Parameter Table (16 DWORDs starting at offset 0x10).
fn write_bfpt(chip: &FlashChip, table: &mut [u8; SFDP_TABLE_SIZE]) {
    let base = BFPT_OFFSET as usize;

    // ------------------------------------------------------------------
    // 1st DWORD: erase granularity, write info, address bytes, read modes
    // ------------------------------------------------------------------
    let has_4k_erase = chip.erase_ops.iter().any(|e| e.block_size == 4096);
    let erase_4k_opcode = chip
        .erase_ops
        .iter()
        .find(|e| e.block_size == 4096)
        .map(|e| e.opcode)
        .unwrap_or(0xFF);

    let mut dw1: u32 = 0;
    // [1:0] Block/Sector erase granularity
    dw1 |= if has_4k_erase { 0x01 } else { 0x03 };
    // [2] Write granularity: 1 = buffer >= 64 bytes (page_size is typically 256)
    if chip.page_size >= 64 {
        dw1 |= 1 << 2;
    }
    // [3] WREN required for volatile SR write
    dw1 |= 1 << 3;
    // [4] WREN opcode: 1 = use 0x06 (standard)
    dw1 |= 1 << 4;
    // [7:5] Reserved = 111
    dw1 |= 0x07 << 5;
    // [15:8] 4KB erase opcode
    dw1 |= (erase_4k_opcode as u32) << 8;
    // [17:16] Address bytes: 00=3-byte only, 01=3 or 4 byte
    if chip.supports_4byte {
        dw1 |= 0x01 << 16;
    }
    // [18] DTR: not supported
    // [19] 1-1-2 Fast Read
    if chip.supports_dual {
        dw1 |= 1 << 19;
    }
    // [20] 1-2-2 Fast Read
    if chip.supports_dual {
        dw1 |= 1 << 20;
    }
    // [21] 1-4-4 Fast Read
    if chip.supports_quad {
        dw1 |= 1 << 21;
    }
    // [22] 1-1-4 Fast Read
    if chip.supports_quad {
        dw1 |= 1 << 22;
    }
    put_dword(table, base, dw1);

    // ------------------------------------------------------------------
    // 2nd DWORD: Flash memory density (in bits, minus 1)
    // ------------------------------------------------------------------
    let density_bits = (chip.total_size as u64) * 8;
    // For densities up to 2 Gbit, bit 31 = 0 and bits[30:0] = density - 1
    let dw2 = if density_bits <= 0x80000000 {
        (density_bits - 1) as u32
    } else {
        // For densities > 2 Gbit: bit 31 = 1, bits[30:0] = N where 2^N = density
        0x80000000 | (density_bits.trailing_zeros())
    };
    put_dword(table, base + 4, dw2);

    // ------------------------------------------------------------------
    // 3rd DWORD: 1-4-4 and 1-1-4 Fast Read parameters
    // ------------------------------------------------------------------
    let dw3 = if chip.supports_quad {
        // 1-4-4 (0xEB): 4 dummy clocks after 2 mode clocks
        let wait_144: u32 = 4; // dummy clocks
        let mode_144: u32 = 2; // mode clocks
        let op_144: u32 = 0xEB;
        // 1-1-4 (0x6B): 8 dummy clocks, 0 mode clocks
        let wait_114: u32 = 8;
        let mode_114: u32 = 0;
        let op_114: u32 = 0x6B;
        (wait_144 & 0x1F)
            | ((mode_144 & 0x07) << 5)
            | (op_144 << 8)
            | ((wait_114 & 0x1F) << 16)
            | ((mode_114 & 0x07) << 21)
            | (op_114 << 24)
    } else {
        0
    };
    put_dword(table, base + 8, dw3);

    // ------------------------------------------------------------------
    // 4th DWORD: 1-2-2 and 1-1-2 Fast Read parameters
    // ------------------------------------------------------------------
    let dw4 = if chip.supports_dual {
        // 1-2-2 (0xBB): 0 dummy clocks, 4 mode clocks
        let wait_122: u32 = 0;
        let mode_122: u32 = 4;
        let op_122: u32 = 0xBB;
        // 1-1-2 (0x3B): 8 dummy clocks, 0 mode clocks
        let wait_112: u32 = 8;
        let mode_112: u32 = 0;
        let op_112: u32 = 0x3B;
        (wait_122 & 0x1F)
            | ((mode_122 & 0x07) << 5)
            | (op_122 << 8)
            | ((wait_112 & 0x1F) << 16)
            | ((mode_112 & 0x07) << 21)
            | (op_112 << 24)
    } else {
        0
    };
    put_dword(table, base + 12, dw4);

    // ------------------------------------------------------------------
    // 5th DWORD: 2-2-2 and 4-4-4 support (not supported by NORbert)
    // ------------------------------------------------------------------
    put_dword(table, base + 16, 0);

    // ------------------------------------------------------------------
    // 6th-7th DWORDs: Reserved / 2-2-2 / 4-4-4 params (unused)
    // ------------------------------------------------------------------
    put_dword(table, base + 20, 0);
    put_dword(table, base + 24, 0);

    // ------------------------------------------------------------------
    // 8th DWORD: Erase Type 1 & 2
    // 9th DWORD: Erase Type 3 & 4
    // Each type: [7:0] size as 2^N (0=unavailable), [15:8] opcode
    // ------------------------------------------------------------------
    let erase_ops = chip.sector_erase_ops();

    let encode_erase = |idx: usize| -> u16 {
        if let Some(op) = erase_ops.get(idx) {
            let n = op.block_size.trailing_zeros() as u8;
            (n as u16) | ((op.opcode as u16) << 8)
        } else {
            0
        }
    };

    let dw8 = (encode_erase(0) as u32) | ((encode_erase(1) as u32) << 16);
    let dw9 = (encode_erase(2) as u32) | ((encode_erase(3) as u32) << 16);
    put_dword(table, base + 28, dw8);
    put_dword(table, base + 32, dw9);

    // ------------------------------------------------------------------
    // 10th DWORD: Erase type typical times (reasonable defaults)
    //
    // Per-type: 7 bits each = [6:5] unit (00=1ms,01=16ms,10=128ms,11=1s)
    //                         [4:0] count (time = (count+1) * unit)
    // [3:0]  Max erase time multiplier: 2*(val+1) applied to typical
    // [10:4] Type 1 typical time
    // [17:11] Type 2 typical time
    // [24:18] Type 3 typical time
    // [31:25] Type 4 typical time
    // ------------------------------------------------------------------
    let mut dw10: u32 = 0x01; // multiplier = 2*(1+1) = 4x
                              // Type 1 (4KB):  ~45ms  -> 16ms * 3 = 48ms  -> unit=01, count=2
    if erase_ops.len() > 0 {
        dw10 |= ((0x01 << 5) | 2) << 4; // 0x22 << 4
    }
    // Type 2 (32KB): ~120ms -> 16ms * 8 = 128ms -> unit=01, count=7
    if erase_ops.len() > 1 {
        dw10 |= ((0x01 << 5) | 7) << 11; // 0x27 << 11
    }
    // Type 3 (64KB): ~150ms -> 128ms * 2 = 256ms -> unit=10, count=1
    if erase_ops.len() > 2 {
        dw10 |= ((0x02 << 5) | 1) << 18; // 0x41 << 18
    }
    put_dword(table, base + 36, dw10);

    // ------------------------------------------------------------------
    // 11th DWORD: Program/erase timing, page size
    //
    // [3:0]  Chip erase max time multiplier: 2*(val+1)
    // [7:4]  Page size: 2^N bytes
    // [12:8] Page program typical time (encoded)
    // [31:13] Additional timing (set to 0)
    // ------------------------------------------------------------------
    let page_size_n = if chip.page_size > 0 {
        chip.page_size.trailing_zeros()
    } else {
        8 // default 256
    };
    let dw11: u32 = 0x01 // chip erase multiplier = 4x
        | ((page_size_n & 0x0F) << 4) // page size
        | (0x06 << 8); // page program ~64us (unit=01(64us), count=0 -> 64us)
    put_dword(table, base + 40, dw11);

    // ------------------------------------------------------------------
    // 12th DWORD: Suspend/Resume (not relevant for emulator)
    // ------------------------------------------------------------------
    put_dword(table, base + 44, 0);

    // ------------------------------------------------------------------
    // 13th DWORD: Deep power down / status register polling (defaults)
    // ------------------------------------------------------------------
    put_dword(table, base + 48, 0);

    // ------------------------------------------------------------------
    // 14th DWORD: Hold/Reset, Quad Enable requirements
    //
    // [3:0]  Hold/Reset disable (0 = not supported)
    // [6:4]  Quad Enable Requirements:
    //   000 = no QE bit (device has no quad)
    //   001 = QE is bit 1 of SR2, set via 0x01 with 2 data bytes
    //   010 = QE is bit 6 of SR1
    //   011 = QE is bit 7 of SR2, set via 0x3E
    //   100 = QE is bit 1 of SR2, set via 0x31
    //   101 = QE is bit 1 of SR2, set via 0x01 with 2 data bytes (Winbond)
    // [31:7] Reserved
    // ------------------------------------------------------------------
    let dw14 = if chip.supports_quad {
        // Use method 5 (0b101): QE = SR2 bit 1, write via 0x01 with 2 bytes
        // This matches Winbond W25Q and many other common chips
        0x05 << 4
    } else {
        0
    };
    put_dword(table, base + 52, dw14);

    // ------------------------------------------------------------------
    // 15th DWORD: 4-byte address mode entry/exit methods
    //
    // [7:0]  Entry methods (bitfield):
    //   bit 1: issue 0xB7 to enter 4-byte mode
    // [31:8] Exit methods
    // ------------------------------------------------------------------
    let dw15 = if chip.supports_4byte {
        0x02 // Enter via 0xB7 (EN4B)
    } else {
        0
    };
    put_dword(table, base + 56, dw15);

    // ------------------------------------------------------------------
    // 16th DWORD: 4-byte address instruction support (0 = use mode switch)
    // ------------------------------------------------------------------
    put_dword(table, base + 60, 0);
}
