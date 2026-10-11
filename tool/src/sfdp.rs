//! SFDP (Serial Flash Discoverable Parameters) table generation.
//!
//! Generates a JESD216B-compatible SFDP table from a chip definition.
//! Only populates fields relevant for read, erase, and write operations.
//!
//! Table layout (80 bytes total):
//!   0x00..0x07  SFDP Header (8 bytes)
//!   0x08..0x0F  Parameter Header 0 / BFPT (8 bytes)
//!   0x10..0x4F  Basic Flash Parameter Table (16 DWORDs = 64 bytes)

use crate::chip::{Features, FlashChip, FlashChipExt, erase_block_size};
use crate::profile::{ChipProfile, ChipState};
use anyhow::{Result, bail};

/// Size of the generated SFDP table in bytes.
pub const SFDP_TABLE_SIZE: usize = 80;

const BFPT_OFFSET: u32 = 0x10;
const BFPT_DWORDS: u8 = 16; // JESD216A/B

/// Generate the full SFDP table for a given chip.
///
/// Parts without explicit or known SFDP support, or with nonuniform addressed erasers,
/// get an all-0xFF table (see [`FlashChipExt::supports_sfdp`]).  A valid SFDP response would mislead
/// tools like flashprog that probe SFDP before consulting their chip
/// table -- they would parse a fabricated signature and potentially
/// override the correct hardcoded behavior.  Returning 0xFF (erased
/// flash pattern, invalid "SFDP" signature) makes the probe fail as
/// it would on the real part.
pub fn generate_sfdp(chip: &FlashChip) -> Result<[u8; SFDP_TABLE_SIZE]> {
    let mut table = [0xFFu8; SFDP_TABLE_SIZE];

    if !chip.supports_sfdp() {
        return Ok(table);
    }
    if chip.total_size == 0 {
        bail!("chip total size must be nonzero");
    }
    let density_bits = (chip.total_size as u64) * 8;
    if density_bits > 0x8000_0000 && !density_bits.is_power_of_two() {
        bail!("chip densities above 2 Gbit must be a power of two");
    }
    if chip.page_size == 0 || !chip.page_size.is_power_of_two() {
        bail!("chip page size must be a nonzero power of two");
    }
    for erase in chip.sector_erase_ops() {
        if let Some(size) = erase_block_size(erase)
            && (size == 0 || !size.is_power_of_two())
        {
            bail!("erase size {size} must be a nonzero power of two");
        }
    }

    // A valid signature must never advertise geometry or framing that
    // controller-compatible configuration cannot represent.
    ChipProfile::build(chip, ChipState::default())?;
    if chip.sector_erase_ops().len() > 4 {
        bail!("SFDP supports at most four erase types");
    }
    write_sfdp_header(&mut table);
    write_param_header(&mut table);
    write_bfpt(chip, &mut table);

    Ok(table)
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
    table[0x0A] = 0x01; // Parameter major revision
    table[0x0B] = BFPT_DWORDS; // Length in DWORDs
    // Table pointer (24-bit little-endian)
    table[0x0C] = BFPT_OFFSET as u8;
    table[0x0D] = (BFPT_OFFSET >> 8) as u8;
    table[0x0E] = (BFPT_OFFSET >> 16) as u8;
    table[0x0F] = 0xFF; // Parameter ID MSB
}

/// Write a little-endian DWORD at a byte offset into the table.
fn put_dword(table: &mut [u8; SFDP_TABLE_SIZE], offset: usize, value: u32) {
    let bytes = value.to_le_bytes();
    table[offset..offset + bytes.len()].copy_from_slice(&bytes);
}

/// Write the Basic Flash Parameter Table (16 DWORDs starting at offset 0x10).
fn write_bfpt(chip: &FlashChip, table: &mut [u8; SFDP_TABLE_SIZE]) {
    let base = BFPT_OFFSET as usize;

    // ------------------------------------------------------------------
    // 1st DWORD: erase granularity, write info, address bytes, read modes
    // ------------------------------------------------------------------
    let has_4k_erase = chip
        .erase_blocks
        .iter()
        .any(|erase| !erase.is_chip_erase() && erase_block_size(erase) == Some(4096));
    let erase_4k_opcode = chip
        .erase_blocks
        .iter()
        .find(|erase| !erase.is_chip_erase() && erase_block_size(erase) == Some(4096))
        .map(|erase| erase.opcode)
        .unwrap_or(0xFF);

    let mut dw1: u32 = 0;
    // [1:0] Block/Sector erase granularity
    dw1 |= if has_4k_erase { 0x01 } else { 0x03 };
    // [2] Write granularity: 1 = buffer >= 64 bytes, 0 = 1 byte.
    //
    // Per JESD216B, this selects between page program (>= 64 byte
    // buffer) and byte program (1 byte).  There is no SFDP flag for
    // AAI word program, so chips whose native multi-byte write is AAI
    // (e.g. SST25VFxxx: AAI writes 2 bytes, byte program writes 1) or
    // that only support 0x02 as a single-byte write must advertise 0
    // here.  Tools that rely solely on SFDP then fall back to byte
    // programming via 0x02; tools with a hardcoded chip table (e.g.
    // flashprog) will use their own AAI path regardless.
    let large_buffer = chip.page_size >= 64 && !chip.aai_word() && !chip.write_byte();
    if large_buffer {
        dw1 |= 1 << 2;
    }
    // [3] SR is volatile-only; [4] its write-enable mechanism is WREN.
    if chip.features.contains(Features::WRSR_VOLATILE_WREN) {
        dw1 |= (1 << 3) | (1 << 4);
    } else if chip.features.contains(Features::WRSR_EWSR)
        && !chip.features.contains(Features::WRSR_WREN)
    {
        dw1 |= 1 << 3;
    }
    // [7:5] Reserved = 111
    dw1 |= 0x07 << 5;
    // [15:8] 4KB erase opcode
    dw1 |= (erase_4k_opcode as u32) << 8;
    // [18:17] Address bytes: 00=3-byte only, 01=3 or 4 byte
    if chip.features.supports_4ba_mode_switch() {
        dw1 |= 0x01 << 17;
    }
    // [19] DTR: not supported
    // [16] 1-1-2 Fast Read
    if chip.supports_dual() {
        dw1 |= 1 << 16;
    }
    // [20] 1-2-2 Fast Read
    if chip.supports_dual() {
        dw1 |= 1 << 20;
    }
    // [21] 1-4-4 Fast Read
    if chip.supports_quad() {
        dw1 |= 1 << 21;
    }
    // [22] 1-1-4 Fast Read
    if chip.supports_quad() {
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
    let dw3 = if chip.supports_quad() {
        // Static default 1-4-4 (EB): Micron uses 10 dummy/no mode clocks;
        // other families (including Macronix Config=0) use 4 dummy+2 mode.
        let (wait_144, mode_144): (u32, u32) = if chip.jedec_manufacturer == 0x20 {
            (10, 0)
        } else {
            (4, 2)
        };
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
    // 4th DWORD: 1-1-2 (low) and 1-2-2 (high) Fast Read parameters
    // ------------------------------------------------------------------
    let dw4 = if chip.supports_dual() {
        // Static default 1-2-2 (BB): Micron uses 8 dummy/no mode clocks;
        // other families use 4 mode/no dummy clocks.
        let (wait_122, mode_122): (u32, u32) = if chip.jedec_manufacturer == 0x20 {
            (8, 0)
        } else {
            (0, 4)
        };
        let op_122: u32 = 0xBB;
        // 1-1-2 (0x3B): 8 dummy clocks, 0 mode clocks
        let wait_112: u32 = 8;
        let mode_112: u32 = 0;
        let op_112: u32 = 0x3B;
        (wait_112 & 0x1F)
            | ((mode_112 & 0x07) << 5)
            | (op_112 << 8)
            | ((wait_122 & 0x1F) << 16)
            | ((mode_122 & 0x07) << 21)
            | (op_122 << 24)
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
        if let Some(op) = erase_ops.get(idx)
            && let Some(block_size) = erase_block_size(op)
        {
            let n = block_size.trailing_zeros() as u8;
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
    if !erase_ops.is_empty() {
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

    // DWORD14 does not contain QER. No HOLD/reset policy is advertised.
    put_dword(table, base + 52, 0);

    // DWORD15[22:20]: QER=0. The compatibility engine accepts SDR quad
    // commands without gating on a physical chip's QE/protection policy.
    put_dword(table, base + 56, 0);

    // DWORD16: EN4B methods[31:24], EX4B[18:14] (JESD216/Linux).
    // Native opcodes require a separate 4BAIT, which this BFPT does not emit.
    let f = chip.features;
    let mut dw16 = 0;
    if f.contains(Features::FOUR_BYTE_ENTER) {
        dw16 |= (1 << 24) | (1 << 14);
    }
    if f.contains(Features::FOUR_BYTE_ENTER_WREN) {
        dw16 |= (1 << 25) | (1 << 15);
    }
    if f.contains(Features::FOUR_BYTE_ENTER_EAR7) {
        if f.contains(Features::EXT_ADDR_REG_1716) {
            dw16 |= (1 << 27) | (1 << 17);
        }
        if f.contains(Features::EXT_ADDR_REG_C5C8) {
            dw16 |= (1 << 26) | (1 << 16);
        }
    }
    put_dword(table, base + 60, dw16);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::chip::{ChipDatabase, find_chip_by_name};

    #[test]
    fn sfdp_signature_requires_conservative_presence() {
        let db = ChipDatabase::new();
        let signature =
            |name| generate_sfdp(find_chip_by_name(&db, name).unwrap()).unwrap()[..4].to_vec();
        assert_eq!(signature("W25Q128.V"), b"SFDP");
        assert_eq!(signature("SST25VF080B"), [0xFF; 4]);
        assert_eq!(signature("AT25DF321"), [0xFF; 4]);
        assert_eq!(signature("SST26VF032B(A)"), [0xFF; 4]);
        assert_eq!(signature("AT25F512"), [0xFF; 4]);
    }
    // Field extraction intentionally follows Linux sfdp_bfpt_reads / JESD216,
    // not generator helpers or the RTL's interpretation of BFPT positions.
    fn dword(table: &[u8], number: usize) -> u32 {
        let offset = 16 + (number - 1) * 4;
        u32::from_le_bytes(table[offset..offset + 4].try_into().unwrap())
    }

    #[test]
    fn winbond_bfpt_matches_known_independently_encoded_fixture() {
        let db = ChipDatabase::new();
        let chip = find_chip_by_name(&db, "W25Q128.V").unwrap();
        let generated = generate_sfdp(chip).unwrap();
        let fixture = include_bytes!("../tests/fixtures/w25q128-v.sfdp");
        assert_eq!(&generated, fixture);
        assert_eq!(&generated[..4], b"SFDP");
        assert_eq!((dword(&generated, 1) >> 17) & 3, 0); // 3-byte only
        assert_ne!(dword(&generated, 1) & (1 << 16), 0); // 1-1-2
        assert_eq!(dword(&generated, 1) & (1 << 19), 0); // NO DTR
        assert_eq!(dword(&generated, 2) + 1, chip.total_size * 8);
        let dw4 = dword(&generated, 4);
        assert_eq!((dw4 >> 8) & 255, 0x3b); // 112 low half
        assert_eq!(dw4 & 31, 8);
        assert_eq!((dw4 >> 24) & 255, 0xbb); // 122 high half
        assert_eq!((dw4 >> 21) & 7, 4);
        assert_eq!((dword(&generated, 11) >> 4) & 15, 8); // 256-byte pages
        assert_eq!(dword(&generated, 14), 0); // QER is NOT DWORD14
        assert_eq!((dword(&generated, 15) >> 20) & 7, 0);
        assert_eq!(dword(&generated, 16), 0);
    }

    #[test]
    fn micron_default_sdr_latency_is_parsed_independently_of_macronix_config() {
        let db = ChipDatabase::new();
        let mut chip = find_chip_by_name(&db, "W25Q128.V").unwrap().clone();
        chip.total_size = 1024 * 1024;
        chip.erase_blocks = vec![crate::chip::EraseBlock::with_count(0x20, 4096, 256)];
        chip.features =
            Features::SFDP | Features::FAST_READ | Features::DUAL_IO | Features::QUAD_IO;
        // JESD216 BFPT read halfword: wait[4:0], mode[7:5], opcode[15:8].
        // Parse output fields, not a regenerated golden or generator helper.
        let read = |word: u32, upper: bool| {
            let half = word >> if upper { 16 } else { 0 };
            ((half >> 8) & 255, (half >> 5) & 7, half & 31)
        };
        for (manufacturer, expected_dual, expected_quad) in [
            (0x20, (0xbb, 0, 8), (0xeb, 0, 10)),
            (0xc2, (0xbb, 4, 0), (0xeb, 2, 4)),
        ] {
            chip.jedec_manufacturer = manufacturer;
            let table = generate_sfdp(&chip).unwrap();
            assert_eq!(&table[..4], b"SFDP"); // Small, uniform explicit-SFDP geometry.
            assert_eq!(read(dword(&table, 4), true), expected_dual);
            assert_eq!(read(dword(&table, 3), false), expected_quad);
            assert_eq!(read(dword(&table, 4), false), (0x3b, 0, 8));
            assert_eq!(read(dword(&table, 3), true), (0x6b, 0, 8));
            if manufacturer == 0xc2 {
                let profile = ChipProfile::build(
                    &chip,
                    ChipState {
                        sr2: Some(0x47),
                        ..Default::default()
                    },
                )
                .unwrap();
                assert_eq!(profile.as_bytes()[13], 0x47);
                // SFDP stays static default QuadIO=6, not Config47's runtime=4.
                let (_, mode, wait) = read(dword(&table, 3), false);
                assert_eq!(mode + wait, 6);
            }
        }
    }

    #[test]
    fn address_methods_use_dword16_not_the_qer_word() {
        let db = ChipDatabase::new();
        let mut chip = find_chip_by_name(&db, "AT25DF321").unwrap().clone();
        chip.unlock = rflasher_chips::Unlock::None;
        chip.total_size = 32 * 1024 * 1024;
        chip.erase_blocks = vec![crate::chip::EraseBlock::with_count(0x20, 4096, 8192)];
        for (flag, expected) in [
            (Features::FOUR_BYTE_ENTER, 0x01004000),
            (Features::FOUR_BYTE_ENTER_WREN, 0x02008000),
            (
                Features::FOUR_BYTE_ENTER_EAR7 | Features::EXT_ADDR_REG_1716,
                0x08020000,
            ),
            (
                Features::FOUR_BYTE_ENTER_EAR7 | Features::EXT_ADDR_REG_C5C8,
                0x04010000,
            ),
        ] {
            chip.features = Features::SFDP | Features::FOUR_BYTE_ADDR | flag;
            let generated = generate_sfdp(&chip).unwrap();
            assert_eq!((dword(&generated, 1) >> 17) & 3, 1);
            assert_eq!(dword(&generated, 1) & ((1 << 16) | (1 << 19)), 0);
            assert_eq!(dword(&generated, 15), 0);
            assert_eq!(dword(&generated, 16), expected);
        }
    }

    #[test]
    fn broad_catalog_sfdp_remains_conservative_and_reports_sdr_geometry() {
        let db = ChipDatabase::new();
        for name in [
            "N25Q256",
            "MX25L12833F",
            "GD25Q128",
            "S25FL128L",
            "IS25LP128",
        ] {
            let mut chip = find_chip_by_name(&db, name).unwrap().clone();
            // Explicit metadata permits synthesis; catalog absence is not guessed.
            chip.features |= Features::SFDP;
            let table = generate_sfdp(&chip).unwrap();
            assert_eq!(&table[..4], b"SFDP", "{name}");
            assert_eq!(dword(&table, 2) + 1, chip.total_size * 8);
            assert_eq!(
                (dword(&table, 11) >> 4) & 15,
                chip.page_size.trailing_zeros()
            );
            assert_eq!(dword(&table, 1) & (1 << 19), 0); // No DTR.
            assert_eq!(dword(&table, 5), 0); // No QPI/command-lane modes.
            assert_eq!(dword(&table, 15), 0); // No QE gating.
        }
        let mut chip = find_chip_by_name(&db, "N25Q256").unwrap().clone();
        chip.features |= Features::SFDP;
        let table = generate_sfdp(&chip).unwrap();
        assert_eq!((dword(&table, 1) >> 17) & 3, 1);
        assert_eq!(dword(&table, 16), 0x02008000); // WREN+B7/E9, not native 4BAIT.
        chip.page_size = 512;
        assert!(generate_sfdp(&chip).is_err());
        chip.page_size = 256;
        chip.features.remove(
            Features::FOUR_BYTE_ENTER
                | Features::FOUR_BYTE_ENTER_WREN
                | Features::FOUR_BYTE_ENTER_EAR7,
        );
        assert_eq!(generate_sfdp(&chip).unwrap(), [0xff; SFDP_TABLE_SIZE]);
    }

    #[test]
    fn byte_granularity_metadata_does_not_advertise_page_buffer_programming() {
        let db = ChipDatabase::new();
        let mut chip = find_chip_by_name(&db, "W25Q128.V").unwrap().clone();
        assert!(!chip.features.contains(Features::WRITE_BYTE));
        chip.write_granularity = rflasher_chips::WriteGranularity::Byte;
        let table = generate_sfdp(&chip).unwrap();
        assert_eq!(dword(&table, 1) & (1 << 2), 0);
        chip.write_granularity = rflasher_chips::WriteGranularity::Page;
        let table = generate_sfdp(&chip).unwrap();
        assert_ne!(dword(&table, 1) & (1 << 2), 0);
    }

    #[test]
    fn nonuniform_erasers_never_get_a_fabricated_uniform_bfpt() {
        let db = ChipDatabase::new();
        let mut chip = find_chip_by_name(&db, "SST26VF032B(A)").unwrap().clone();
        chip.features |= Features::SFDP;
        assert_eq!(generate_sfdp(&chip).unwrap(), [0xff; SFDP_TABLE_SIZE]);
        assert!(crate::chip::erase_block_size(&chip.erase_blocks[1]).is_none());
        assert_eq!(chip.sector_erase_ops().len(), 2); // retain boot eraser for profile
    }
    #[test]
    fn chip_erase_is_never_advertised_as_addressed_4k_erase() {
        let db = ChipDatabase::new();
        let mut chip = find_chip_by_name(&db, "AT25DF321").unwrap().clone();
        chip.unlock = rflasher_chips::Unlock::None;
        chip.features = Features::SFDP;
        chip.total_size = 4096;
        chip.erase_blocks = vec![crate::chip::EraseBlock::new(0xc7, 4096)];
        let table = generate_sfdp(&chip).unwrap();
        assert_eq!(dword(&table, 1) & 3, 3);
        assert_eq!((dword(&table, 1) >> 8) & 255, 255);
        assert_eq!(dword(&table, 8), 0);
    }
}
