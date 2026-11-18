//! Conventional SDR SPI NOR protocol hints for protocol 9.
//!
//! Profiles describe transaction framing and erase geometry, not protection
//! fidelity. Status writes are shadows; array mutations are always unprotected.
use crate::chip::{Features, FlashChip, FlashChipExt};
use anyhow::{Result, ensure};
use rflasher_chips::Unlock;

pub const PROFILE_SIZE: usize = 128;

/// Initial shadow state. Omission always starts unprotected, with zero UID.
#[derive(Clone, Copy, Debug, Default)]
pub struct ChipState {
    /// `true` is rejected: the compatibility engine cannot enforce protection.
    pub protected: Option<bool>,
    pub uid: Option<[u8; 8]>,
    pub sr1: Option<u8>,
    pub sr2: Option<u8>,
    pub sr3: Option<u8>,
}

#[derive(Clone, Debug)]
pub struct ChipProfile {
    bytes: [u8; PROFILE_SIZE],
}

impl ChipProfile {
    pub fn as_bytes(&self) -> &[u8; PROFILE_SIZE] {
        &self.bytes
    }

    pub fn build(chip: &FlashChip, state: ChipState) -> Result<Self> {
        ensure!(
            chip.total_size.is_power_of_two()
                && (256..=64 * 1024 * 1024).contains(&chip.total_size),
            "chip capacity must be a power of two between 256 bytes and 64 MiB"
        );
        ensure!(
            chip.page_size.is_power_of_two() && chip.page_size <= 256,
            "hardware supports only power-of-two page sizes from 1 to 256 bytes"
        );
        let mut bytes = [0; PROFILE_SIZE];
        bytes[0] = 2;
        bytes[28] = chip.page_size.trailing_zeros() as u8;
        write_protocol_hints(chip, &mut bytes)?;
        write_erasers(chip, &mut bytes)?;
        write_initial_state(chip.jedec_manufacturer, state, &mut bytes)?;
        Ok(Self { bytes })
    }
}

fn write_protocol_hints(chip: &FlashChip, b: &mut [u8; PROFILE_SIZE]) -> Result<()> {
    let f = chip.features;
    b[1] = match chip.unlock {
        Unlock::At2x => 2, // 3C register alias instead of native dual-output read.
        Unlock::Sst26_6 => 3,
        Unlock::Sst26_10 => 4,
        Unlock::Sst26_18 => 5,
        _ => 0,
    };
    ensure!(
        b[1] != 2 || !f.contains(Features::FOUR_BYTE_DUAL_OUT_READ),
        "At2x 3C register/native-read framing collision"
    );
    // These masks only force unprotected/success status indications. They do
    // not select a range decoder, lock policy or command permission.
    b[2] = if f.contains(Features::WP_WINBOND)
        || chip.aai_word()
        || (chip.jedec_manufacturer == 0xef
            && matches!(
                chip.name.as_str(),
                "W25Q64.V" | "W25Q64BV/W25Q64CV/W25Q64FV" | "W25Q64JV-.Q"
            ))
    {
        0x1c
    } else {
        chip.unlock.status_masks().map_or(0, |masks| masks.0)
    };
    b[3] = 0xff;
    b[4] = 0xff;
    b[5] = 0xff;
    b[7] = chip.unlock.error_mask();
    // The catalog has no QE_SR1 flag. For coarse quad metadata without an
    // explicit SR2 QE hint, preserve the common SR1 bit6 config/QE shadow.
    // Unlock masks are compatibility indications, not authoritative register
    // layouts; even conflicting custom BP/error hints must not hide that bit.
    if f.contains(Features::QUAD_IO) && !f.contains(Features::QE_SR2) {
        b[2] &= !0x40;
        b[7] &= !0x40;
    }
    b[8] = if f.contains(Features::QE_SR2) { 2 } else { 0 };
    b[9] = 0x7f;
    // EAR7 is a framing hint, NOT an unconditional capability: setting bank
    // bit7 must only switch address length on parts using that convention.
    b[10] = if f.contains(Features::FOUR_BYTE_ENTER_EAR7) {
        7 | feature_bits(
            f,
            &[
                (Features::EXT_ADDR_REG_C5C8, 8),
                (Features::EXT_ADDR_REG_1716, 16),
            ],
        )
    } else {
        0x1b
    };
    ensure!(
        !f.contains(Features::FOUR_BYTE_ENTER_EAR7)
            || f.intersects(Features::EXT_ADDR_REG_C5C8 | Features::EXT_ADDR_REG_1716),
        "EAR7 framing requires an explicit register command pair"
    );
    b[11] = 0xa7 | feature_bits(f, &[(Features::AAI_WORD, 8), (Features::AT25F_ID, 64)]);
    // Byte granularity is catalog metadata independent of WRITE_BYTE. AAI
    // parts' 02 is also byte-only, regardless of the catalog page default.
    if chip.write_byte() || chip.aai_word() {
        b[11] |= 16;
    }
    Ok(())
}

fn feature_bits(features: Features, fields: &[(Features, u8)]) -> u8 {
    fields
        .iter()
        .filter(|(flag, _)| features.contains(*flag))
        .fold(0, |bits, (_, bit)| bits | bit)
}

// Core read/program/status commands cannot be reinterpreted as erasers.
// Explicit erase geometry takes priority over vendor config/lock aliases and
// EWSR50, so those aliases must not be reserved unconditionally.
fn validate_erase_opcode(opcode: u8) -> Result<()> {
    const FIXED: &[u8] = &[
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x0b, 0x0c, 0x11, 0x12, 0x13, 0x15, 0x16, 0x17, 0x31,
        0x32, 0x34, 0x35, 0x36, 0x38, 0x39, 0x3b, 0x3c, 0x42, 0x4b, 0x5a, 0x60, 0x62, 0x66, 0x6b,
        0x6c, 0x72, 0x98, 0x99, 0x9e, 0x9f, 0xad, 0xb7, 0xbb, 0xbc, 0xc5, 0xc7, 0xc8, 0xe9, 0xeb,
        0xec,
    ];
    ensure!(
        !FIXED.contains(&opcode),
        "erase opcode {opcode:02x} collides with a fixed command"
    );
    Ok(())
}

fn write_erasers(chip: &FlashChip, b: &mut [u8; PROFILE_SIZE]) -> Result<()> {
    let mut descriptor = 0;
    let mut opcodes = Vec::new();
    for erase in &chip.erase_blocks {
        ensure!(
            erase.opcode != 0 && erase.opcode != 0xff,
            "invalid erase opcode"
        );
        ensure!(!opcodes.contains(&erase.opcode), "duplicate erase opcode");
        opcodes.push(erase.opcode);
        ensure!(
            (1..=5).contains(&erase.regions.len()),
            "erase requires 1..5 regions"
        );
        let covered = erase.regions.iter().try_fold(0u64, |sum, region| {
            ensure!(
                region.size.is_power_of_two() && (256..=64 * 1024 * 1024).contains(&region.size),
                "invalid erase region size"
            );
            ensure!(
                (1..=0xff_ffff).contains(&region.count),
                "invalid erase region count"
            );
            ensure!(
                sum % u64::from(region.size) == 0,
                "unaligned erase region boundary"
            );
            Ok::<_, anyhow::Error>(sum + u64::from(region.size) * u64::from(region.count))
        })?;
        ensure!(
            covered == u64::from(chip.total_size),
            "erase regions must cover capacity exactly"
        );
        if matches!(erase.opcode, 0xc7 | 0x60 | 0x62) {
            ensure!(
                erase.is_chip_erase() && erase.opcode_4b.is_none(),
                "invalid chip erase layout"
            );
            b[24] |= match erase.opcode {
                0xc7 => 1,
                0x60 => 2,
                _ => 4,
            };
            continue;
        }
        validate_erase_opcode(erase.opcode)?;
        ensure!(
            descriptor < 4,
            "at most four addressed erasers are supported"
        );
        ensure!(
            b[1] != 2 || erase.opcode != 0x3c,
            "At2x 3C opcode collision"
        );
        let offset = 32 + descriptor * 24;
        b[offset] = erase.opcode;
        if let Some(native) = erase.opcode_4b {
            ensure!(
                native != 0 && native != 0xff && !opcodes.contains(&native),
                "invalid/duplicate native erase opcode"
            );
            ensure!(!(b[1] == 2 && native == 0x3c), "At2x 3C opcode collision");
            validate_erase_opcode(native)?;
            opcodes.push(native);
            b[offset + 1] = native;
        }
        b[offset + 2] = erase.regions.len() as u8;
        for (index, region) in erase.regions.iter().enumerate() {
            let record = offset + 4 + index * 4;
            b[record] = region.size.trailing_zeros() as u8;
            b[record + 1..record + 4].copy_from_slice(&region.count.to_le_bytes()[..3]);
        }
        descriptor += 1;
    }
    ensure!(
        !chip.erase_blocks.is_empty(),
        "chip has no erase definitions"
    );
    Ok(())
}

fn write_initial_state(
    manufacturer: u8,
    state: ChipState,
    b: &mut [u8; PROFILE_SIZE],
) -> Result<()> {
    ensure!(
        (b[2] | b[7]) & 3 == 0,
        "BP/error indication masks cannot include WIP/WEL polling bits"
    );
    ensure!(
        state.protected != Some(true),
        "protected initial state is unsupported: controller compatibility never enforces protection"
    );
    if let Some(sr1) = state.sr1 {
        ensure!(
            sr1 & (b[2] | b[7] | 0x83) == 0,
            "initial SR1 cannot set protection/lock, error or busy/write-enable indications"
        );
        b[12] = sr1;
    }
    // Macronix stores Config in the SR2 shadow: WRSR01's second byte is
    // returned unmasked by READ15, including its latency/configuration bits.
    ensure!(
        manufacturer == 0xc2 || state.sr2.unwrap_or(0) & 0x41 == 0,
        "initial SR2 cannot set CMP/SRL protection indications"
    );
    b[13] = state.sr2.unwrap_or(0);
    b[14] = state.sr3.unwrap_or(0);
    if let Some(uid) = state.uid {
        b[16..24].copy_from_slice(&uid);
    }
    Ok(())
}

/// Parse eight UID bytes in SPI response order (16 hex digits).
pub fn parse_uid(value: &str) -> std::result::Result<[u8; 8], String> {
    let hex = value.strip_prefix("0x").unwrap_or(value);
    if hex.len() != 16 || !hex.bytes().all(|b| b.is_ascii_hexdigit()) {
        return Err("UID must be exactly 16 hex digits in SPI response order".into());
    }
    let mut bytes = [0; 8];
    for (index, byte) in bytes.iter_mut().enumerate() {
        *byte = u8::from_str_radix(&hex[index * 2..index * 2 + 2], 16)
            .map_err(|error| error.to_string())?;
    }
    Ok(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::chip::{ChipDatabase, find_chip_by_name};
    use rflasher_chips::{EraseBlock, EraseRegion};

    fn chip(name: &str) -> FlashChip {
        find_chip_by_name(&ChipDatabase::new(), name)
            .unwrap()
            .clone()
    }

    #[test]
    fn compatibility_profiles_match_independent_wire_fixtures() {
        for (name, fixture) in [
            (
                "W25Q128.V",
                include_bytes!("../tests/fixtures/w25q128-v.profile"),
            ),
            (
                "W25Q64BV/W25Q64CV/W25Q64FV",
                include_bytes!("../tests/fixtures/w25q64-fv.profile"),
            ),
            (
                "AT25DF321",
                include_bytes!("../tests/fixtures/at25df321.profile"),
            ),
            (
                "SST26VF032B(A)",
                include_bytes!("../tests/fixtures/sst26vf032b.profile"),
            ),
            (
                "SST25VF016B",
                include_bytes!("../tests/fixtures/sst25vf016b.profile"),
            ),
            (
                "SST25VF032B",
                include_bytes!("../tests/fixtures/sst25vf032b.profile"),
            ),
        ] {
            assert_eq!(
                ChipProfile::build(&chip(name), ChipState::default())
                    .unwrap()
                    .as_bytes(),
                fixture,
                "{name}"
            );
        }
        for (name, fixture) in [
            (
                "SST26VF016B(A)",
                include_str!("../tests/fixtures/sst26vf016b.profile.hex"),
            ),
            (
                "SST26VF064B(A)",
                include_str!("../tests/fixtures/sst26vf064b.profile.hex"),
            ),
        ] {
            let expected: Vec<u8> = fixture
                .split_whitespace()
                .map(|b| u8::from_str_radix(b, 16).unwrap())
                .collect();
            assert_eq!(
                ChipProfile::build(&chip(name), ChipState::default())
                    .unwrap()
                    .as_bytes()
                    .as_slice(),
                expected,
                "{name}"
            );
        }
    }

    #[test]
    fn broad_vendor_profiles_are_unprotected_protocol_hints() {
        for name in [
            "N25Q128",
            "N25Q256",
            "N25Q512",
            "MT25QL256",
            "MX25L12833F",
            "GD25Q128",
            "S25FL128L",
            "S25FL256",
            "IS25LP128",
            "EN25Q128",
            "SST26VF016",
            "SST26VF080A",
            "AT25F512A",
            "AT25DF021",
            "AT26DF041",
            "W25Q256JV_Q",
            "SST25WF080",
        ] {
            let part = chip(name);
            let p = ChipProfile::build(&part, ChipState::default())
                .unwrap_or_else(|error| panic!("{name}: {error}"));
            let b = p.as_bytes();
            assert_eq!(b[0], 2, "{name}");
            assert_eq!((b[2] | b[7]) & 3, 0, "{name}");
            assert_eq!(&b[3..7], &[0xff, 0xff, 0xff, 0]);
            assert_eq!(b[9], 0x7f);
            assert_eq!(b[10] & !4, 0x1b);
            assert_eq!(b[11] & 0xa7, 0xa7);
            assert_eq!(&b[12..24], &[0; 12]);
            assert_eq!(&b[25..28], &[0; 3]);
        }
        for policy in [
            Unlock::Unknown,
            Unlock::Unsupported,
            Unlock::Bp3Srwd,
            Unlock::Bp4Srwd,
            Unlock::N25q,
            Unlock::Bp2EpSrwd,
        ] {
            let mut part = chip("W25Q128.V");
            part.unlock = policy;
            part.features |= Features::WP_UNRESOLVED | Features::WP_BP3 | Features::WP_WPS;
            ChipProfile::build(&part, ChipState::default()).unwrap();
        }
    }

    #[test]
    fn initial_shadows_and_uid_are_validated_without_promising_protection() {
        for name in ["W25Q128.V", "N25Q128", "AT25DF321", "SST26VF032B(A)"] {
            let part = chip(name);
            let error = ChipProfile::build(
                &part,
                ChipState {
                    protected: Some(true),
                    ..Default::default()
                },
            )
            .unwrap_err()
            .to_string();
            assert!(error.contains("never enforces protection"));
            let uid = parse_uid("0123456789abcdef").unwrap();
            let p = ChipProfile::build(
                &part,
                ChipState {
                    protected: Some(false),
                    uid: Some(uid),
                    sr2: Some(0x82),
                    sr3: Some(0x80),
                    ..Default::default()
                },
            )
            .unwrap();
            assert_eq!(
                &p.as_bytes()[16..24],
                &[1, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef]
            );
            assert_eq!(&p.as_bytes()[12..15], &[0, 0x82, 0x80]);
            for sr1 in [1, 2, 3, 0x80, p.as_bytes()[2] | p.as_bytes()[7]] {
                if sr1 != 0 {
                    assert!(
                        ChipProfile::build(
                            &part,
                            ChipState {
                                sr1: Some(sr1),
                                ..Default::default()
                            }
                        )
                        .is_err()
                    );
                }
            }
        }
        let p = ChipProfile::build(
            &chip("W25Q128.V"),
            ChipState {
                sr1: Some(0x20),
                ..Default::default()
            },
        )
        .unwrap();
        assert_eq!(p.as_bytes()[12], 0x20); // Harmless config shadow, not enforcement.
        for sr2 in [1, 0x40, 0x41] {
            assert!(
                ChipProfile::build(
                    &chip("W25Q128.V"),
                    ChipState {
                        sr2: Some(sr2),
                        ..Default::default()
                    }
                )
                .is_err()
            );
        }
        assert!(parse_uid("é00000000000000").is_err());
        assert!(parse_uid("1234").is_err());
    }

    #[test]
    fn macronix_initial_config_shadow_preserves_all_bits() {
        let mut part = chip("MX25L12833F");
        assert_eq!(part.jedec_manufacturer, 0xc2);
        for config in [0x47, 0xff] {
            let p = ChipProfile::build(
                &part,
                ChipState {
                    sr2: Some(config),
                    ..Default::default()
                },
            )
            .unwrap();
            assert_eq!(p.as_bytes()[13], config); // Unmasked READ15 Config shadow.
            assert_eq!(p.as_bytes()[8], 0); // No invented QE_SR2 hint.
        }
        for manufacturer in [0xef, 0x20, 0x01] {
            part.jedec_manufacturer = manufacturer;
            assert!(
                ChipProfile::build(
                    &part,
                    ChipState {
                        sr2: Some(0x47),
                        ..Default::default()
                    }
                )
                .unwrap_err()
                .to_string()
                .contains("CMP/SRL")
            );
        }
    }

    #[test]
    fn framing_hints_preserve_aai_legacy_id_and_native_erase_geometry() {
        let sst = ChipProfile::build(&chip("SST25VF016B"), ChipState::default()).unwrap();
        assert_eq!(sst.as_bytes()[11], 0xbf); // AAI + byte-only 02 + common commands.
        let at = ChipProfile::build(&chip("AT25F512"), ChipState::default()).unwrap();
        assert_eq!(at.as_bytes()[24], 4); // Addressless 62.
        assert_eq!(at.as_bytes()[11], 0xe7); // Legacy ID15.
        let micron = ChipProfile::build(&chip("N25Q256"), ChipState::default()).unwrap();
        assert_eq!(
            &micron.as_bytes()[32..40],
            &[0x20, 0x21, 1, 0, 12, 0, 32, 0]
        );
        assert_eq!(&micron.as_bytes()[56..64], &[0xd8, 0xdc, 1, 0, 16, 0, 2, 0]);
        for (name, model) in [
            ("SST26VF016B(A)", 3),
            ("SST26VF032B(A)", 4),
            ("SST26VF064B(A)", 5),
        ] {
            let p = ChipProfile::build(&chip(name), ChipState::default()).unwrap();
            assert_eq!(p.as_bytes()[1], model);
            assert_eq!(p.as_bytes()[58], 5); // Nonuniform D8 preserved, not BPR enforced.
        }
    }

    #[test]
    fn byte_only_framing_uses_granularity_flags_and_aai() {
        use rflasher_chips::WriteGranularity;

        for name in ["AT26DF041", "F25L008A", "F25L004A"] {
            let part = chip(name);
            assert_eq!(part.write_granularity, WriteGranularity::Byte);
            assert!(!part.features.contains(Features::WRITE_BYTE));
            let p = ChipProfile::build(&part, ChipState::default()).unwrap();
            assert_ne!(p.as_bytes()[11] & 16, 0, "{name}");
        }
        let mut part = chip("W25Q128.V");
        for (granularity, features, expected) in [
            (WriteGranularity::Page, Features::empty(), 0),
            (WriteGranularity::Bit, Features::empty(), 0),
            (WriteGranularity::Byte, Features::empty(), 16),
            (WriteGranularity::Page, Features::WRITE_BYTE, 16),
            (WriteGranularity::Page, Features::AAI_WORD, 24),
        ] {
            part.write_granularity = granularity;
            part.features = features;
            let p = ChipProfile::build(&part, ChipState::default()).unwrap();
            assert_eq!(p.as_bytes()[11] & 24, expected);
        }
    }

    #[test]
    fn at26df041_explicit_erase_geometry_overrides_vendor_config_alias() {
        let part = chip("AT26DF041");
        let p = ChipProfile::build(&part, ChipState::default()).unwrap();
        let b = p.as_bytes();
        assert_eq!(part.total_size, 512 * 1024);
        assert_eq!(b[0], 2);
        assert_ne!(b[11] & 16, 0); // Byte-only02 without WRITE_BYTE/AAI flags.
        assert_eq!(b[24], 0); // No invented addressless chip erase.
        assert_eq!(&b[32..40], &[0x81, 0, 1, 0, 8, 0, 8, 0]); // 256B * 2048.
        assert_eq!(&b[56..64], &[0x50, 0, 1, 0, 11, 0, 1, 0]); // 2KiB * 256.
        assert_eq!(&b[80..88], &[0x20, 0, 1, 0, 12, 128, 0, 0]); // 4KiB * 128.
        for start in [40, 64, 88] {
            assert_eq!(&b[start..start + 16], &[0; 16]);
        }
        assert_eq!(&b[104..], &[0; 24]);
    }

    #[test]
    fn shared_initial_validation_rejects_masks_that_hide_polling_bits() {
        for index in [2, 7] {
            for mask in [1, 2, 3, 0xff] {
                let mut bytes = [0; PROFILE_SIZE];
                bytes[index] = mask;
                let error = write_initial_state(0xef, ChipState::default(), &mut bytes)
                    .unwrap_err()
                    .to_string();
                assert!(error.contains("WIP/WEL polling bits"));
            }
        }
        for (bp, error) in [(0, 0), (0x1c, 0x20), (0x3c, 0x60)] {
            let mut bytes = [0; PROFILE_SIZE];
            bytes[2] = bp;
            bytes[7] = error;
            write_initial_state(0xef, ChipState::default(), &mut bytes).unwrap();
            assert_eq!(bytes[2], bp);
            assert_eq!(bytes[7], error);
        }
    }

    #[test]
    fn sr1_quad_config_shadow_survives_wire_readback_masks() {
        for name in [
            "MX25L12833F",
            "IS25WP128",
            "N25Q256",
            "GD25Q256D/GD25B256D/GD25R256D",
        ] {
            let part = chip(name);
            assert!(part.features.contains(Features::QUAD_IO));
            assert!(!part.features.contains(Features::QE_SR2));
            let p = ChipProfile::build(
                &part,
                ChipState {
                    sr1: Some(0x40),
                    ..Default::default()
                },
            )
            .unwrap();
            let b = p.as_bytes();
            // RTL SR1 readback clears BP/error/SRWD but preserves config/QE.
            let readback = b[12] & !(b[2] | b[7] | 0x80);
            assert_eq!(readback, 0x40, "{name}");
        }
        let mut part = chip("MX25L12833F");
        part.unlock = Unlock::Bp4Srwd;
        part.features = Features::QUAD_IO;
        let p = ChipProfile::build(
            &part,
            ChipState {
                sr1: Some(0x40),
                ..Default::default()
            },
        )
        .unwrap();
        assert_eq!(p.as_bytes()[2], 0x3c); // Retain BP2..5, not config6.
        part.unlock = Unlock::Bp2EpSrwd;
        let p = ChipProfile::build(
            &part,
            ChipState {
                sr1: Some(0x40),
                ..Default::default()
            },
        )
        .unwrap();
        assert_eq!(p.as_bytes()[2], 0x1c);
        assert_eq!(p.as_bytes()[7], 0x20); // Error6 cannot hide config6 either.
        assert_eq!(
            p.as_bytes()[12] & !(p.as_bytes()[2] | p.as_bytes()[7] | 0x80),
            0x40
        );
        part.unlock = Unlock::Bp4Srwd;
        part.features |= Features::QE_SR2;
        let p = ChipProfile::build(&part, ChipState::default()).unwrap();
        assert_eq!(p.as_bytes()[2], 0x7c); // Explicit SR2 QE does not reinterpret BP6.
        assert!(
            ChipProfile::build(
                &part,
                ChipState {
                    sr1: Some(0x40),
                    ..Default::default()
                }
            )
            .is_err()
        );
    }

    #[test]
    fn address_framing_ambiguities_are_rejected_but_coarse_capabilities_are_not() {
        let mut part = chip("AT25DF321");
        part.features |= Features::FOUR_BYTE_DUAL_OUT_READ;
        assert!(ChipProfile::build(&part, ChipState::default()).is_err());
        part.unlock = Unlock::Unknown;
        part.features = Features::FOUR_BYTE_ENTER_EAR7;
        assert!(ChipProfile::build(&part, ChipState::default()).is_err());
        part.features |= Features::EXT_ADDR_REG_1716;
        assert_eq!(
            ChipProfile::build(&part, ChipState::default())
                .unwrap()
                .as_bytes()[10],
            0x17
        );
        part.features = Features::EXT_ADDR_REG
            | Features::FOUR_BYTE_ADDR
            | Features::FOUR_BYTE_NATIVE
            | Features::WRSR_VOLATILE_WREN;
        ChipProfile::build(&part, ChipState::default()).unwrap();
    }

    #[test]
    fn page_sizes_use_the_actual_hardware_wrap_exponent() {
        for size in [1, 2, 16, 64, 128, 256] {
            let mut part = chip("W25Q128.V");
            part.page_size = size;
            let p = ChipProfile::build(&part, ChipState::default()).unwrap();
            assert_eq!(p.as_bytes()[28], size.trailing_zeros() as u8);
        }
        for size in [0, 3, 255, 512, 1024] {
            let mut part = chip("W25Q128.V");
            part.page_size = size;
            assert!(ChipProfile::build(&part, ChipState::default()).is_err());
        }
    }

    #[test]
    fn compatibility_can_use_custom_and_whole_capacity_addressed_erasers() {
        let mut part = chip("AT25DF321");
        part.unlock = Unlock::None;
        part.features = Features::empty();
        part.erase_blocks = vec![
            EraseBlock::with_regions_and_4b(0x50, Some(0x51), &[EraseRegion::new(4096, 1024)]),
            EraseBlock::new(0xd0, part.total_size),
            EraseBlock::new(0xc7, part.total_size),
        ];
        let p = ChipProfile::build(&part, ChipState::default()).unwrap();
        assert_eq!(p.as_bytes()[1], 0);
        assert_eq!(&p.as_bytes()[32..40], &[0x50, 0x51, 1, 0, 12, 0, 4, 0]);
        assert_eq!(&p.as_bytes()[56..64], &[0xd0, 0, 1, 0, 22, 1, 0, 0]);
        assert_eq!(p.as_bytes()[24], 1);
        assert_eq!(part.sector_erase_ops().len(), 2);
    }

    #[test]
    fn bad_erase_layouts_and_geometry_are_rejected_not_truncated() {
        let original = chip("W25Q128.V");
        let mut part = original.clone();
        part.page_size = 512;
        assert!(ChipProfile::build(&part, ChipState::default()).is_err());
        for size in [0, 128, 384, 128 * 1024 * 1024] {
            part = original.clone();
            part.total_size = size;
            assert!(ChipProfile::build(&part, ChipState::default()).is_err());
        }
        for erasers in [
            (0xd0..=0xd4)
                .map(|op| EraseBlock::with_count(op, 4096, 4096))
                .collect::<Vec<_>>(),
            vec![],
            vec![EraseBlock::with_count(0, 4096, 4096)],
            vec![EraseBlock::with_count(0xff, 4096, 4096)],
            vec![EraseBlock::with_regions(0x20, &[])],
            vec![EraseBlock::with_regions(
                0x20,
                &[
                    EraseRegion::new(4096, 1),
                    EraseRegion::new(65536, 255),
                    EraseRegion::new(4096, 15),
                ],
            )],
            vec![EraseBlock::with_regions_and_4b(
                0x20,
                Some(0),
                &[EraseRegion::new(4096, 4096)],
            )],
            vec![EraseBlock::with_regions_and_4b(
                0x20,
                Some(0xff),
                &[EraseRegion::new(4096, 4096)],
            )],
            vec![EraseBlock::with_regions_and_4b(
                0x20,
                Some(0x20),
                &[EraseRegion::new(4096, 4096)],
            )],
            vec![EraseBlock::with_count(0x20, 4096, 4095)],
            vec![EraseBlock::with_count(0x20, 4096, 0)],
            vec![EraseBlock::with_count(0x20, 4096, 0x1000000)],
            vec![EraseBlock::with_count(0x20, 4097, 4096)],
            vec![EraseBlock::with_count(0xc7, 4096, 4096)],
            vec![EraseBlock::with_regions(
                0x20,
                &[EraseRegion::new(4096, 1); 6],
            )],
            vec![EraseBlock::with_count(0x20, 4096, 4096); 2],
        ] {
            part = original.clone();
            part.erase_blocks = erasers;
            assert!(ChipProfile::build(&part, ChipState::default()).is_err());
        }
    }

    #[test]
    fn erasers_cannot_reinterpret_core_commands_but_vendor_aliases_have_descriptor_priority() {
        let mut part = chip("W25Q128.V");
        for opcode in [
            0x01, 0x03, 0x05, 0x11, 0x15, 0x32, 0x34, 0x38, 0x3c, 0x4b, 0x60, 0xc7, 0xc8, 0xeb,
        ] {
            part.erase_blocks = vec![EraseBlock::with_count(opcode, 4096, 4096)];
            assert!(ChipProfile::build(&part, ChipState::default()).is_err());
            part.erase_blocks = vec![EraseBlock::with_regions_and_4b(
                0xd0,
                Some(opcode),
                &[EraseRegion::new(4096, 4096)],
            )];
            assert!(ChipProfile::build(&part, ChipState::default()).is_err());
        }
        for opcode in [0x61, 0x65, 0x70, 0x81, 0x85, 0xb1, 0xb5, 0xe0, 0xe2] {
            part.erase_blocks = vec![EraseBlock::with_count(opcode, 4096, 4096)];
            let p = ChipProfile::build(&part, ChipState::default()).unwrap();
            assert_eq!(&p.as_bytes()[32..40], &[opcode, 0, 1, 0, 12, 0, 16, 0]);
            part.erase_blocks = vec![EraseBlock::with_regions_and_4b(
                0xd0,
                Some(opcode),
                &[EraseRegion::new(4096, 4096)],
            )];
            let p = ChipProfile::build(&part, ChipState::default()).unwrap();
            assert_eq!(&p.as_bytes()[32..40], &[0xd0, opcode, 1, 0, 12, 0, 16, 0]);
        }
        for unlock in [Unlock::None, Unlock::Unknown, Unlock::At2x, Unlock::N25q] {
            part.unlock = unlock;
            part.features = Features::WRSR_PERSISTENT_EWSR;
            part.erase_blocks = vec![EraseBlock::with_count(0x50, 4096, 4096)];
            assert_eq!(
                ChipProfile::build(&part, ChipState::default())
                    .unwrap()
                    .as_bytes()[32],
                0x50
            );
        }
    }
}
