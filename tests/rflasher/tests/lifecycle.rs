use futures_lite::future::block_on;
use norbert_rflasher_integration::{RtlMaster, hex};
use rflasher_chips::{ChipDatabase, Features, FlashChip, Unlock, WriteGranularity};
use rflasher_core::flash::{
    FlashContext, FlashDevice, ProbeOptions, SpiFlashDevice, probe_with_options,
};

const UID: [u8; 8] = [0x4e, 0x4f, 0x52, 0x62, 0x65, 0x72, 0x74, 0x38];

fn chip(name: &str) -> FlashChip {
    ChipDatabase::new()
        .iter()
        .find(|c| c.name == name)
        .expect("pinned catalog entry")
        .clone()
}

// Independent wire encoder, not a protection-policy whitelist. Only transaction
// framing and exact catalog geometry matter; every profile starts unprotected.
fn profile(chip: &FlashChip) -> [u8; 128] {
    assert!(chip.total_size.is_power_of_two());
    assert!((128..=64 * 1024 * 1024).contains(&chip.total_size));
    assert!(chip.page_size.is_power_of_two() && chip.page_size <= 256);
    let mut p = [0; 128];
    p[0] = 2;
    p[1] = match chip.unlock {
        Unlock::At2x => 2,
        Unlock::Sst26_6 => 3,
        Unlock::Sst26_10 => 4,
        Unlock::Sst26_18 => 5,
        _ => 0,
    };
    p[2] = if chip.features.contains(Features::WP_WINBOND)
        || chip.features.contains(Features::AAI_WORD)
        || (chip.jedec_manufacturer == 0xef && chip.total_size == 8 * 1024 * 1024)
    {
        0x1c
    } else {
        chip.unlock.status_masks().map_or(0, |m| m.0)
    };
    p[3] = 0xff;
    p[4] = 0xff;
    p[7] = chip.unlock.error_mask();
    // On SR1-QE chips, bit6 is configuration rather than a protection/error
    // indication. Preserve it just like the production framing hints do.
    if chip.features.contains(Features::QUAD_IO) && !chip.features.contains(Features::QE_SR2) {
        p[2] &= !0x40;
        p[7] &= !0x40;
    }
    p[11] = if chip.features.contains(Features::AAI_WORD) {
        8
    } else {
        0
    } | if chip.write_granularity == WriteGranularity::Byte {
        16
    } else {
        0
    } | if chip.features.contains(Features::AT25F_ID) {
        64
    } else {
        0
    };
    p[16..24].copy_from_slice(&UID);
    p[28] = chip.page_size.trailing_zeros() as u8;
    let mut descriptor = 0;
    let mut opcodes = std::collections::HashSet::new();
    for erase in chip.erase_blocks() {
        assert_eq!(
            erase
                .regions
                .iter()
                .map(|r| u64::from(r.size) * u64::from(r.count))
                .sum::<u64>(),
            u64::from(chip.total_size),
            "complete geometry for {:02x}",
            erase.opcode
        );
        if erase.is_chip_erase() {
            p[24] |= match erase.opcode {
                0xc7 => 1,
                0x60 => 2,
                0x62 => 4,
                _ => panic!("unrepresentable chip erase"),
            };
        } else {
            assert!(descriptor < 4, "no geometry truncation");
            assert!((1..=5).contains(&erase.regions.len()));
            assert!(opcodes.insert(erase.opcode), "duplicate erase opcode");
            if let Some(native) = erase.opcode_4b {
                assert!(opcodes.insert(native), "duplicate native erase opcode");
            }
            let offset = 32 + 24 * descriptor;
            p[offset] = erase.opcode;
            p[offset + 1] = erase.opcode_4b.unwrap_or(0);
            p[offset + 2] = erase.regions.len() as u8;
            let mut start = 0u64;
            for (i, region) in erase.regions.iter().enumerate() {
                assert!(region.size.is_power_of_two() && region.size >= 256);
                assert!((1..=0xff_ffff).contains(&region.count));
                assert_eq!(start % u64::from(region.size), 0, "aligned region");
                p[offset + 4 + i * 4] = region.size.trailing_zeros() as u8;
                p[offset + 5 + i * 4..offset + 8 + i * 4]
                    .copy_from_slice(&region.count.to_le_bytes()[..3]);
                start += u64::from(region.size) * u64::from(region.count);
            }
            descriptor += 1;
        }
    }
    p
}

fn configured(name: &str) -> SpiFlashDevice<RtlMaster> {
    let chip = chip(name);
    let independent = profile(&chip);
    // Optional end-to-end path: compare independent geometry/framing, then
    // actually configure RTL with the production builder's packet.
    let p = if let Ok(dir) = std::env::var("NORBERT_PROFILE_FIXTURES") {
        let actual: [u8; 128] = std::fs::read(format!("{dir}/{name}.bin"))
            .expect("format2 host-generated profile fixture")
            .try_into()
            .expect("128-byte profile");
        for i in [0, 1, 2, 7, 24, 27, 28].into_iter().chain(32..128) {
            assert_eq!(actual[i], independent[i], "host fixture {name} byte {i}");
        }
        assert_eq!(actual[11] & 0x58, independent[11], "program/ID framing");
        assert_eq!(&actual[16..24], &UID, "requested production UID");
        actual
    } else {
        independent
    };
    configured_profile(chip, &p)
}

fn configured_profile(chip: FlashChip, p: &[u8; 128]) -> SpiFlashDevice<RtlMaster> {
    let mut master = RtlMaster::new();
    assert_eq!(master.host(&[0x30]), [9]);
    assert_eq!(master.host(&[0x35]), [1]);
    let bursts = chip.total_size / 8 - 1;
    assert_eq!(
        master.host(&[
            0x33,
            chip.jedec_manufacturer,
            (chip.jedec_device >> 8) as u8,
            chip.jedec_device as u8,
            0,
            (bursts >> 16) as u8,
            (bursts >> 8) as u8,
            bursts as u8,
            0,
        ]),
        [1]
    );
    let mut config = vec![0x3c];
    config.extend(p);
    assert_eq!(master.host(&config), [1], "CHIPPROFILE {}", hex(p));
    assert_eq!(master.host(&[0x34]), [1]);
    assert_eq!(
        master.raw(&[0x9f], 3),
        [
            chip.jedec_manufacturer,
            (chip.jedec_device >> 8) as u8,
            chip.jedec_device as u8
        ]
    );
    assert_eq!(
        master.raw(&[0x05], 1)[0] & (p[2] | p[7] | 0x83),
        0,
        "initially idle and unprotected (informational WP# may be set)"
    );
    SpiFlashDevice::new(master, FlashContext::new(chip))
}

fn wait_ready(master: &mut RtlMaster) {
    assert!(
        (0..200).any(|_| master.raw(&[0x05], 1)[0] & 1 == 0),
        "WIP timeout"
    );
}

async fn mutate_and_verify(device: &mut SpiFlashDevice<RtlMaster>, address: u32) {
    device.master().seed(address, &[0; 4096]);
    if address != 0 {
        device.master().seed(address - 8, &[0x96; 8]);
    }
    device.master().seed(address + 4096, &[0x69; 32]);
    device
        .erase(address, 4096)
        .await
        .expect("real driver erase");
    assert_eq!(device.master().memory(address, 4096), vec![0xff; 4096]);
    if address != 0 {
        assert_eq!(device.master().memory(address - 8, 8), [0x96; 8]);
    }
    assert_eq!(device.master().memory(address + 4096, 32), [0x69; 32]);
    let data: Vec<u8> = (0..513).map(|i| (i * 29 + 7) as u8).collect();
    device
        .write(address + 7, &data)
        .await
        .expect("cross-page driver write");
    let mut readback = vec![0; data.len()];
    device
        .read(address + 7, &mut readback)
        .await
        .expect("SPI array readback");
    assert_eq!(device.master().memory(address + 7, data.len()), data);
    assert_eq!(readback, data);
    assert_eq!(device.master().memory(address, 7), [0xff; 7]);
    assert_eq!(device.master().memory(address + 520, 8), [0xff; 8]);
    device
        .write(address + 11, &[0xf0; 17])
        .await
        .expect("NOR RMW second write");
    let expected: Vec<u8> = data[4..21].iter().map(|b| b & 0xf0).collect();
    assert_eq!(device.master().memory(address + 11, 17), expected);
    let mut readback = [0; 17];
    device.read(address + 11, &mut readback).await.unwrap();
    assert_eq!(readback.as_slice(), expected);
    assert_eq!(
        device.master().host(&[0x3b]),
        [0x80],
        "complete prefetch beats"
    );
}

#[test]
fn broad_vendor_jedec_probe_and_mutation() {
    block_on(async {
        for name in [
            "N25Q128",
            "N25Q256",
            "N25Q512",
            "MX25L12833F",
            "MX66L51235F",
            "GD25Q128",
            "GD25Q256D",
            "IS25LP128",
            "IS25LP512MG",
            "S25FL128L",
            "EN25Q64",
            "AT26DF041",
        ] {
            let device = configured(name);
            let expected = device.context().chip.clone();
            let (mut master, _) = device.into_parts();
            let manufacturer = expected.jedec_manufacturer;
            let density = (expected.jedec_device as u8).wrapping_sub(1);
            assert_eq!(master.raw(&[0x9e], 3), master.raw(&[0x9f], 3));
            assert_eq!(
                master.raw(&[0x90, 0, 0, 0], 4),
                [manufacturer, density, manufacturer, density]
            );
            assert_eq!(master.raw(&[0xab, 0, 0, 0], 2), [density; 2]);
            // Use the actual pinned JEDEC/SFDP driver path, including ambiguous
            // catalog IDs resolved by name. No SFDP is advertised in this case.
            let result = probe_with_options(
                &mut master,
                &ChipDatabase::new(),
                &ProbeOptions {
                    chip: Some(name),
                    force: false,
                },
            )
            .await
            .unwrap_or_else(|e| panic!("probe {name}: {e:?}"));
            assert!(result.from_database);
            assert!(result.sfdp.is_none());
            assert_eq!(result.chip.name, expected.name);
            assert_eq!(result.chip.total_size, expected.total_size);
            assert_eq!(result.chip.page_size, expected.page_size);
            assert_eq!(result.chip.erase_blocks(), expected.erase_blocks());
            let mut device = SpiFlashDevice::new(master, result.into_context());
            // Native 4-byte mutations really access a high bank, not bank zero.
            let address = if expected.total_size > 16 * 1024 * 1024 {
                expected.total_size - 8192
            } else {
                4096
            };
            device.master().seed(address & 0xff_ffff, &[0x39; 32]);
            mutate_and_verify(&mut device, address).await;
            if name == "AT26DF041" {
                // Actual 81 geometry wins over Micron config-write aliasing.
                let eraser = device
                    .erase_blocks()
                    .iter()
                    .find(|b| b.opcode == 0x81)
                    .unwrap()
                    .clone();
                device.master().seed(0x4000, &[0; 256]);
                device.master().seed(0x3ff8, &[0x69; 8]);
                device.master().seed(0x4100, &[0x96; 8]);
                device
                    .erase_operation(&rflasher_core::flash::OptimalEraseOp {
                        start: 0x4000,
                        size: 256,
                        erase_block: eraser,
                    })
                    .await
                    .expect("actual 256-byte 81 erase");
                assert_eq!(device.master().memory(0x4000, 256), [0xff; 256]);
                assert_eq!(device.master().memory(0x3ff8, 8), [0x69; 8]);
                assert_eq!(device.master().memory(0x4100, 8), [0x96; 8]);
                device.master().raw(&[0x06], 0);
                device.master().raw(&[0x02, 0, 0x40, 1, 0x5a, 0, 0], 0);
                wait_ready(device.master());
                assert_eq!(device.master().memory(0x4000, 4), [0xff, 0x5a, 0xff, 0xff]);
            }
            if address > 0xff_ffff {
                assert_eq!(device.master().memory(address & 0xff_ffff, 32), [0x39; 32]);
                assert!(device.master().opcodes.contains(&0x12));
                assert!(device.master().opcodes.contains(&0x21));
                assert!(device.master().opcodes.contains(&0x13));
            }
        }
    });
}

#[test]
fn bp_lock_and_qe_off_initialization_remains_permissive() {
    block_on(async {
        for name in [
            "W25Q128.V",
            "N25Q128",
            "MX25L12833F",
            "GD25Q128",
            "AT25DF321",
        ] {
            let mut device = configured(name);
            let mask = profile(&device.context().chip)[2];
            let master = device.master();
            // No WREN or EWSR: status writes are harmless shadows, not gates.
            master.raw(&[0x01, 0xfc, 0xff], 0);
            assert_eq!(master.raw(&[0x05], 1)[0] & (mask | 0x80), 0);
            if name == "MX25L12833F" {
                assert_eq!(
                    master.raw(&[0x35], 1),
                    [0xdf],
                    "C2 config, active4B bit5 clear"
                );
                master.raw(&[0x01, 0xbc, 0x41], 0); // SR1 QE off; full C2 config shadow.
                assert_eq!(master.raw(&[0x05], 1)[0] & 0x40, 0);
                assert_eq!(master.raw(&[0x15], 1), [0x41], "C2 config alias, not SR3");
            } else {
                assert_eq!(master.raw(&[0x35], 1), [0xbe], "shadow except CMP/SRL");
                master.raw(&[0x31, 0x41], 0); // QE off, CMP/SRL attempts remain harmless.
                assert_eq!(master.raw(&[0x35], 1), [0]);
                master.raw(&[0x11, 0xa4], 0);
                assert_eq!(master.raw(&[0x15], 1), [0xa4], "SR3 shadow");
            }
            master.raw(&[0x36, 0, 0, 0], 0); // Atmel sector lock attempt.
            master.raw(&[0x7e], 0); // global lock attempt.
            if name == "AT25DF321" {
                assert_eq!(master.raw(&[0x3c, 0, 0, 0], 1), [0]);
            }
            // Raw PP proves the array itself is never protected, before any
            // driver's prepare/unlock helper has an opportunity to change it.
            master.raw(&[0x06], 0);
            master.raw(&[0x02, 0, 0x20, 1, 0x5a], 0);
            wait_ready(master);
            assert_eq!(master.memory(0x2001, 1), [0x5a]);
            mutate_and_verify(&mut device, 0).await;
        }
    });
}

#[test]
fn sst26_protection_writes_are_ignored_and_reads_unmasked() {
    block_on(async {
        for (name, len) in [
            ("SST26VF016B(A)", 6),
            ("SST26VF032B(A)", 10),
            ("SST26VF064B(A)", 18),
        ] {
            let mut device = configured(name);
            let capacity = device.context().chip.total_size;
            let master = device.master();
            master.raw(&[0x06], 0);
            let mut lock = vec![0x42];
            lock.extend(vec![0xff; len]);
            master.raw(&lock, 0);
            master.raw(&[0x36, 0, 0, 0], 0);
            master.raw(&[0x7e], 0);
            assert_eq!(master.raw(&[0x72], len + 2), vec![0; len + 2]);
            for start in [0x7ffe, capacity - 32770] {
                master.seed(start, &[0x11, 0x22, 0x33, 0x44]);
                assert_eq!(
                    master.raw(
                        &[0x03, (start >> 16) as u8, (start >> 8) as u8, start as u8],
                        4
                    ),
                    [0x11, 0x22, 0x33, 0x44]
                );
            }
            mutate_and_verify(&mut device, 0).await;
            assert_eq!(device.master().raw(&[0x72], len), vec![0; len]);
        }
    });
}

#[test]
fn uid_aai_and_byte_only_program_framing() {
    block_on(async {
        let mut winbond = configured("W25Q128.V");
        assert_eq!(winbond.read_unique_id().await.unwrap(), UID);
        assert_eq!(winbond.master().raw(&[0x4b, 0, 0, 0, 0], 8), UID);
        for name in ["SST25VF016B", "SST25VF032B"] {
            let mut device = configured(name);
            mutate_and_verify(&mut device, 0).await;
            assert!(
                device.master().opcodes.contains(&0xad),
                "real AAI path {name}"
            );
            let master = device.master();
            master.raw(&[0x01, 0xff], 0); // No EWSR requirement / BP gating.
            master.raw(&[0x06], 0);
            master.raw(&[0x02, 0, 0x20, 1, 0x5a, 0, 0], 0);
            wait_ready(master);
            assert_eq!(
                master.memory(0x2000, 4),
                [0xff, 0x5a, 0xff, 0xff],
                "byte-only 02 {name}"
            );
        }
    });
}

#[test]
fn micron_flag_status_ready_busy_and_config_shadows() {
    let mut device = configured("N25Q128");
    let master = device.master();
    assert_eq!(master.raw(&[0x70], 1), [0x80]);
    master.raw(&[0x81, 0x5a], 0);
    assert_eq!(master.raw(&[0x85], 1), [0x5a]);
    master.raw(&[0xb1, 0x34, 0x12], 0);
    assert_eq!(master.raw(&[0xb5], 2), [0x34, 0x12]);
    master.raw(&[0x61, 0xa5], 0);
    assert_eq!(master.raw(&[0x65], 1), [0xa5]); // 65 is config read, not reset.
    master.raw(&[0x06], 0);
    let mut pp = vec![0x02, 0, 0, 0];
    pp.extend([0x5a; 256]);
    master.raw(&pp, 0);
    assert_eq!(
        master.raw(&[0x70], 1),
        [0],
        "Micron busy must not report ready"
    );
    wait_ready(master);
    assert_eq!(master.raw(&[0x70], 1), [0x80]);
    assert_eq!(master.memory(0, 256), [0x5a; 256]);
}

// The byte-only bridge still captures every real SPI clock. Extract the data
// at its exact bit offset rather than rounding a six-clock wait to a byte.
fn read_after_dummy(
    master: &mut RtlMaster,
    header: &[u8],
    clocks: usize,
    length: usize,
) -> Vec<u8> {
    let rx = master.raw(header, (clocks + length * 8).div_ceil(8));
    (0..length)
        .map(|i| {
            (0..8).fold(0, |byte, bit| {
                let position = clocks + i * 8 + bit;
                (byte << 1) | ((rx[position / 8] >> (7 - position % 8)) & 1)
            })
        })
        .collect()
}

#[test]
fn micron_nvcr_reset_and_macronix_configuration_select_real_framing() {
    block_on(async {
        let mut device = configured("N25Q256");
        let master = device.master();
        let data: Vec<u8> = (0..17).map(|i| (i * 29 + 0x53) as u8).collect();
        master.seed(0x01000123, &data);
        master.seed(0x123, &[0x92; 17]);
        master.raw(&[0xb1, 0xfe, 0x6f], 0); // NVCR: active-low4B, six dummy clocks.
        master.raw(&[0x66], 0);
        master.raw(&[0x99], 0);
        assert_eq!(master.raw(&[0xb5], 4), [0xfe, 0x6f, 0xfe, 0x6f]);
        assert_eq!(master.raw(&[0x85], 1), [0x6b]);
        assert_eq!(master.raw(&[0x03, 1, 0, 1, 0x23], data.len()), data);
        assert_eq!(
            read_after_dummy(master, &[0x0b, 1, 0, 1, 0x23], 6, data.len()),
            data
        );
        for (code, clocks) in [(0, 8), (15, 8), (10, 10)] {
            let vcr = (code << 4) | 0x0b;
            master.raw(&[0x81, vcr], 0);
            assert_eq!(master.raw(&[0x85], 2), [vcr; 2]);
            assert_eq!(
                read_after_dummy(master, &[0x0b, 1, 0, 1, 0x23], clocks, data.len()),
                data
            );
        }
        master.raw(&[0x66], 0);
        master.raw(&[0x99], 0);
        assert_eq!(master.raw(&[0xb5], 2), [0xfe, 0x6f], "NVCR survives reset");
        assert_eq!(
            read_after_dummy(master, &[0x0b, 1, 0, 1, 0x23], 6, data.len()),
            data
        );
        master.raw(&[0xb1, 0xff, 0xff], 0);
        master.raw(&[0x66], 0);
        master.raw(&[0x99], 0);
        assert_eq!(
            master.raw(&[0x03, 0, 1, 0x23], 17),
            [0x92; 17],
            "NVCR bit0 restores3B"
        );

        let chip = chip("MX25L25635F");
        let mut p = profile(&chip);
        p[13] = 0x41; // Full initial C2 config allowed, not CMP/SRL protection.
        let mut device = configured_profile(chip, &p);
        let master = device.master();
        assert_eq!(master.raw(&[0x15], 2), [0x41; 2]);
        master.raw(&[0x01, 0x40, 0x41], 0); // SR1 QE6 plus second-byte config.
        assert_eq!(
            master.raw(&[0x05], 1)[0] & 0x40,
            0x40,
            "SR1 QE survives hints"
        );
        assert_eq!(master.raw(&[0x15], 1), [0x41]);
        master.seed(0x123, &data);
        master.seed(0x01000123, &[0x69; 17]);
        assert_eq!(
            read_after_dummy(master, &[0x0b, 0, 1, 0x23], 6, data.len()),
            data
        );
        master.raw(&[0xb7], 0);
        assert_eq!(
            master.raw(&[0x15], 1),
            [0x61],
            "C2 config reflects active4B"
        );
        assert_eq!(
            read_after_dummy(master, &[0x0b, 1, 0, 1, 0x23], 6, 17),
            [0x69; 17]
        );
        master.raw(&[0xe9], 0);
        assert_eq!(master.raw(&[0x15], 1), [0x41]);
        mutate_and_verify(&mut device, 4096).await;
    });
}

#[test]
fn spansion_addressed_configuration_init_polling_and_high_bank_mutation() {
    block_on(async {
        let mut device = configured("S25FL256L");
        let master = device.master();
        // CR1/QE and small config registers use addressed71/65, not EVCR65.
        for (index, value, expected) in
            [(2, 0x43, 2), (4, 0x15, 5), (5, 0xa5, 0xa5), (6, 0x5a, 0x5a)]
        {
            master.raw(&[0x71, 0, 0, index, value], 0);
            assert_eq!(master.raw(&[0x65, 0, 0, index], 2), [expected; 2]);
            assert_eq!(
                master.raw(&[0x65, 0, 0, index, 0], 2),
                [expected; 2],
                "whole-byte dummy is harmless"
            );
        }
        master.raw(&[0x71, 0, 0, 3, 0x8b], 0); // Active4B; CR2codeB selects24 clocks.
        assert_eq!(master.raw(&[0x65, 0, 0, 0, 3], 2), [0x8b; 2]);
        master.seed(0x01000123, &[0x96; 17]);
        master.seed(0x123, &[0x69; 17]);
        assert_eq!(
            read_after_dummy(master, &[0x0b, 1, 0, 1, 0x23], 24, 17),
            [0x96; 17]
        );
        master.raw(&[0xb8], 0);
        assert_eq!(master.raw(&[0x65, 0, 0, 3], 1), [0x0b]);
        master.raw(&[0xb7], 0);
        assert_eq!(master.raw(&[0x65, 0, 0, 0, 3], 1), [0x8b]);
        master.raw(&[0xe9], 0);
        assert_eq!(master.raw(&[0x65, 0, 0, 3], 1), [0x0b]);
        master.raw(&[0x71, 0, 0, 3, 0x80], 0); // Restore conventional latency, active4B.
        master.seed(0x01001000, &[0; 4096]);
        master.seed(0x1000, &[0x39; 32]);
        master.raw(&[0x06], 0);
        assert_eq!(
            master.raw(&[0x65, 0, 0, 0, 0], 1)[0] & 3,
            2,
            "addressed SR1 WEL"
        );
        master.raw(&[0x21, 1, 0, 0x10, 0], 0);
        assert_eq!(
            master.raw(&[0x65, 0, 0, 0, 0], 1)[0] & 3,
            1,
            "addressed SR1 busy"
        );
        assert_eq!(
            master.raw(&[0x65, 0, 0, 0, 0, 0], 1)[0] & 3,
            1,
            "busy polling with8 dummy clocks"
        );
        master.raw(&[0x71, 0, 0, 0, 0, 0], 0); // Status shadow cannot clear core WIP.
        assert_eq!(
            master.raw(&[0x65, 0, 0, 0, 0], 1)[0] & 3,
            1,
            "WIP survives status write"
        );
        wait_ready(master);
        assert_eq!(master.raw(&[0x65, 0, 0, 0, 0], 2), [0; 2]);
        assert_eq!(master.memory(0x01001000, 4096), [0xff; 4096]);
        assert_eq!(master.memory(0x1000, 32), [0x39; 32]);
        master.seed(0x2000, &[0x39; 32]);
        mutate_and_verify(&mut device, 0x01002000).await;
        assert_eq!(device.master().memory(0x2000, 32), [0x39; 32]);
    });
}

#[test]
fn reset_66_99_and_program_address_data_boundaries() {
    let mut device = configured("W25Q128.V");
    let master = device.master();
    master.raw(&[0x06], 0);
    master.raw(&[0x66], 0);
    master.raw(&[0x99], 0);
    assert_eq!(master.raw(&[0x05], 1)[0] & 3, 0, "reset clears WEL/WIP");
    master.raw(&[0x02, 0, 0, 1, 0], 0);
    master.request("D 100");
    assert_eq!(master.memory(1, 1), [0xff], "reset must clear WEL");
    // Truncated address / address without data cannot become a mutation.
    for incomplete in [&[0x02, 0, 0][..], &[0x02, 0, 0, 1][..]] {
        master.raw(&[0x06], 0);
        master.raw(incomplete, 0);
        master.request("D 100");
        assert_eq!(master.memory(0, 8), [0xff; 8]);
    }
    master.raw(&[0x06], 0);
    master.raw(&[0x02, 0, 0, 0xfe, 0x5a, 0xa5, 0x3c, 0xc3], 0);
    wait_ready(master);
    assert_eq!(master.memory(0xfe, 4), [0x5a, 0xa5, 0xff, 0xff]);
    assert_eq!(
        master.memory(0, 2),
        [0x3c, 0xc3],
        "page wrap, not address/data slip"
    );
    assert_eq!(master.raw(&[0x03, 0, 0, 0xfe], 4), [0x5a, 0xa5, 0xff, 0xff]);
    // Explicit synthetic sub-256 page shape: never silently round to 256.
    let mut small_page = chip("W25Q128.V");
    small_page.page_size = 128;
    let mut device = configured_profile(small_page.clone(), &profile(&small_page));
    let master = device.master();
    master.raw(&[0x06], 0);
    master.raw(&[0x02, 0, 0, 0x7e, 0x5a, 0xa5, 0x3c, 0xc3], 0);
    wait_ready(master);
    assert_eq!(master.memory(0x7e, 4), [0x5a, 0xa5, 0xff, 0xff]);
    assert_eq!(master.memory(0, 2), [0x3c, 0xc3], "128-byte page wrap");
}

#[test]
fn stop_cancels_pending_mutations_and_page_bytes_before_reconfiguration() {
    for opcode in [0x02, 0x20, 0xc7] {
        let mut device = configured("W25Q128.V");
        let master = device.master();
        master.raw(&[0x06], 0);
        let pending = match opcode {
            0x02 => vec![opcode, 0, 0, 0, 0],
            0x20 => vec![opcode, 0, 0, 0],
            _ => vec![opcode],
        };
        master.request(&format!("B {}", hex(&pending)));
        assert_eq!(master.host(&[0x35]), [1]);
        let mut config = vec![0x3c];
        config.extend(profile(&chip("W25Q128.V")));
        assert_eq!(master.host(&config), [1]);
        let mut load = vec![0x32, 0, 0, 0, 0, 1];
        load.extend([0xa5; 8]);
        assert_eq!(master.host(&load), [1]);
        master.request("C");
        master.request("D 100");
        assert_eq!(master.memory(0, 8), [0xa5; 8], "replayed {opcode:02x}");
        assert_eq!(master.host(&[0x34]), [1]);
        master.raw(&[0x06], 0);
        master.raw(&[0x02, 0, 1, 1, 0x3c], 0);
        wait_ready(master);
        assert_eq!(
            master.memory(0x100, 2),
            [0xff, 0x3c],
            "stale page flag {opcode:02x}"
        );
    }
}

#[test]
fn stop_ack_drains_an_accepted_program_before_uart_ram_loading() {
    let mut device = configured("W25Q128.V");
    let master = device.master();
    master.raw(&[0x06], 0);
    master.raw(&[0x02, 0, 0, 0, 0x5a], 0);
    assert_eq!(master.host(&[0x35]), [1]);
    assert_eq!(
        master.memory(0, 1),
        [0x5a],
        "STOP ACK preceded program completion"
    );
    let mut load = vec![0x32, 0, 0, 0, 0, 1];
    load.extend([0xa5; 8]);
    assert_eq!(master.host(&load), [1]);
    master.request("D 100");
    assert_eq!(master.memory(0, 8), [0xa5; 8]);
}

#[test]
fn real_sfdp_probe_reads_uploaded_bfpt_without_high_address_aliases() {
    block_on(async {
        let mut device = configured("W25Q128.V");
        let chip = device.context().chip.clone();
        let master = device.master();
        assert_eq!(master.host(&[0x35]), [1]);
        let sfdp = include_bytes!("../../../tool/tests/fixtures/w25q128-v.sfdp");
        let bursts = chip.total_size / 8 - 1;
        let mut config = vec![
            0x33,
            chip.jedec_manufacturer,
            (chip.jedec_device >> 8) as u8,
            chip.jedec_device as u8,
            0,
            (bursts >> 16) as u8,
            (bursts >> 8) as u8,
            bursts as u8,
            sfdp.len() as u8,
        ];
        config.extend(sfdp);
        assert_eq!(master.host(&config), [1]);
        let mut config = vec![0x3c];
        config.extend(profile(&chip));
        assert_eq!(master.host(&config), [1]);
        assert_eq!(master.host(&[0x34]), [1]);
        let info = rflasher_core::sfdp::probe(master).await.unwrap();
        assert_eq!(info.basic_params.density_bytes, u64::from(chip.total_size));
        assert_eq!(info.page_size(), 256);
        let mut actual = [0; 80];
        rflasher_core::sfdp::read_sfdp(master, 0, &mut actual)
            .await
            .unwrap();
        assert_eq!(actual.as_slice(), sfdp);
        for address in [79, 80, 127, 128, 0x1000, 0x010000] {
            let mut blank = [0; 8];
            rflasher_core::sfdp::read_sfdp(master, address, &mut blank)
                .await
                .unwrap();
            if address == 79 {
                assert_eq!(blank[0], sfdp[79]);
                assert_eq!(&blank[1..], &[0xff; 7]);
            } else {
                assert_eq!(blank, [0xff; 8], "SFDP alias {address:x}");
            }
        }
    });
}

#[test]
fn malformed_complete_profiles_fail_closed_including_last_byte() {
    let mut device = configured("W25Q128.V");
    let master = device.master();
    assert_eq!(master.host(&[0x35]), [1]);
    let valid = profile(&chip("W25Q128.V"));
    for (index, value) in [
        (0, 1),
        (1, 0xff),
        (2, valid[2] | 1),
        (2, valid[2] | 2),
        (7, valid[7] | 1),
        (7, valid[7] | 2),
        (12, 0x83),
        (13, 0x41),
        (15, 1),
        (25, 1),
        (26, 1),
        (27, 1),
        (28, 9),
        (34, 6),
        (39, 0x80),
        (32, 3),
        (127, 1),
    ] {
        let mut p = valid;
        p[index] = value;
        let mut config = vec![0x3c];
        config.extend(p);
        assert_eq!(master.host(&config), [2], "invalid byte {index}");
        assert_eq!(master.host(&[0x34]), [2], "START after byte {index}");
    }
    let mut config = vec![0x3c];
    config.extend(valid);
    assert_eq!(master.host(&config), [1]);
    assert_eq!(master.host(&[0x34]), [1]);
}

#[test]
fn custom_50_nonuniform_eraser_preserves_region_boundaries() {
    use rflasher_core::flash::OptimalEraseOp;
    block_on(async {
        let mut chip = chip("SST26VF016B(A)");
        // Keep every pinned asymmetric region; only replace the command byte.
        chip.erase_blocks
            .iter_mut()
            .find(|b| b.opcode == 0xd8)
            .unwrap()
            .opcode = 0x50;
        let mut p = profile(&chip);
        p[1] = 0;
        chip.unlock = Unlock::None;
        chip.features.remove(Features::SST26_BPR);
        let mut device = configured_profile(chip, &p);
        let eraser = device
            .erase_blocks()
            .iter()
            .find(|b| b.opcode == 0x50)
            .unwrap()
            .clone();
        for (start, size) in [
            (0x6000u32, 8192u32),
            (0x8000, 32768),
            (0x10000, 65536),
            (0x1f8000, 8192),
        ] {
            device.master().seed(start, &vec![0; size as usize]);
            device.master().seed(start - 8, &[0x69; 8]);
            device.master().seed(start + size, &[0x96; 8]);
            let operation = OptimalEraseOp {
                start,
                size,
                erase_block: eraser.clone(),
            };
            device
                .erase_operation(&operation)
                .await
                .expect("custom addressed eraser");
            assert_eq!(
                device.master().memory(start, size as usize),
                vec![0xff; size as usize]
            );
            assert_eq!(device.master().memory(start - 8, 8), [0x69; 8]);
            assert_eq!(device.master().memory(start + size, 8), [0x96; 8]);
        }
        assert!(device.master().opcodes.contains(&0x50));
    });
}

#[test]
fn extended_address_and_ear7_helpers_select_real_array_banks() {
    use rflasher_chips::EraseBlock;
    use rflasher_core::{programmer::SpiMaster, protocol, spi::SpiCommand};
    block_on(async {
        for (capability, flag, read_opcode) in [
            (Features::EXT_ADDR_REG_C5C8, 8, 0xc8),
            (Features::EXT_ADDR_REG_1716, 16, 0x16),
        ] {
            // Test-only 32MiB shape; not a claim that W25Q128 itself has EAR.
            let mut chip = chip("W25Q128.V");
            chip.total_size = 32 * 1024 * 1024;
            chip.erase_blocks = vec![EraseBlock::with_count(0x20, 4096, 8192)];
            let mut p = profile(&chip);
            p[10] = flag | 4;
            let mut device = configured_profile(chip, &p);
            device.master().seed(0x123, &[0x19; 32]);
            device.master().seed(0x01000123, &[0x92; 32]);
            protocol::set_extended_address(device.master(), capability, 1)
                .await
                .unwrap();
            assert_eq!(device.master().raw(&[read_opcode], 1), [1]);
            let mut data = [0; 32];
            protocol::read_3b(device.master(), 0x123, &mut data)
                .await
                .unwrap();
            assert_eq!(data, [0x92; 32]);
            protocol::set_extended_address(device.master(), capability, 0)
                .await
                .unwrap();
            protocol::read_3b(device.master(), 0x123, &mut data)
                .await
                .unwrap();
            assert_eq!(data, [0x19; 32]);
            let mode = capability | Features::FOUR_BYTE_ENTER_EAR7;
            protocol::enter_4byte_mode_with_features(device.master(), mode)
                .await
                .unwrap();
            device
                .master()
                .execute(&mut SpiCommand::read_4b(0x03, 0x01000123, &mut data))
                .await
                .unwrap();
            assert_eq!(data, [0x92; 32]);
            protocol::exit_4byte_mode_with_features(device.master(), mode)
                .await
                .unwrap();
            protocol::read_3b(device.master(), 0x123, &mut data)
                .await
                .unwrap();
            assert_eq!(data, [0x19; 32]);
        }
    });
}

#[test]
fn partial_profile_times_out_and_start_refuses_until_reconfigured() {
    let mut device = configured("W25Q128.V");
    assert_eq!(device.master().host(&[0x35]), [1]);
    assert!(device.master().host(&[0x3c, 2, 0]).is_empty());
    device.master().request("D 1000");
    assert_eq!(device.master().host(&[0x34]), [2]);
    let p = profile(&device.context().chip);
    let mut bytes = vec![0x3c];
    bytes.extend(p);
    assert_eq!(device.master().host(&bytes), [1]);
    assert_eq!(device.master().host(&[0x34]), [1]);
    assert_eq!(device.master().raw(&[0x05], 1), [0]);
}
