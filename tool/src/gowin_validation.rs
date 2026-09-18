// SPDX-License-Identifier: Apache-2.0
// Derived from openFPGALoader's Gowin bitstream parser.
// Copyright (C) 2019 Gwenhael Goavec-Merou and openFPGALoader contributors.

#[derive(Debug, Clone)]
pub(crate) struct GowinBitstream {
    pub(crate) idcode: u32,
    pub(crate) checksum: u32,
    pub(crate) sram_data: Vec<u8>,
    pub(crate) flash_data: Vec<u8>,
}

impl GowinBitstream {
    pub(crate) fn parse(input: &[u8], sram_reverse_bytes: bool) -> Result<Self, String> {
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
        for chunk in line.as_bytes().as_chunks::<8>().0 {
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
    if options.conf_data_len == 0 {
        return Err("Gowin .fs file declares no configuration data".to_string());
    }
    let data_lines = lines
        .get(options.end_header + 1..)
        .and_then(|lines| lines.get(..options.conf_data_len))
        .ok_or("truncated Gowin .fs configuration data: fewer lines than ConfDataLength")?;
    let drop_bits = if options.crc_check { 8 * 8 } else { 6 * 8 };
    let mut checksum_bits = String::new();

    for line in data_lines {
        if line.len() <= drop_bits {
            return Err("truncated Gowin .fs configuration line".to_string());
        }
        if options.compressed {
            let payload = &line[..line.len() - drop_bits];
            if payload.len() % 8 != 0 {
                return Err("compressed Gowin .fs line is not byte aligned".to_string());
            }
            for chunk in payload.as_bytes().as_chunks::<8>().0 {
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
    for chunk in checksum_bits.as_bytes().as_chunks::<16>().0 {
        checksum = checksum.wrapping_add(bit_to_val(chunk) as u32 & 0xffff);
    }
    Ok(checksum & 0xffff)
}

fn reverse_byte(byte: u8) -> u8 {
    byte.reverse_bits()
}

/// Decode DWORD 2 of the JEDEC SFDP Basic Flash Parameter Table.
pub(crate) fn flash_capacity(density: u32) -> Result<u64, String> {
    let bits = if density & (1 << 31) == 0 {
        u64::from(density) + 1
    } else {
        1u64.checked_shl(density & 0x7fff_ffff)
            .ok_or("invalid SFDP flash density exponent")?
    };
    if bits < 8 || !bits.is_multiple_of(8) {
        return Err("invalid SFDP flash density".to_string());
    }
    Ok(bits / 8)
}

pub(crate) fn find_erase_opcode(descriptors: &[u8], erase_size: u64) -> Result<Option<u8>, String> {
    if descriptors.len() != 8 {
        return Err("invalid SFDP erase descriptor length".to_string());
    }
    Ok(descriptors
        .as_chunks::<2>()
        .0
        .iter()
        .find_map(|descriptor| {
            let exponent = u32::from(descriptor[0]);
            let opcode = descriptor[1];
            (opcode != 0 && opcode != 0xff && 1u64.checked_shl(exponent) == Some(erase_size))
                .then_some(opcode)
        }))
}

pub(crate) fn validate_flash_range(offset: u32, len: usize, capacity: u64) -> Result<u32, String> {
    // This programmer only sends three-byte addresses and erases 4 KiB sectors.
    let limit = capacity.min(0x0100_0000);
    let end = u64::from(offset)
        .checked_add(len as u64)
        .ok_or("flash range overflow")?;
    let erase_end = end.checked_add(0xfff).ok_or("flash erase range overflow")? & !0xfff;
    if len == 0 || u64::from(offset) >= limit || erase_end > limit {
        return Err(format!(
            "flash range 0x{offset:06x}..0x{end:06x} (including sector erase) exceeds usable capacity 0x{limit:x}"
        ));
    }
    Ok(end as u32)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn image(declared: u16, rows: usize) -> String {
        let header = format!(
            "{:064b}\n{:032b}\n",
            0x060000000001281bu64,
            0x3b000000u32 | u32::from(declared)
        );
        header + &format!("{:064b}\n", 1u64 << 48).repeat(rows)
    }

    #[test]
    fn rejects_missing_configuration_rows() {
        for reverse in [false, true] {
            for text in [
                image(10, 0),
                image(10, 9),
                image(0, 0),
                image(1, 0) + "\n" + &image(1, 1),
            ] {
                assert!(GowinBitstream::parse(text.as_bytes(), reverse).is_err());
            }
        }
    }

    #[test]
    fn accepts_complete_image_with_trailer() {
        let text = image(2, 2) + "11111111\n";
        for reverse in [false, true] {
            let parsed = GowinBitstream::parse(text.as_bytes(), reverse).unwrap();
            assert_eq!(parsed.idcode, 0x0001281b);
            assert_eq!(parsed.checksum, 2);
            assert_eq!(parsed.flash_data.len(), 29);
            assert_eq!(parsed.flash_data.last(), Some(&0xff));
            assert_eq!(parsed.sram_data[0], if reverse { 0x60 } else { 0x06 });
        }
    }

    #[test]
    fn decodes_sfdp_density_encodings() {
        assert_eq!(flash_capacity(0x03ff_ffff).unwrap(), 8 * 1024 * 1024);
        assert_eq!(flash_capacity(0x8000_001a).unwrap(), 8 * 1024 * 1024);
        assert!(flash_capacity(0).is_err());
        assert!(flash_capacity(0x8000_0040).is_err());
    }

    #[test]
    fn selects_only_the_requested_sfdp_erase_size() {
        let descriptors = [12, 0x20, 15, 0x52, 16, 0xd8, 0, 0xff];
        assert_eq!(find_erase_opcode(&descriptors, 4096).unwrap(), Some(0x20));
        assert_eq!(find_erase_opcode(&descriptors, 32768).unwrap(), Some(0x52));
        assert_eq!(find_erase_opcode(&descriptors, 65536).unwrap(), Some(0xd8));
        assert_eq!(find_erase_opcode(&descriptors, 8192).unwrap(), None);
        assert!(find_erase_opcode(&descriptors[..6], 4096).is_err());
    }

    #[test]
    fn bounds_writes_and_erases_by_capacity_and_address_width() {
        let capacity = 8 * 1024 * 1024;
        assert_eq!(
            validate_flash_range(0x7fff00, 256, capacity).unwrap(),
            0x800000
        );
        assert!(validate_flash_range(0x800000, 256, capacity).is_err());
        assert!(validate_flash_range(0x7fff00, 257, capacity).is_err());
        assert!(validate_flash_range(0, 0, capacity).is_err());
        assert!(validate_flash_range(0x1000000, 1, 32 * 1024 * 1024).is_err());
        assert!(validate_flash_range(0, 1, 1024).is_err());
        assert!(validate_flash_range(u32::MAX, usize::MAX, capacity).is_err());
    }
}
