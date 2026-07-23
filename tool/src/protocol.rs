//! Host-side definitions of the byte-level FPGA protocol.
//!
//! These types intentionally contain only byte arrays and `u8` fields. This
//! makes their `repr(C)` layout identical to the wire format, while zerocopy's
//! derives verify that the complete value can safely be viewed as bytes.

use zerocopy::{Immutable, IntoBytes};

pub const PROTOCOL_VERSION: u8 = 5;

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

pub const LOG_POLL_TERMINATOR: u8 = 0xA0;
pub const LOG_POLL_ESCAPE: u8 = 0xA5;

pub const TOCTOU_ARM: u8 = 0x02;
pub const TOCTOU_DISARM: u8 = 0x03;
pub const TOCTOU_RESET: u8 = 0x04;
pub const TOCTOU_RESET_ALL: u8 = 0x05;

pub const LOG_CMD: u8 = 0xA1;
pub const LOG_ADDR: u8 = 0xA2;
pub const LOG_END: u8 = 0xA3;
pub const LOG_TRAP: u8 = 0xA4;

#[derive(Clone, Copy, Debug, Eq, Immutable, IntoBytes, PartialEq)]
#[repr(transparent)]
struct BeU16([u8; 2]);

impl BeU16 {
    const fn new(value: u16) -> Self {
        Self(value.to_be_bytes())
    }
}

#[derive(Clone, Copy, Debug, Eq, Immutable, IntoBytes, PartialEq)]
#[repr(transparent)]
struct BeU24([u8; 3]);

impl BeU24 {
    const MAX: u32 = 0x00ff_ffff;

    const fn new(value: u32) -> Self {
        let bytes = value.to_be_bytes();
        Self([bytes[1], bytes[2], bytes[3]])
    }
}

#[derive(Clone, Copy, Debug, Eq, Immutable, IntoBytes, PartialEq)]
#[repr(C)]
pub struct RamHeader {
    command: u8,
    address_bursts: BeU24,
    length_bursts: BeU16,
}

impl RamHeader {
    pub fn read(address_bursts: u32, length_bursts: u16) -> Option<Self> {
        Self::new(CMD_READ, address_bursts, length_bursts)
    }

    pub fn write(address_bursts: u32, length_bursts: u16) -> Option<Self> {
        Self::new(CMD_WRITE, address_bursts, length_bursts)
    }

    fn new(command: u8, address_bursts: u32, length_bursts: u16) -> Option<Self> {
        (address_bursts <= BeU24::MAX).then(|| Self {
            command,
            address_bursts: BeU24::new(address_bursts),
            length_bursts: BeU16::new(length_bursts),
        })
    }
}

#[derive(Clone, Copy, Debug, Eq, Immutable, IntoBytes, PartialEq)]
#[repr(C)]
pub struct ChipConfigHeader {
    command: u8,
    jedec_id: [u8; 3],
    flags: u8,
    erase_bursts: BeU24,
    sfdp_length: u8,
}

impl ChipConfigHeader {
    pub fn new(jedec_id: [u8; 3], flags: u8, erase_bursts: u32, sfdp_length: u8) -> Option<Self> {
        // The FPGA stores this field in 23 bits; bit 23 is not part of the
        // count even though the wire representation occupies three bytes.
        (erase_bursts <= 0x007f_ffff).then(|| Self {
            command: CMD_CHIPCONFIG,
            jedec_id,
            flags,
            erase_bursts: BeU24::new(erase_bursts),
            sfdp_length,
        })
    }
}

#[derive(Clone, Copy, Debug, Eq, Immutable, IntoBytes, PartialEq)]
#[repr(C)]
pub struct ControlRequest {
    command: u8,
    value: u8,
}

impl ControlRequest {
    pub const fn hold(enable: bool) -> Self {
        Self {
            command: CMD_HOLDCTL,
            value: enable as u8,
        }
    }

    pub const fn log(enable: bool) -> Self {
        Self {
            command: CMD_LOGCTL,
            value: enable as u8,
        }
    }

    pub const fn toctou_reset_all() -> Self {
        Self {
            command: CMD_TOCTOU,
            value: TOCTOU_RESET_ALL,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, Immutable, IntoBytes, PartialEq)]
#[repr(C)]
pub struct ToctouIndexRequest {
    command: u8,
    subcommand: u8,
    index: u8,
}

impl ToctouIndexRequest {
    pub const fn arm(index: u8) -> Self {
        Self::new(TOCTOU_ARM, index)
    }

    pub const fn disarm(index: u8) -> Self {
        Self::new(TOCTOU_DISARM, index)
    }

    pub const fn reset(index: u8) -> Self {
        Self::new(TOCTOU_RESET, index)
    }

    const fn new(subcommand: u8, index: u8) -> Self {
        Self {
            command: CMD_TOCTOU,
            subcommand,
            index: index & 0x03,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, Immutable, IntoBytes, PartialEq)]
#[repr(C)]
pub struct ToctouSetRequest {
    command: u8,
    subcommand: u8,
    index: u8,
    start: BeU24,
    mask: BeU24,
    replace: BeU24,
}

impl ToctouSetRequest {
    pub fn new(index: u8, start: u32, mask: u32, replace: u32) -> Option<Self> {
        (start <= BeU24::MAX && mask <= BeU24::MAX && replace <= BeU24::MAX).then(|| Self {
            command: CMD_TOCTOU,
            subcommand: 0x01,
            index: index & 0x03,
            start: BeU24::new(start),
            mask: BeU24::new(mask),
            replace: BeU24::new(replace),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wire_struct_sizes_have_no_padding() {
        assert_eq!(size_of::<RamHeader>(), 6);
        assert_eq!(size_of::<ChipConfigHeader>(), 9);
        assert_eq!(size_of::<ControlRequest>(), 2);
        assert_eq!(size_of::<ToctouIndexRequest>(), 3);
        assert_eq!(size_of::<ToctouSetRequest>(), 12);
    }

    #[test]
    fn ram_headers_match_the_existing_wire_format() {
        assert_eq!(
            RamHeader::read(0x12_3456, 0x789a).unwrap().as_bytes(),
            &[CMD_READ, 0x12, 0x34, 0x56, 0x78, 0x9a]
        );
        assert_eq!(
            RamHeader::write(0x00_00ff, 0xffff).unwrap().as_bytes(),
            &[CMD_WRITE, 0x00, 0x00, 0xff, 0xff, 0xff]
        );
        assert!(RamHeader::read(0x0100_0000, 1).is_none());
    }

    #[test]
    fn chip_config_header_matches_the_existing_wire_format() {
        let header = ChipConfigHeader::new([0xef, 0x40, 0x18], 1, 0x12_3456, 128).unwrap();
        assert_eq!(
            header.as_bytes(),
            &[CMD_CHIPCONFIG, 0xef, 0x40, 0x18, 1, 0x12, 0x34, 0x56, 128]
        );
        assert!(ChipConfigHeader::new([0; 3], 0, 0x0080_0000, 0).is_none());
    }

    #[test]
    fn control_requests_match_the_existing_wire_format() {
        assert_eq!(ControlRequest::hold(true).as_bytes(), &[CMD_HOLDCTL, 1]);
        assert_eq!(ControlRequest::log(false).as_bytes(), &[CMD_LOGCTL, 0]);
        assert_eq!(
            ControlRequest::toctou_reset_all().as_bytes(),
            &[CMD_TOCTOU, TOCTOU_RESET_ALL]
        );
        assert_eq!(
            ToctouIndexRequest::arm(5).as_bytes(),
            &[CMD_TOCTOU, TOCTOU_ARM, 1]
        );
    }

    #[test]
    fn toctou_set_request_matches_the_existing_wire_format() {
        let request = ToctouSetRequest::new(5, 0x12_3456, 0xff_f000, 0xab_cdef).unwrap();
        assert_eq!(
            request.as_bytes(),
            &[
                CMD_TOCTOU, 0x01, 0x01, 0x12, 0x34, 0x56, 0xff, 0xf0, 0x00, 0xab, 0xcd, 0xef,
            ]
        );
        assert!(ToctouSetRequest::new(0, 0x0100_0000, 0, 0).is_none());
    }
}
