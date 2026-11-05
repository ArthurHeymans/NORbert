//! Flash chip database access backed by rflasher.

use anyhow::{Context, Result, bail};
use rflasher_chips::WriteGranularity;
pub use rflasher_chips::{ChipDatabase, EraseBlock, Features, FlashChip};
use std::path::Path;

pub trait FlashChipExt {
    /// Full 3-byte JEDEC ID: [manufacturer, device_hi, device_lo].
    fn jedec_id_bytes(&self) -> [u8; 3];

    /// Whether the chip supports 4-byte addressing as advertised by rflasher.
    fn supports_4byte(&self) -> bool;

    fn supports_dual(&self) -> bool;
    fn supports_quad(&self) -> bool;
    fn aai_word(&self) -> bool;
    fn write_byte(&self) -> bool;

    /// Whether the emulated chip should expose an SFDP table.
    fn supports_sfdp(&self) -> bool;

    /// Erase operations sorted by block size, excluding chip erase.
    fn sector_erase_ops(&self) -> Vec<&EraseBlock>;
}

impl FlashChipExt for FlashChip {
    fn jedec_id_bytes(&self) -> [u8; 3] {
        [
            self.jedec_manufacturer,
            (self.jedec_device >> 8) as u8,
            self.jedec_device as u8,
        ]
    }

    fn supports_4byte(&self) -> bool {
        self.features.intersects(
            Features::FOUR_BYTE_ADDR
                | Features::FOUR_BYTE_ENTER
                | Features::FOUR_BYTE_ENTER_WREN
                | Features::FOUR_BYTE_ENTER_EAR7
                | Features::FOUR_BYTE_READ,
        )
    }

    fn supports_dual(&self) -> bool {
        self.features.contains(Features::DUAL_IO)
    }

    fn supports_quad(&self) -> bool {
        self.features.contains(Features::QUAD_IO)
    }

    fn aai_word(&self) -> bool {
        self.features.contains(Features::AAI_WORD)
    }

    fn write_byte(&self) -> bool {
        self.features.contains(Features::WRITE_BYTE)
            || self.write_granularity == WriteGranularity::Byte
    }

    fn supports_sfdp(&self) -> bool {
        // No catalog-wide guess: only explicit metadata or these audited
        // modern Winbond identities can expose our synthesized BFPT.
        let audited = self.jedec_manufacturer == 0xef
            && matches!(
                self.name.as_str(),
                "W25Q128.V" | "W25Q64.V" | "W25Q64BV/W25Q64CV/W25Q64FV" | "W25Q64JV-.Q"
            );
        !self.features.contains(Features::AT25F_ID)
            && !self.name.starts_with("SST25")
            && (self.total_size <= 16 * 1024 * 1024 || self.features.supports_4ba_mode_switch())
            && (self.features.contains(Features::SFDP) || audited)
            && self
                .erase_blocks
                .iter()
                .filter(|erase| !erase.is_chip_erase())
                .all(|erase| erase.is_uniform())
    }

    fn sector_erase_ops(&self) -> Vec<&EraseBlock> {
        let mut ops: Vec<&EraseBlock> = self
            .erase_blocks
            .iter()
            .filter(|erase| !erase.is_chip_erase())
            .collect();
        ops.sort_by_key(|erase| erase_block_size(erase).unwrap_or(0));
        ops
    }
}

/// Return a block size only when the complete operation is uniform.
pub fn erase_block_size(erase: &EraseBlock) -> Option<u32> {
    erase.uniform_size()
}

/// Load rflasher's compiled-in chip database, or an explicit directory override.
pub fn load_chip_db(dir: Option<&Path>) -> Result<ChipDatabase> {
    match dir {
        Some(dir) => ChipDatabase::from_dir(dir)
            .with_context(|| format!("Failed to load chip database directory: {}", dir.display())),
        None => Ok(ChipDatabase::new()),
    }
}

/// Find a chip by name (case-insensitive exact or substring match).
pub fn find_chip_by_name<'a>(db: &'a ChipDatabase, name: &str) -> Result<&'a FlashChip> {
    let name_lower = name.to_lowercase();

    if let Some(exact) = db.iter().find(|chip| chip.name.eq_ignore_ascii_case(name)) {
        return Ok(exact);
    }

    let matches: Vec<_> = db
        .iter()
        .filter(|chip| chip.name.to_lowercase().contains(&name_lower))
        .collect();

    match matches.len() {
        0 => bail!("No chip found matching '{}'", name),
        1 => Ok(matches[0]),
        n => {
            let names: Vec<_> = matches
                .iter()
                .map(|chip| format!("  {} ({})", chip.name, chip.vendor))
                .collect();
            bail!(
                "Ambiguous chip name '{}', {} matches:\n{}",
                name,
                n,
                names.join("\n")
            );
        }
    }
}
