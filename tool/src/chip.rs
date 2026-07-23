//! Flash chip database access backed by rflasher.

use anyhow::{Context, Result, bail};
pub use rflasher_chips::{ChipDatabase, EraseBlock, Features, FlashChip};
use std::path::Path;

pub trait FlashChipExt {
    /// Full 3-byte JEDEC ID: [manufacturer, device_hi, device_lo].
    fn jedec_id_bytes(&self) -> [u8; 3];

    /// Number of 8-byte SDRAM bursts needed to erase the whole chip, minus 1.
    fn chip_erase_bursts(&self) -> u32;

    /// Whether the chip supports 4-byte addressing as advertised by rflasher.
    fn supports_4byte(&self) -> bool;

    fn supports_dual(&self) -> bool;
    fn supports_quad(&self) -> bool;
    fn aai_word(&self) -> bool;
    fn write_byte(&self) -> bool;
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

    fn chip_erase_bursts(&self) -> u32 {
        (self.total_size / 8).saturating_sub(1)
    }

    fn supports_4byte(&self) -> bool {
        self.features
            .intersects(Features::FOUR_BYTE_ADDR | Features::FOUR_BYTE_ENTER)
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
    }

    fn supports_sfdp(&self) -> bool {
        self.features.contains(Features::SFDP)
    }

    fn sector_erase_ops(&self) -> Vec<&EraseBlock> {
        let mut ops: Vec<&EraseBlock> = self
            .erase_blocks
            .iter()
            .filter(|erase| {
                erase_block_size(erase).is_some_and(|block_size| block_size < self.total_size)
            })
            .collect();
        ops.sort_by_key(|erase| erase_block_size(erase).unwrap_or(0));
        ops
    }
}

/// Return the first region's block size, matching NORbert's uniform-chip use.
pub fn erase_block_size(erase: &EraseBlock) -> Option<u32> {
    erase.regions.first().map(|region| region.size)
}

/// Load rflasher's compiled-in chip database, or an explicit directory override.
pub fn load_chip_db(dir: Option<&Path>) -> Result<ChipDatabase> {
    match dir {
        Some(dir) => ChipDatabase::from_dir(dir).with_context(|| {
            format!("Failed to load chip database directory: {}", dir.display())
        }),
        None => Ok(ChipDatabase::new()),
    }
}

/// Find a chip by name (case-insensitive exact or substring match).
pub fn find_chip_by_name<'a>(db: &'a ChipDatabase, name: &str) -> Result<&'a FlashChip> {
    let name_lower = name.to_lowercase();

    let exact: Vec<_> = db
        .iter()
        .filter(|chip| chip.name.to_lowercase() == name_lower)
        .collect();
    if exact.len() == 1 {
        return Ok(exact[0]);
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
