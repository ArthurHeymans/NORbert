//! Chip definition types for loading rflasher-compatible RON files.
//!
//! These types mirror the RON schema used by rflasher's chip database,
//! allowing NORbert to reuse the same chip definition files.

#![allow(dead_code)]

use anyhow::{bail, Context, Result};
use serde::Deserialize;
use std::path::Path;

// ---------------------------------------------------------------------------
// RON deserialization types (must match rflasher's schema exactly)
// ---------------------------------------------------------------------------

#[derive(Debug, Deserialize)]
pub struct VendorDef {
    pub vendor: String,
    pub manufacturer_id: u8,
    pub chips: Vec<ChipDef>,
}

#[derive(Debug, Deserialize)]
pub struct ChipDef {
    pub name: String,
    pub device_id: u16,
    pub total_size: Size,
    #[serde(default = "default_page_size")]
    pub page_size: u16,
    #[serde(default)]
    pub features: FeaturesDef,
    #[serde(default)]
    pub voltage: VoltageDef,
    #[serde(default)]
    pub write_granularity: WriteGranularityDef,
    pub erase_blocks: Vec<EraseBlockDef>,
    #[serde(default)]
    pub tested: TestStatusesDef,
}

fn default_page_size() -> u16 {
    256
}

#[derive(Debug, Clone, Copy, Deserialize)]
pub enum Size {
    B(u32),
    KiB(u32),
    MiB(u32),
}

impl Size {
    pub fn to_bytes(self) -> u32 {
        match self {
            Size::B(n) => n,
            Size::KiB(n) => n * 1024,
            Size::MiB(n) => n * 1024 * 1024,
        }
    }
}

#[derive(Debug, Default, Deserialize)]
pub struct FeaturesDef {
    #[serde(default)]
    pub wrsr_wren: bool,
    #[serde(default)]
    pub wrsr_ewsr: bool,
    #[serde(default)]
    pub wrsr_ext: bool,
    #[serde(default)]
    pub fast_read: bool,
    #[serde(default)]
    pub dual_io: bool,
    #[serde(default)]
    pub quad_io: bool,
    #[serde(default)]
    pub four_byte_addr: bool,
    #[serde(default)]
    pub four_byte_enter: bool,
    #[serde(default)]
    pub four_byte_native: bool,
    #[serde(default)]
    pub ext_addr_reg: bool,
    #[serde(default)]
    pub otp: bool,
    #[serde(default)]
    pub qpi: bool,
    #[serde(default)]
    pub security_reg: bool,
    #[serde(default)]
    pub sfdp: bool,
    #[serde(default)]
    pub write_byte: bool,
    #[serde(default)]
    pub aai_word: bool,
    #[serde(default)]
    pub status_reg_2: bool,
    #[serde(default)]
    pub status_reg_3: bool,
    #[serde(default)]
    pub qe_sr2: bool,
    #[serde(default)]
    pub deep_power_down: bool,
    #[serde(default)]
    pub wp_tb: bool,
    #[serde(default)]
    pub wp_sec: bool,
    #[serde(default)]
    pub wp_cmp: bool,
    #[serde(default)]
    pub wp_srl: bool,
    #[serde(default)]
    pub wp_volatile: bool,
    #[serde(default)]
    pub wp_bp3: bool,
    #[serde(default)]
    pub wp_wps: bool,
}

#[derive(Debug, Deserialize)]
pub struct VoltageDef {
    #[serde(default = "default_voltage_min")]
    pub min: u16,
    #[serde(default = "default_voltage_max")]
    pub max: u16,
}

fn default_voltage_min() -> u16 {
    2700
}
fn default_voltage_max() -> u16 {
    3600
}

impl Default for VoltageDef {
    fn default() -> Self {
        Self {
            min: 2700,
            max: 3600,
        }
    }
}

#[derive(Debug, Default, Deserialize)]
pub enum WriteGranularityDef {
    Bit,
    Byte,
    #[default]
    Page,
}

#[derive(Debug, Deserialize)]
pub struct EraseBlockDef {
    pub opcode: u8,
    pub regions: Vec<RegionDef>,
}

#[derive(Debug, Deserialize)]
pub struct RegionDef {
    pub size: Size,
    pub count: u32,
}

#[derive(Debug, Default, Deserialize)]
pub enum TestStatusDef {
    #[default]
    Untested,
    Ok,
    Bad,
    Na,
}

#[derive(Debug, Default, Deserialize)]
pub struct TestStatusesDef {
    #[serde(default)]
    pub probe: TestStatusDef,
    #[serde(default)]
    pub read: TestStatusDef,
    #[serde(default)]
    pub erase: TestStatusDef,
    #[serde(default)]
    pub write: TestStatusDef,
    #[serde(default)]
    pub wp: TestStatusDef,
}

// ---------------------------------------------------------------------------
// Runtime chip representation (converted from RON types)
// ---------------------------------------------------------------------------

/// Erase operation: an SPI opcode and the block size it erases.
#[derive(Debug, Clone)]
pub struct EraseOp {
    pub opcode: u8,
    pub block_size: u32, // in bytes (from the first/only uniform region)
}

/// Resolved chip definition ready for use.
#[derive(Debug, Clone)]
pub struct FlashChip {
    pub vendor: String,
    pub name: String,
    pub jedec_manufacturer: u8,
    pub jedec_device: u16,
    pub total_size: u32,
    pub page_size: u16,
    pub supports_4byte: bool,
    pub supports_dual: bool,
    pub supports_quad: bool,
    pub supports_fast_read: bool,
    pub erase_ops: Vec<EraseOp>,
}

impl FlashChip {
    /// Full 3-byte JEDEC ID: [manufacturer, device_hi, device_lo]
    pub fn jedec_id_bytes(&self) -> [u8; 3] {
        [
            self.jedec_manufacturer,
            (self.jedec_device >> 8) as u8,
            (self.jedec_device & 0xFF) as u8,
        ]
    }

    /// Number of 8-byte SDRAM bursts needed to erase the whole chip, minus 1.
    pub fn chip_erase_bursts(&self) -> u32 {
        (self.total_size / 8).saturating_sub(1)
    }

    /// Erase operations sorted by block size (smallest first), excluding chip erase.
    pub fn sector_erase_ops(&self) -> Vec<&EraseOp> {
        let mut ops: Vec<&EraseOp> = self
            .erase_ops
            .iter()
            .filter(|op| op.block_size < self.total_size)
            .collect();
        ops.sort_by_key(|op| op.block_size);
        ops
    }
}

// ---------------------------------------------------------------------------
// Loading
// ---------------------------------------------------------------------------

/// Load all RON chip files from a directory, returning a flat list of chips.
pub fn load_chip_db(dir: &Path) -> Result<Vec<FlashChip>> {
    let mut chips = Vec::new();

    let entries: Vec<_> = std::fs::read_dir(dir)
        .with_context(|| format!("Failed to read chip database directory: {}", dir.display()))?
        .collect::<std::io::Result<Vec<_>>>()
        .with_context(|| format!("Failed to list {}", dir.display()))?;

    for entry in entries {
        let path = entry.path();
        if path.extension().and_then(|e| e.to_str()) != Some("ron") {
            continue;
        }

        let content = std::fs::read_to_string(&path)
            .with_context(|| format!("Failed to read {}", path.display()))?;

        let vendor_def: VendorDef = ron::from_str(&content)
            .with_context(|| format!("Failed to parse {}", path.display()))?;

        for chip_def in &vendor_def.chips {
            let mut erase_ops = Vec::new();
            for eb in &chip_def.erase_blocks {
                // Use the first region's size as the block size.
                // Non-uniform erase layouts (boot sector chips) are rare;
                // for emulation we only need the uniform block size.
                if let Some(region) = eb.regions.first() {
                    erase_ops.push(EraseOp {
                        opcode: eb.opcode,
                        block_size: region.size.to_bytes(),
                    });
                }
            }

            chips.push(FlashChip {
                vendor: vendor_def.vendor.clone(),
                name: chip_def.name.clone(),
                jedec_manufacturer: vendor_def.manufacturer_id,
                jedec_device: chip_def.device_id,
                total_size: chip_def.total_size.to_bytes(),
                page_size: chip_def.page_size,
                supports_4byte: chip_def.features.four_byte_addr
                    || chip_def.features.four_byte_enter,
                supports_dual: chip_def.features.dual_io,
                supports_quad: chip_def.features.quad_io,
                supports_fast_read: chip_def.features.fast_read,
                erase_ops,
            });
        }
    }

    Ok(chips)
}

/// Find a chip by name (case-insensitive substring match).
pub fn find_chip_by_name<'a>(chips: &'a [FlashChip], name: &str) -> Result<&'a FlashChip> {
    let name_lower = name.to_lowercase();

    // Try exact match first
    let exact: Vec<_> = chips
        .iter()
        .filter(|c| c.name.to_lowercase() == name_lower)
        .collect();
    if exact.len() == 1 {
        return Ok(exact[0]);
    }

    // Try substring match
    let matches: Vec<_> = chips
        .iter()
        .filter(|c| c.name.to_lowercase().contains(&name_lower))
        .collect();

    match matches.len() {
        0 => bail!("No chip found matching '{}'", name),
        1 => Ok(matches[0]),
        n => {
            let names: Vec<_> = matches
                .iter()
                .map(|c| format!("  {} ({})", c.name, c.vendor))
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
