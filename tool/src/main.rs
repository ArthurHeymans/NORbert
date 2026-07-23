mod chip;
mod cli;
mod commands;
mod device;
mod protocol;
mod sfdp;
mod transport;

use anyhow::Result;
use clap::Parser;
use cli::Cli;

fn main() -> Result<()> {
    let cli = Cli::parse();
    commands::run(&cli)
}
