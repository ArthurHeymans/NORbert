use super::*;
use crate::spi_log::{ActivityLog, HEADER, opcode_name};

// ---------------------------------------------------------------------------
// Monitor command -- real-time SPI bus logging with TOCTOU detection
// ---------------------------------------------------------------------------

pub(super) fn cmd_monitor(cli: &Cli) -> Result<()> {
    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, Ordering};

    let mut device = open_device(cli)?;

    // Set up Ctrl+C handler
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    })
    .context("Failed to set Ctrl+C handler")?;

    // Enable logging
    device.log_start()?;
    eprintln!("Logging started. Press Ctrl+C to stop.\n");
    eprintln!("{HEADER}");
    eprintln!("{}", "-".repeat(60));

    let mut log = ActivityLog::default();

    // Poll loop: ask the FPGA for log data every ~5ms.  Every poll
    // response is self-delimited (ends with LOG_POLL_TERMINATOR) so
    // there is no ambiguity between log bytes and protocol bytes.
    while running.load(Ordering::SeqCst) {
        let data = device.log_poll()?;
        if data.is_empty() {
            thread::sleep(Duration::from_millis(5));
            continue;
        }
        print!("{}", log.feed(&data));
        std::io::stdout().flush()?;
    }

    // Stop logging and drain any residual log bytes.  Since logging
    // and protocol responses travel separately now (no mux), the stop
    // ACK is guaranteed to be the *only* reply to CMD_LOGCTL.
    eprintln!("\nStopping...");
    device.log_stop()?;
    let remaining = device.log_poll()?;
    if !remaining.is_empty() {
        eprintln!("(drained {} residual log bytes)", remaining.len());
    }

    // Print TOCTOU summary
    let double_reads: Vec<_> = log.double_reads().collect();
    if !double_reads.is_empty() {
        eprintln!("\n--- TOCTOU Detection Summary ---");
        eprintln!("{} address(es) read more than once:\n", double_reads.len());
        for (addr, opcode, count) in double_reads {
            eprintln!(
                "  0x{:06X}  ({})  read {} times",
                addr,
                opcode_name(opcode),
                count
            );
        }
        eprintln!("\nThese are potential TOCTOU attack targets.");
        eprintln!("Use 'toctou set' to configure traps for these addresses.");
    } else {
        eprintln!("\nNo double-reads detected.");
    }

    Ok(())
}
