use super::*;

// ---------------------------------------------------------------------------
// SPI opcode names for log display
// ---------------------------------------------------------------------------

fn spi_opcode_name(opcode: u8) -> &'static str {
    match opcode {
        0x01 => "WRITE_STATUS",
        0x02 => "PAGE_PROGRAM",
        0x03 => "READ",
        0x04 => "WRITE_DISABLE",
        0x05 => "READ_STATUS",
        0x06 => "WRITE_ENABLE",
        0x0B => "FAST_READ",
        0x0C => "FAST_READ_4B",
        0x12 => "PAGE_PROGRAM_4B",
        0x13 => "READ_4B",
        0x20 => "SECTOR_ERASE_4K",
        0x21 => "SECTOR_ERASE_4K_4B",
        0x35 => "READ_STATUS2",
        0x3B => "DUAL_READ",
        0x3C => "DUAL_READ_4B",
        0x52 => "BLOCK_ERASE_32K",
        0x5C => "BLOCK_ERASE_32K_4B",
        0x5A => "READ_SFDP",
        0x60 => "CHIP_ERASE",
        0x6B => "QUAD_READ",
        0x6C => "QUAD_READ_4B",
        0x9E | 0x9F => "READ_JEDEC_ID",
        0xB7 => "4BYTE_ENABLE",
        0xBB => "DUAL_IO_READ",
        0xBC => "DUAL_IO_READ_4B",
        0xC7 => "CHIP_ERASE",
        0xD8 => "BLOCK_ERASE_64K",
        0xDC => "BLOCK_ERASE_64K_4B",
        0xE9 => "4BYTE_DISABLE",
        0xEB => "QUAD_IO_READ",
        0xEC => "QUAD_IO_READ_4B",
        _ => "UNKNOWN",
    }
}

fn is_read_opcode(opcode: u8) -> bool {
    match opcode {
        0x03 | 0x0B | 0x0C | 0x13 | 0x3B | 0x3C | 0x5A | 0x6B | 0x6C | 0xBB | 0xBC | 0xEB
        | 0xEC => true,
        _ => false,
    }
}

// ---------------------------------------------------------------------------
// Monitor command -- real-time SPI bus logging with TOCTOU detection
// ---------------------------------------------------------------------------

pub(super) fn cmd_monitor(cli: &Cli) -> Result<()> {
    use std::collections::HashMap;
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
    eprintln!("{:<6} {:<18} {:<10} INFO", "TXN#", "COMMAND", "ADDRESS");
    eprintln!("{}", "-".repeat(60));

    // TOCTOU detection state: address range -> access count
    // Key: (start_addr, opcode), Value: count
    let mut access_map: HashMap<(u32, u8), u32> = HashMap::new();
    let mut double_reads: Vec<(u32, u8)> = Vec::new();

    // Log packet parser state
    let mut pending = Vec::new();
    let mut txn_count: u32 = 0;
    let mut current_opcode: u8 = 0;
    let mut current_addr: u32 = 0;
    let mut command_line_open = false;

    // Poll loop: ask the FPGA for log data every ~5ms.  Every poll
    // response is self-delimited (ends with LOG_POLL_TERMINATOR) so
    // there is no ambiguity between log bytes and protocol bytes.
    while running.load(Ordering::SeqCst) {
        let data = device.log_poll()?;
        if data.is_empty() {
            thread::sleep(Duration::from_millis(5));
            continue;
        }

        pending.extend_from_slice(&data);

        // Parse complete packets from pending buffer
        let mut pos = 0;
        while pos < pending.len() {
            let remaining = pending.len() - pos;
            match pending[pos] {
                LOG_CMD if remaining >= 2 => {
                    if command_line_open {
                        println!();
                    }

                    let opcode = pending[pos + 1];
                    current_opcode = opcode;
                    txn_count += 1;
                    print!(
                        "{:<6} 0x{:02X} {:<13}",
                        txn_count,
                        opcode,
                        spi_opcode_name(opcode)
                    );
                    std::io::stdout().flush()?;
                    command_line_open = true;
                    pos += 2;
                }
                LOG_ADDR if remaining >= 5 => {
                    let addr = ((pending[pos + 1] as u32) << 24)
                        | ((pending[pos + 2] as u32) << 16)
                        | ((pending[pos + 3] as u32) << 8)
                        | (pending[pos + 4] as u32);
                    current_addr = addr;
                    if addr > 0xFFFFFF {
                        print!(" 0x{:08X}", addr);
                    } else {
                        print!(" 0x{:06X}", addr);
                    }

                    // TOCTOU detection: track read accesses
                    if is_read_opcode(current_opcode) {
                        let key = (addr, current_opcode);
                        let count = access_map.entry(key).or_insert(0);
                        *count += 1;
                        if *count == 2 {
                            double_reads.push(key);
                            print!("  ** DOUBLE READ (TOCTOU candidate)");
                        } else if *count > 2 {
                            print!("  ** READ #{}", count);
                        }
                    }
                    println!();
                    command_line_open = false;
                    pos += 5;
                }
                LOG_END if remaining >= 4 => {
                    let byte_count = ((pending[pos + 1] as u32) << 16)
                        | ((pending[pos + 2] as u32) << 8)
                        | (pending[pos + 3] as u32);

                    // Addressless commands do not receive a LOG_ADDR packet,
                    // so terminate their command row when the transaction ends.
                    if command_line_open {
                        println!();
                        command_line_open = false;
                    }

                    if byte_count > 1 {
                        println!(
                            "       end: {} bytes from 0x{:06X}",
                            byte_count, current_addr
                        );
                    }
                    pos += 4;
                }
                LOG_TRAP if remaining >= 6 => {
                    let index = pending[pos + 1];
                    let addr = ((pending[pos + 2] as u32) << 16)
                        | ((pending[pos + 3] as u32) << 8)
                        | (pending[pos + 4] as u32);
                    eprintln!(
                        "  !! TOCTOU TRAP #{} FIRED at 0x{:06X} -- serving replacement data",
                        index, addr
                    );
                    pos += 6; // type(1) + index(1) + addr(3) + pad(1)
                }
                0xA1..=0xAF => {
                    // Known packet type but incomplete -- wait for more data
                    break;
                }
                _ => {
                    // Unknown byte, skip it
                    pos += 1;
                }
            }
        }
        // Remove consumed bytes
        pending.drain(..pos);
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
    if !double_reads.is_empty() {
        eprintln!("\n--- TOCTOU Detection Summary ---");
        eprintln!("{} address(es) read more than once:\n", double_reads.len());
        for (addr, opcode) in &double_reads {
            let count = access_map[&(*addr, *opcode)];
            eprintln!(
                "  0x{:06X}  ({})  read {} times",
                addr,
                spi_opcode_name(*opcode),
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
