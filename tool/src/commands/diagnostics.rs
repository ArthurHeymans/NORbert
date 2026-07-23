use super::*;

// ---------------------------------------------------------------------------
// Probe command -- D2XX backend
// ---------------------------------------------------------------------------

#[cfg(feature = "d2xx")]
pub(super) fn cmd_probe(cli: &Cli) -> Result<()> {
    use libftd2xx::FtdiCommon;

    // We need raw access to the FT2232H, not the Transport abstraction,
    // because we want short timeouts per-probe and queue_status checks.
    eprintln!("Opening FT2232H for probe (D2XX)...");
    let serial = cli.ft_serial.as_deref();
    let mut ft = if let Some(sn) = serial {
        libftd2xx::Ft2232h::with_serial_number(sn)
            .with_context(|| format!("No FT2232H with serial '{}'", sn))?
    } else {
        libftd2xx::Ft2232h::with_description("NORbert FT245 A")
            .context("No FT2232H found (looking for 'NORbert FT245 A')")?
    };

    ft.reset().context("FT2232H reset failed")?;
    ft.purge_all().context("FT2232H purge failed")?;
    ft.set_usb_parameters(65536)?;
    ft.set_latency_timer(Duration::from_millis(2))?;
    ft.set_timeouts(Duration::from_millis(500), Duration::from_secs(5))?;

    // Drain stale data
    let mut trash = [0u8; 4096];
    loop {
        let n = ft.queue_status()?;
        if n == 0 {
            break;
        }
        let to_read = std::cmp::min(n, trash.len());
        let _ = ft.read(&mut trash[..to_read]);
    }

    // Test bytes: mix of valid commands, diagnostics, and non-commands
    let probes: Vec<(u8, &str, Option<u8>)> = vec![
        (0x30, "CMD_VERSION", Some(PROTOCOL_VERSION)),
        (0x00, "NOP / zero", None),
        (0x55, "non-command 0x55", None),
        (0xAA, "non-command 0xAA", None),
        (0xFF, "non-command 0xFF", None),
    ];

    println!("FT245 probe (D2XX): send 1 byte, wait 200ms, read back\n");
    println!("{:<30} {:>4}   {:>8}  Response", "Test", "Sent", "Expected");
    println!("{}", "-".repeat(70));

    for (byte, label, expected) in &probes {
        // Purge before each probe
        ft.purge_all()?;
        thread::sleep(Duration::from_millis(50));

        // Drain anything residual
        loop {
            let n = ft.queue_status()?;
            if n == 0 {
                break;
            }
            let to_read = std::cmp::min(n, trash.len());
            let _ = ft.read(&mut trash[..to_read]);
        }

        // Send the byte
        ft.write_all(&[*byte])?;

        // Wait for response
        thread::sleep(Duration::from_millis(200));

        // Check what came back
        let n = ft.queue_status()?;
        let exp_str = match expected {
            Some(v) => format!("0x{:02X}", v),
            None => "none".to_string(),
        };
        if n == 0 {
            let status = if expected.is_none() {
                "OK"
            } else {
                "FAIL(none)"
            };
            println!(
                "{:<30} 0x{:02X}   {:>8}  (no response) {}",
                label, byte, exp_str, status
            );
        } else {
            let to_read = std::cmp::min(n, 32);
            let mut buf = vec![0u8; to_read];
            let got = ft.read(&mut buf)?;
            buf.truncate(got);
            let hex_str: Vec<String> = buf.iter().map(|b| format!("0x{:02X}", b)).collect();
            let status = match expected {
                Some(v) if got == 1 && buf[0] == *v => "OK",
                Some(_) if got == 1 && buf[0] == *byte => "FAIL(echo)",
                _ => "FAIL",
            };
            println!(
                "{:<30} 0x{:02X}   {:>8}  {} bytes: [{}] {}",
                label,
                byte,
                exp_str,
                got,
                hex_str.join(", "),
                status,
            );
        }
    }

    // Bonus: send multiple bytes at once and see what comes back
    println!("\n--- Multi-byte test ---");
    ft.purge_all()?;
    thread::sleep(Duration::from_millis(50));
    loop {
        let n = ft.queue_status()?;
        if n == 0 {
            break;
        }
        let to_read = std::cmp::min(n, trash.len());
        let _ = ft.read(&mut trash[..to_read]);
    }

    let multi = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];
    ft.write_all(&multi)?;
    thread::sleep(Duration::from_millis(200));
    let n = ft.queue_status()?;
    if n == 0 {
        println!("Sent {:?}: (no response)", multi);
    } else {
        let to_read = std::cmp::min(n, 64);
        let mut buf = vec![0u8; to_read];
        let got = ft.read(&mut buf)?;
        buf.truncate(got);
        let hex_str: Vec<String> = buf.iter().map(|b| format!("0x{:02X}", b)).collect();
        let is_echo = buf == multi[..got];
        println!(
            "Sent {} bytes: {:02X?}\nGot  {} bytes: [{}]{}",
            multi.len(),
            multi,
            got,
            hex_str.join(", "),
            if is_echo { " << EXACT ECHO" } else { "" },
        );
    }

    Ok(())
}

// ---------------------------------------------------------------------------
// Probe command -- rs-ftdi backend
// ---------------------------------------------------------------------------

#[cfg(all(feature = "ftdi", not(feature = "d2xx")))]
pub(super) fn cmd_probe(cli: &Cli) -> Result<()> {
    // Open FT2232H via rs-ftdi
    eprintln!("Opening FT2232H for probe (rs-ftdi)...");
    let serial = cli.ft_serial.as_deref();
    let mut dev = if let Some(sn) = serial {
        let filter = ftdi::DeviceFilter::new(FTDI_VID, FT2232H_PID).serial(sn);
        ftdi::FtdiDevice::open_with_filter(&filter, ftdi::Interface::A)
            .with_context(|| format!("No FT2232H with serial '{}'", sn))?
    } else {
        let filter = ftdi::DeviceFilter::new(FTDI_VID, FT2232H_PID).description("NORbert FT245");
        ftdi::FtdiDevice::open_with_filter(&filter, ftdi::Interface::A)
            .or_else(|_| {
                ftdi::FtdiDevice::open_with_interface(FTDI_VID, FT2232H_PID, ftdi::Interface::A)
            })
            .context("No FT2232H found")?
    };

    dev.usb_reset().context("FT2232H reset failed")?;
    dev.flush_all().context("FT2232H flush failed")?;
    dev.set_latency_timer(2)?;
    dev.set_read_timeout(Duration::from_millis(500));
    dev.set_write_timeout(Duration::from_secs(5));

    // Drain stale data
    let mut trash = [0u8; 4096];
    loop {
        match dev.read_data(&mut trash) {
            Ok(0) => break,
            Ok(_) => continue,
            Err(_) => break,
        }
    }

    // Test bytes
    let probes: Vec<(u8, &str, Option<u8>)> = vec![
        (0x30, "CMD_VERSION", Some(PROTOCOL_VERSION)),
        (0x00, "NOP / zero", None),
        (0x55, "non-command 0x55", None),
        (0xAA, "non-command 0xAA", None),
        (0xFF, "non-command 0xFF", None),
    ];

    println!("FT245 probe (rs-ftdi): send 1 byte, wait 200ms, read back\n");
    println!("{:<30} {:>4}   {:>8}  Response", "Test", "Sent", "Expected");
    println!("{}", "-".repeat(70));

    for (byte, label, expected) in &probes {
        // Flush before each probe
        dev.flush_all().ok();
        thread::sleep(Duration::from_millis(50));

        // Drain
        loop {
            match dev.read_data(&mut trash) {
                Ok(0) => break,
                Ok(_) => continue,
                Err(_) => break,
            }
        }

        // Send the byte
        dev.write_all(&[*byte]).map_err(|e| anyhow!(e))?;

        // Wait for response
        thread::sleep(Duration::from_millis(200));

        // Check what came back
        let mut buf = [0u8; 32];
        let got = dev.read_data(&mut buf).unwrap_or(0);
        let exp_str = match expected {
            Some(v) => format!("0x{:02X}", v),
            None => "none".to_string(),
        };
        if got == 0 {
            let status = if expected.is_none() {
                "OK"
            } else {
                "FAIL(none)"
            };
            println!(
                "{:<30} 0x{:02X}   {:>8}  (no response) {}",
                label, byte, exp_str, status
            );
        } else {
            let hex_str: Vec<String> = buf[..got].iter().map(|b| format!("0x{:02X}", b)).collect();
            let status = match expected {
                Some(v) if got == 1 && buf[0] == *v => "OK",
                Some(_) if got == 1 && buf[0] == *byte => "FAIL(echo)",
                _ => "FAIL",
            };
            println!(
                "{:<30} 0x{:02X}   {:>8}  {} bytes: [{}] {}",
                label,
                byte,
                exp_str,
                got,
                hex_str.join(", "),
                status,
            );
        }
    }

    // Multi-byte test
    println!("\n--- Multi-byte test ---");
    dev.flush_all().ok();
    thread::sleep(Duration::from_millis(50));
    loop {
        match dev.read_data(&mut trash) {
            Ok(0) => break,
            Ok(_) => continue,
            Err(_) => break,
        }
    }

    let multi = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];
    dev.write_all(&multi).map_err(|e| anyhow!(e))?;
    thread::sleep(Duration::from_millis(200));
    let mut buf = [0u8; 64];
    let got = dev.read_data(&mut buf).unwrap_or(0);
    if got == 0 {
        println!("Sent {:?}: (no response)", multi);
    } else {
        let hex_str: Vec<String> = buf[..got].iter().map(|b| format!("0x{:02X}", b)).collect();
        let is_echo = buf[..got] == multi[..got];
        println!(
            "Sent {} bytes: {:02X?}\nGot  {} bytes: [{}]{}",
            multi.len(),
            multi,
            got,
            hex_str.join(", "),
            if is_echo { " << EXACT ECHO" } else { "" },
        );
    }

    Ok(())
}

// ---------------------------------------------------------------------------
// Probe command -- no FT245 backend
// ---------------------------------------------------------------------------

#[cfg(not(any(feature = "d2xx", feature = "ftdi")))]
pub(super) fn cmd_probe(_cli: &Cli) -> Result<()> {
    bail!("Probe requires the 'd2xx' or 'ftdi' feature");
}

// ---------------------------------------------------------------------------
// FT-list command
// ---------------------------------------------------------------------------

#[cfg(feature = "d2xx")]
pub(super) fn cmd_ft_list() -> Result<()> {
    let devices = libftd2xx::list_devices().context("Failed to enumerate FTDI devices")?;

    if devices.is_empty() {
        println!("No FTDI devices found");
        println!(
            "Hint: FT2232H has VID:PID {:04x}:{:04x}",
            FTDI_VID, FT2232H_PID
        );
        return Ok(());
    }

    println!("FTDI devices ({} found):", devices.len());
    for (i, dev) in devices.iter().enumerate() {
        println!("  [{}] {:?}", i, dev);
    }
    println!();
    println!("Use --ft-serial <SERIAL> to select a specific device.");
    println!("Channel A is used by default for async FIFO.");
    Ok(())
}

#[cfg(all(feature = "ftdi", not(feature = "d2xx")))]
pub(super) fn cmd_ft_list() -> Result<()> {
    let devices = ftdi::find_devices(FTDI_VID, FT2232H_PID)
        .map_err(|e| anyhow!(e))
        .context("Failed to enumerate FTDI devices")?;

    if devices.is_empty() {
        println!("No FT2232H devices found");
        println!(
            "Hint: FT2232H has VID:PID {:04x}:{:04x}",
            FTDI_VID, FT2232H_PID
        );
        return Ok(());
    }

    println!("FT2232H devices ({} found):", devices.len());
    for (i, dev) in devices.iter().enumerate() {
        println!(
            "  [{}] bus={} addr={} vid={:04x} pid={:04x}",
            i,
            dev.busnum(),
            dev.device_address(),
            dev.vendor_id(),
            dev.product_id(),
        );
    }
    println!();
    println!("Use --ft-serial <SERIAL> to select a specific device.");
    println!("Channel A is used by default for async FIFO.");
    Ok(())
}

#[cfg(not(any(feature = "d2xx", feature = "ftdi")))]
pub(super) fn cmd_ft_list() -> Result<()> {
    bail!("ft-list requires the 'd2xx' or 'ftdi' feature");
}
