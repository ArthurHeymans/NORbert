//! End-to-end PR73 operations against the Verilated NORbert SPI/host engines.

use rflasher_core::programmer::{SpiFeatures, SpiMaster, default_execute};
use rflasher_core::spi::SpiCommand;
use rflasher_core::{Error, Result};
use std::io::{BufRead, BufReader, Write};
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};

pub struct RtlMaster {
    child: Child,
    input: ChildStdin,
    output: BufReader<ChildStdout>,
    pub opcodes: Vec<u8>,
}

pub fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

impl RtlMaster {
    pub fn new() -> Self {
        let executable = std::env::var("NORBERT_BRIDGE").expect("run make test-rflasher");
        let mut child = Command::new(executable)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .expect("start Verilator bridge");
        let input = child.stdin.take().unwrap();
        let output = BufReader::new(child.stdout.take().unwrap());
        Self {
            child,
            input,
            output,
            opcodes: Vec::new(),
        }
    }

    pub fn request(&mut self, line: &str) -> Vec<u8> {
        writeln!(self.input, "{line}").expect("write bridge request");
        self.input.flush().unwrap();
        let mut response = String::new();
        assert_ne!(
            self.output
                .read_line(&mut response)
                .expect("read bridge response"),
            0,
            "bridge exited handling {line}"
        );
        let text = response.trim();
        assert_eq!(text.len() % 2, 0, "malformed bridge response: {text}");
        (0..text.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&text[i..i + 2], 16).expect("bridge response hex"))
            .collect()
    }

    pub fn host(&mut self, bytes: &[u8]) -> Vec<u8> {
        self.request(&format!("H {}", hex(bytes)))
    }
    pub fn raw(&mut self, bytes: &[u8], read: usize) -> Vec<u8> {
        self.opcodes.push(bytes[0]);
        let mut tx = bytes.to_vec();
        tx.resize(tx.len() + read, 0);
        let rx = self.request(&format!("S {}", hex(&tx)));
        assert_eq!(rx.len(), tx.len());
        rx[bytes.len()..].to_vec()
    }
    pub fn seed(&mut self, address: u32, bytes: &[u8]) {
        assert!(
            self.request(&format!("M {address} {}", hex(bytes)))
                .is_empty()
        );
    }
    pub fn memory(&mut self, address: u32, length: usize) -> Vec<u8> {
        self.request(&format!("R {address} {length}"))
    }
}
impl Default for RtlMaster {
    fn default() -> Self {
        Self::new()
    }
}
impl Drop for RtlMaster {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}
impl SpiMaster for RtlMaster {
    fn features(&self) -> SpiFeatures {
        SpiFeatures::FOUR_BYTE_ADDR
    }
    fn max_read_len(&self) -> usize {
        512
    }
    fn max_write_len(&self) -> usize {
        256
    }
    async fn execute(&mut self, command: &mut SpiCommand<'_>) -> Result<()> {
        if command.read_buf.len() > self.max_read_len()
            || command.write_data.len() > self.max_write_len()
        {
            return Err(Error::ProgrammerError);
        }
        default_execute(command, self.features(), |write, read| {
            read.copy_from_slice(&self.raw(write, read.len()));
            Ok(())
        })
    }
    async fn delay_us(&mut self, us: u32) {
        assert!(self.request(&format!("D {us}")).is_empty());
    }
}
