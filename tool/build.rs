use std::env;
use std::fs;
use std::io::Write;
use std::path::Path;

fn main() {
    let manifest_dir = env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR");
    let chips_dir = Path::new(&manifest_dir).join("chips/vendors");
    let out_dir = env::var("OUT_DIR").expect("OUT_DIR");
    let out_path = Path::new(&out_dir).join("embedded_chip_db.rs");

    println!("cargo:rerun-if-changed={}", chips_dir.display());

    let mut entries = Vec::new();
    if chips_dir.is_dir() {
        for entry in fs::read_dir(&chips_dir).expect("read chips/vendors") {
            let entry = entry.expect("chip db entry");
            let path = entry.path();
            if path.extension().and_then(|ext| ext.to_str()) == Some("ron") {
                println!("cargo:rerun-if-changed={}", path.display());
                let name = path
                    .file_name()
                    .and_then(|name| name.to_str())
                    .expect("utf-8 chip db filename")
                    .to_owned();
                entries.push(name);
            }
        }
    }
    entries.sort();

    let mut file = fs::File::create(out_path).expect("create embedded_chip_db.rs");
    writeln!(file, "const EMBEDDED_CHIP_RON: &[(&str, &str)] = &[").unwrap();
    for entry in entries {
        writeln!(
            file,
            "    ({entry:?}, include_str!(concat!(env!(\"CARGO_MANIFEST_DIR\"), \"/chips/vendors/{entry}\"))),"
        )
        .unwrap();
    }
    writeln!(file, "];").unwrap();
}
