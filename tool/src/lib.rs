#[cfg(all(feature = "cli", feature = "wasm"))]
compile_error!(
    "the 'cli' and 'wasm' features are mutually exclusive; build WASM with --no-default-features --features wasm"
);

#[cfg(all(feature = "wasm", target_arch = "wasm32"))]
#[allow(dead_code)]
mod device;

#[cfg(all(feature = "wasm", target_arch = "wasm32"))]
#[allow(dead_code)]
mod protocol;

#[cfg(all(feature = "wasm", target_arch = "wasm32"))]
mod web;

#[cfg(all(feature = "wasm", target_arch = "wasm32"))]
pub use web::WebFlashDevice;
