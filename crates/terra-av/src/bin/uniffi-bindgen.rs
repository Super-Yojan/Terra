//! Generates the Swift bindings: `cargo run -p terra-av --features bindgen --bin uniffi-bindgen -- generate ...`
fn main() {
    uniffi::uniffi_bindgen_main();
}
