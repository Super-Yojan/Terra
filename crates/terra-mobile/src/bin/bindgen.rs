//! Command-line entry point for generating mobile-language UniFFI bindings.
/// Run the UniFFI binding-generator CLI using its own argument and error handling.
fn main() {
    uniffi::uniffi_bindgen_main();
}
