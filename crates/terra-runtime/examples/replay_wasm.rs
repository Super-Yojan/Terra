#[path = "support/replay.rs"]
mod replay;
#[unsafe(no_mangle)]
pub extern "C" fn parity_len() -> u32 {
    replay::trace().len() as u32
}
#[unsafe(no_mangle)]
pub extern "C" fn parity_value(index: u32) -> f64 {
    replay::trace()[index as usize]
}
