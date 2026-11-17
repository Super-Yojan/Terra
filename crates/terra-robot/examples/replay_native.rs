#[path = "support/replay.rs"]
mod replay;
fn main() {
    println!("{}", serde_json::to_string(&replay::trace()).unwrap());
}
