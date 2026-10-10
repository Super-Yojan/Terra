//! Single-phone prototype router: localhost, Tailscale, or an explicit all-interface bind.
use std::{net::Ipv4Addr, time::Duration};
use zenoh::Wait;
fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let mut endpoints = vec!["tcp/127.0.0.1:7448".to_string()];
    if let Some(address) = std::env::args().nth(1) {
        let ip: Ipv4Addr = address.parse()?;
        let [a, b, _, _] = ip.octets();
        if ip.is_unspecified() {
            endpoints = vec!["tcp/0.0.0.0:7448".to_string()];
        } else if a != 100 || !(64..=127).contains(&b) {
            return Err("Expected 0.0.0.0 or a Tailscale IPv4 address in 100.64.0.0/10".into());
        }
        if !ip.is_unspecified() { endpoints.push(format!("tcp/{ip}:7448")); }
    }
    let mut config = zenoh::Config::default();
    for (key, value) in [
        ("mode", "\"router\"".to_string()),
        ("listen/endpoints", serde_json::to_string(&endpoints)?),
        ("scouting/multicast/enabled", "false".to_string()),
        ("scouting/gossip/enabled", "false".to_string()),
    ] {
        config.insert_json5(key, &value)?;
    }
    let session = zenoh::open(config).wait()?;
    println!(
        "ARGOS prototype router listening on {}",
        endpoints.join(", ")
    );
    println!("One phone per prefix. Keep TerraPhone in the foreground. Ctrl-C to stop.");
    while !session.is_closed() {
        std::thread::sleep(Duration::from_secs(1));
    }
    Ok(())
}
