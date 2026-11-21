//! A local Zenoh router for testing without the cloud one. Same Zenoh
//! version as terra-comms, no login, no TLS.
//!
//!     cargo run -p terra-comms --example router [-- tcp/0.0.0.0:7447]
//!     just router            # same, and prints the endpoint for the phone
//!
//! Rover (phone) `[comms]`:  endpoint = "tcp/<this Mac's LAN IP>:7447"
//! Base station `[comms]`:   endpoint = "tcp/127.0.0.1:7447"
//! Leave out `user` on both: this router has no login.
use std::collections::HashSet;
use std::time::Duration;
use zenoh::Wait;

fn main() {
    // Enable Zenoh transport/TCP logs (override with RUST_LOG, e.g. RUST_LOG=debug).
    zenoh::init_log_from_env_or("zenoh_link_tcp=debug,zenoh_transport=debug,zenoh::net=info");

    let listen = std::env::args().nth(1).unwrap_or_else(|| "tcp/0.0.0.0:7447".into());
    let mut config = zenoh::Config::default();
    config.insert_json5("mode", r#""router""#).unwrap();
    config.insert_json5("listen/endpoints", &serde_json::to_string(&[&listen]).unwrap()).unwrap();
    config.insert_json5("scouting/multicast/enabled", "false").unwrap();
    let router = match zenoh::open(config).wait() {
        Ok(session) => session,
        Err(e) => {
            eprintln!("router: cannot listen on {listen}: {e}");
            std::process::exit(1);
        }
    };
    println!("router: zid={} listening on {listen} (Ctrl-C to stop)", router.zid());

    // Also subscribe to Zenoh adminspace transport events for clean connect/disconnect lines.
    let admin_key = format!("@/{}/session/transport/unicast/**", router.zid());
    let _admin_sub = router
        .declare_subscriber(&admin_key)
        .callback(|sample| {
            let payload = sample
                .payload()
                .try_to_string()
                .map(|s| s.into_owned())
                .unwrap_or_else(|_| format!("{} bytes", sample.payload().len()));
            println!("router: [{}] {} -> {}", sample.kind(), sample.key_expr(), payload);
        })
        .wait()
        .ok();

    let mut known_peers: HashSet<String> = HashSet::new();
    loop {
        let current: HashSet<String> = router
            .info()
            .peers_zid()
            .wait()
            .map(|z| z.to_string())
            .collect();
        for added in current.difference(&known_peers) {
            println!("router: peer connected    {added}");
        }
        for removed in known_peers.difference(&current) {
            println!("router: peer disconnected {removed}");
        }
        known_peers = current;
        std::thread::sleep(Duration::from_millis(500));
    }
}
