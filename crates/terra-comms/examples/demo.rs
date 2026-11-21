//! A rover and a base station talking through terra-comms, in one process.
//!
//!     cargo run -p terra-comms --example demo
//!
//! It starts a local Zenoh router, then:
//! - `run_rover`: a pretend rover (a 20 Hz loop in a thread). It advertises
//!   one controller, checks incoming tasks, drives toward the goal, and holds
//!   still on e-stop.
//! - `run_base`: a base station. It reads the rover's capabilities, validates
//!   a parameter form, sends a task, sees a bad task rejected, e-stops and
//!   resumes, then waits until the goal is reached.
//!
//! For a real router, drop `local_router` and put its endpoint in `[comms]`.
use std::thread;
use std::time::{Duration, Instant};

use terra_comms::*;

const ROVER_ID: u64 = 0;

fn main() {
    let port = 7447;

    run_base(port);
    // stop.store(true, Ordering::Relaxed);
    // rover.join().unwrap();
}

/// The `[comms]` table. In terra-av it is part of `terra-av.toml`; the
/// password, if any, comes from the Keychain via `with_password`.
fn config(port: u16, rover_id: Option<u64>) -> CommsConfig {
    let id = rover_id
        .map(|id| format!("rover_id = {id}\n"))
        .unwrap_or_default();
    let toml = format!("[comms]\nendpoint = \"tcp/127.0.0.1:{port}\"\nprefix = \"terra\"\n{id}");
    CommsConfig::from_toml(&toml).expect("valid [comms]")
}

fn run_base(port: u16) {
    let base = connect(&config(port, None)).expect("valid config").base();

    let status = base.latest::<TaskStatus>(ROVER_ID).map(|(s, _)| s);

    // 1. Find the rover and read what it can run (drives the UI dropdown).
    let (caps, _) = wait("capabilities", || base.latest::<Capabilities>(ROVER_ID));
    let ids: Vec<_> = caps.controllers.iter().map(|c| c.id.as_str()).collect();
    println!(
        "base:  rovers online {:?}, rover {ROVER_ID} offers {ids:?}",
        base.rovers()
    );
    // let controller = caps
    // .controller(&caps.default_controller)
    // .expect("default is listed");
    println!("Status: {:?}", status);

    read_depth_status(&base, ROVER_ID, Duration::from_secs(5));
    stream_pose(&base, ROVER_ID);
}

/// Continuously print `Pose` messages and save any incoming `WorldMap` from `rover_id` until Ctrl-C.
fn stream_pose(base: &BaseLink, rover_id: u64) {
    println!("base:  streaming pose & world_map for rover {rover_id} (Ctrl-C to stop)");
    let _map_sub = base.subscribe::<WorldMap>(move |id, map| {
        if id == rover_id {
            let path = format!("/tmp/rover-{id}-worldmap-{}.arworldmap", map.sequence);
            match std::fs::write(&path, &map.data) {
                Ok(()) => println!(
                    "map:   rover {id} seq={} t={:.3}s points={} size={:.1} KB -> saved {path}",
                    map.sequence,
                    map.t,
                    map.points.len(),
                    map.data.len() as f64 / 1024.0
                ),
                Err(e) => eprintln!("map:   failed to save {path}: {e}"),
            }
        }
    });
    let _pose_sub = base.subscribe::<Pose>(move |id, pose| {
        if id == rover_id {
            println!(
                "pose:  rover {id} x={:.2} y={:.2} yaw={:.2} tracked={} t={:.3}",
                pose.x, pose.y, pose.yaw, pose.tracked, pose.t
            );
        }
    });
    loop {
        thread::park();
    }
}

/// Subscribe to `Depth` (required because `BaseLink` only pulls depth while a
/// subscription is alive), wait for a frame from `rover_id`, and print its
/// status summary.
fn read_depth_status(base: &BaseLink, rover_id: u64, timeout: Duration) -> Option<(Depth, f64)> {
    let _sub = base.subscribe::<Depth>(|_, _| {});
    let result = wait_timeout(timeout, || base.latest::<Depth>(rover_id));
    let stats = base.stats()[&Topic::Depth];

    match &result {
        Some((depth, age)) => {
            let range = depth
                .points
                .iter()
                .map(|[x, y, z]| (x * x + y * y + z * z).sqrt())
                .fold((f32::INFINITY, 0.0_f32), |(min, max), d| {
                    (min.min(d), max.max(d))
                });
            if depth.points.is_empty() {
                println!(
                    "base:  depth seq={} t={:.3}s age={:.0}ms points=0 (received={}, bad={})",
                    depth.sequence,
                    depth.t,
                    age * 1000.0,
                    stats.received,
                    stats.bad_payload
                );
            } else {
                let first = depth.points[0];
                println!(
                    "base:  depth seq={} t={:.3}s age={:.0}ms points={} range=[{:.2}m, {:.2}m] first=[{:.2}, {:.2}, {:.2}] (received={}, bad={})",
                    depth.sequence,
                    depth.t,
                    age * 1000.0,
                    depth.points.len(),
                    range.0,
                    range.1,
                    first[0],
                    first[1],
                    first[2],
                    stats.received,
                    stats.bad_payload
                );
            }
        }
        None => {
            println!(
                "base:  depth: no frame from rover {rover_id} within {:.1}s (received={}, bad={})",
                timeout.as_secs_f64(),
                stats.received,
                stats.bad_payload
            );
        }
    }

    result
}

// ------------------------------------------------------------------ helpers

fn wait<T>(what: &str, mut f: impl FnMut() -> Option<T>) -> T {
    wait_timeout(Duration::from_secs(15), &mut f)
        .unwrap_or_else(|| panic!("timed out waiting for {what}"))
}

fn wait_timeout<T>(timeout: Duration, mut f: impl FnMut() -> Option<T>) -> Option<T> {
    let deadline = Instant::now() + timeout;
    loop {
        if let Some(v) = f() {
            return Some(v);
        }
        if Instant::now() >= deadline {
            return None;
        }
        thread::sleep(Duration::from_millis(20));
    }
}
