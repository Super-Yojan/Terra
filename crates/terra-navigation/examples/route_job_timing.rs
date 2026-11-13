//! RouteJob timing on the saved phone maps, from the repo root:
//!
//!   cargo run --release -p terra-navigation --example route_job_timing
use terra_mapping::MapSnapshot;
use terra_navigation::{HybridConfig, Pose, RouteJob};

/// L3 arrival radius minus a margin, as the arbiter passes it.
const SNAP: f64 = 0.65;
/// Expansions per control tick.
const BUDGET: u32 = 400;

fn main() {
    use std::time::Instant;
    for (path, start, goals) in [
        (
            "maps/run2.json",
            (-1.05, 1.65),
            vec![(2.35, 4.85), (-10.05, -1.15), (1.75, -0.95), (-1.25, 6.0)],
        ),
        (
            "maps/run1.json",
            (11.55, -3.75),
            vec![(13.75, -1.15), (9.15, -4.35), (6.75, 5.65)],
        ),
    ] {
        let Ok(text) = std::fs::read_to_string(path) else {
            println!("skipping {path}: not found");
            continue;
        };
        let v: serde_json::Value = serde_json::from_str(&text).unwrap();
        let map = MapSnapshot {
            width: v["width"].as_u64().unwrap() as u32,
            height: v["height"].as_u64().unwrap() as u32,
            resolution: v["resolution"].as_f64().unwrap(),
            origin_x: v["origin_x"].as_f64().unwrap(),
            origin_y: v["origin_y"].as_f64().unwrap(),
            occupancy: v["occupancy"]
                .as_array()
                .unwrap()
                .iter()
                .map(|c| c.as_i64().unwrap() as i8)
                .collect(),
        };
        let start = Pose {
            x: start.0,
            y: start.1,
            yaw: 0.,
        };
        for (gx, gy) in goals {
            let began = Instant::now();
            let job = RouteJob::new(&map, start, gx, gy, &HybridConfig::default(), SNAP);
            let setup = began.elapsed().as_secs_f64() * 1e3;
            let Ok(mut job) = job else {
                println!("{path} -> ({gx}, {gy}): {:?} in {setup:.1} ms", job.err());
                continue;
            };
            let (mut worst, mut ticks) = (0f64, 0);
            let result = loop {
                let t = Instant::now();
                let step = job.advance(BUDGET);
                worst = worst.max(t.elapsed().as_secs_f64() * 1e3);
                ticks += 1;
                if let Some(result) = step {
                    break result;
                }
            };
            println!(
                "{path} -> ({gx}, {gy}): {} setup {setup:.1} ms, {ticks} ticks, worst tick {worst:.2} ms",
                match &result {
                    Ok(r) => format!(
                        "ok ({} samples, reaches {})",
                        r.waypoints.len(),
                        r.reaches_goal
                    ),
                    Err(e) => e.to_string(),
                }
            );
        }
    }
}
