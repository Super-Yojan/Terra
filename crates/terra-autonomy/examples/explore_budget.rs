//! Kinematic exploration run. Writes one JSON object per tick.
//!
//! ```sh
//! cargo run -p terra-autonomy --example explore_budget -- /tmp/explore.jsonl
//! ```
use std::fs::File;
use std::io::{BufWriter, Write};
use terra_autonomy::*;
use terra_mapping::MapSnapshot;
use terra_navigation::{Pose, Twist};

fn main() {
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "explore.jsonl".into());
    let budget = 4.0;
    let mut arbiter = AutonomyArbiter::default();
    arbiter.set_level(LevelRequest::new(Level::Explore, "demo-1").with_budget_seconds(budget));
    let mut map = MapSnapshot {
        width: 80,
        height: 80,
        resolution: 0.25,
        origin_x: -10.,
        origin_y: -10.,
        occupancy: vec![-1; 6400],
    };
    for index in 0..map.occupancy.len() {
        let x = map.origin_x + (index % 80) as f64 * map.resolution;
        let y = map.origin_y + (index / 80) as f64 * map.resolution;
        if x.hypot(y) <= 3. {
            map.occupancy[index] = 0;
        }
    }
    let mut pose = Pose::default();
    let mut measured = Twist::default();
    let mut file = BufWriter::new(File::create(&path).expect("trace file"));
    let mut coverage = 0.0_f64;
    for step in 0..50 {
        let now = step as f64 * 0.1;
        let output = arbiter.step(ArbiterInput {
            now,
            pose,
            pose_time: now,
            map: Some(&map),
            map_time: Some(now),
            map_revision: step as u64,
            measured,
            healthy: true,
        });
        let status = output.exploration.clone().expect("exploration status");
        coverage = coverage.max(status.coverage_m2);
        let line = serde_json::json!({
            "t": now,
            "x": pose.x,
            "y": pose.y,
            "yaw": pose.yaw,
            "linear": output.twist.linear,
            "angular": output.twist.angular,
            "coverage_m2": status.coverage_m2,
            "phase": status.phase,
            "end_reason": status.end_reason,
            "reason": output.status.reason,
            "target": status.target,
            "width": map.width,
            "height": map.height,
            "resolution": map.resolution,
            "origin_x": map.origin_x,
            "origin_y": map.origin_y,
            "occupancy": map.occupancy,
        });
        serde_json::to_writer(&mut file, &line).unwrap();
        file.write_all(b"\n").unwrap();
        let dt = 0.1;
        pose.yaw += output.twist.angular * dt;
        pose.x += output.twist.linear * pose.yaw.cos() * dt;
        pose.y += output.twist.linear * pose.yaw.sin() * dt;
        measured = output.twist;
        for index in 0..map.occupancy.len() {
            let x = map.origin_x + (index % map.width as usize) as f64 * map.resolution;
            let y = map.origin_y + (index / map.width as usize) as f64 * map.resolution;
            if (x - pose.x).hypot(y - pose.y) <= 2.2 && map.occupancy[index] < 0 {
                map.occupancy[index] = 0;
            }
        }
        if status.phase == ExplorePhase::Ended {
            println!(
                "ended at {now:.1}s reason {} coverage {coverage:.2} m2 pose ({:.2}, {:.2})",
                status.end_reason.unwrap().as_str(),
                pose.x,
                pose.y
            );
            break;
        }
    }
    println!("wrote {path}");
}
