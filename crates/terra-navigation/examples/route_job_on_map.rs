//! Run a waypoint RouteJob on a saved map the way the arbiter does (same
//! weight, cap and per-tick budget) and report what happens.
//!
//! cargo run -p terra-navigation --release --example route_job_on_map -- maps/run3.json x y yaw goal_x goal_y
//!
//! The phone map is centred on the rover, so `x y` is usually the map centre.
use terra_mapping::MapSnapshot;
use terra_navigation::{HybridConfig, Pose, RouteJob};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let [path, x, y, yaw, gx, gy] = &args[..] else {
        panic!("usage: route_job_on_map <map.json> x y yaw goal_x goal_y");
    };
    let n = |s: &String| s.parse::<f64>().expect("number");
    let v: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
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
    let pose = Pose {
        x: n(x),
        y: n(y),
        yaw: n(yaw),
    };
    // As the arbiter: weight 2, 30k cap, 150 expansions per tick, snap 0.65 m.
    let config = HybridConfig {
        max_expansions: 30_000,
        heuristic_weight: 2.0,
        ..HybridConfig::default()
    };
    let mut job = match RouteJob::new(&map, pose, n(gx), n(gy), &config, 0.65) {
        Ok(job) => job,
        Err(why) => {
            println!("RouteJob::new failed: {why}");
            return;
        }
    };
    println!("routing to {:?}", job.goal());
    for tick in 1..=1000 {
        if let Some(result) = job.advance(150) {
            match result {
                Ok(route) => println!(
                    "tick {tick}: route with {} samples, reaches_goal={}, ends at ({:.2}, {:.2})",
                    route.waypoints.len(),
                    route.reaches_goal,
                    route.waypoints.last().unwrap().x,
                    route.waypoints.last().unwrap().y
                ),
                Err(why) => println!("tick {tick}: {why}"),
            }
            return;
        }
    }
    println!("still searching after 1000 ticks");
}
