//! Drive the arbiter on a saved phone map (the map stays as saved, so this is
//! the worst case: nothing new gets revealed). Prints progress.
//!
//!   TERRA_MAP=maps/run3.json TERRA_START=-0.2,-0.1,0.6 TERRA_GOAL=2.5,2.0 \
//!   cargo run --release -p terra-autonomy --example drive_on_saved_map
use terra_autonomy::*;
use terra_mapping::MapSnapshot;
use terra_navigation::{Pose, Twist};
use terra_waypoint::{GoalCommand, GoalState};

fn waypoint_arbiter(x: f64, y: f64) -> AutonomyArbiter {
    let mut arbiter = AutonomyArbiter::default();
    arbiter.set_level(LevelRequest {
        level: Level::Waypoint,
        token: "mode".into(),
        budget_seconds: None,
        budget_minutes: None,
    });
    assert!(arbiter.accept_goal(&GoalCommand::Local {
        x,
        y,
        yaw: None,
        token: Some("goal".into()),
    }));
    arbiter
}

fn step(
    arbiter: &mut AutonomyArbiter,
    map: &MapSnapshot,
    pose: Pose,
    measured: Twist,
    now: f64,
) -> ArbiterOutput {
    arbiter.step(ArbiterInput {
        now,
        pose,
        pose_time: now,
        map: Some(map),
        map_time: Some(now),
        map_revision: 1,
        measured,
        healthy: true,
    })
}

fn main() {
    let Ok(path) = std::env::var("TERRA_MAP") else {
        eprintln!("set TERRA_MAP (see the header for the other variables)");
        return;
    };
    let nums = |name: &str| -> Vec<f64> {
        std::env::var(name)
            .unwrap()
            .split(',')
            .map(|s| s.parse().unwrap())
            .collect()
    };
    let (start, goal) = (nums("TERRA_START"), nums("TERRA_GOAL"));
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
    let mut arbiter = waypoint_arbiter(goal[0], goal[1]);
    let mut pose = Pose {
        x: start[0],
        y: start[1],
        yaw: start[2],
    };
    let mut measured = Twist::default();
    let mut last_reason = String::new();
    for tick in 1..=3000 {
        let now = tick as f64 * 0.01;
        let out = step(&mut arbiter, &map, pose, measured, now);
        if out.status.reason != last_reason || tick % 200 == 0 {
            println!(
                "t={now:6.2} pose=({:5.2},{:5.2},{:5.2}) twist=({:5.2},{:5.2}) reason={} route={}",
                pose.x,
                pose.y,
                pose.yaw,
                out.twist.linear,
                out.twist.angular,
                out.status.reason,
                out.status.global_route.len()
            );
            last_reason = out.status.reason.clone();
        }
        if out.goal.state == GoalState::Arrived {
            println!("arrived at t={now:.2}");
            return;
        }
        measured = out.twist;
        pose.yaw += measured.angular * 0.01;
        pose.x += measured.linear * pose.yaw.cos() * 0.01;
        pose.y += measured.linear * pose.yaw.sin() * 0.01;
    }
    println!("not arrived; at ({:.2}, {:.2})", pose.x, pose.y);
}
