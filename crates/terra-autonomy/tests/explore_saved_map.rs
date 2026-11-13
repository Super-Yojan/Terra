//! L4 exploration against a saved phone map used as the "real world": the rover
//! starts knowing nothing, a simulated camera (120 degree view, 5 m, line of
//! sight) reveals the world at 10 Hz while the arbiter runs at 100 Hz, and every
//! frontier event is printed. Unseen cells of the saved map count as walls.
//!
//!   TERRA_MAP=../../maps/run2.json TERRA_START=-1.05,1.65,0 TERRA_SECONDS=120 \
//!   cargo test --release -p terra-autonomy --test explore_saved_map -- --ignored --nocapture
//!
//! Optional: `TERRA_TRACE=from,to` prints pose, command, route and local planner
//! candidates between those times; `TERRA_DUMP=t` prints the known map around
//! the rover at time `t`.
use terra_autonomy::*;
use terra_mapping::MapSnapshot;
use terra_navigation::{Pose, Twist};

#[test]
#[ignore]
fn explore_saved_map() {
    let Ok(path) = std::env::var("TERRA_MAP") else {
        return;
    };
    let start: Vec<f64> = std::env::var("TERRA_START")
        .unwrap()
        .split(',')
        .map(|s| s.parse().unwrap())
        .collect();
    let seconds: f64 = std::env::var("TERRA_SECONDS").map_or(120., |s| s.parse().unwrap());
    let v: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
    let truth = MapSnapshot {
        width: v["width"].as_u64().unwrap() as u32,
        height: v["height"].as_u64().unwrap() as u32,
        resolution: v["resolution"].as_f64().unwrap(),
        origin_x: v["origin_x"].as_f64().unwrap(),
        origin_y: v["origin_y"].as_f64().unwrap(),
        occupancy: v["occupancy"]
            .as_array()
            .unwrap()
            .iter()
            .map(|c| {
                if c.as_i64().unwrap() < 0 {
                    100
                } else {
                    c.as_i64().unwrap() as i8
                }
            })
            .collect(),
    };
    let (w, h, res) = (truth.width as i64, truth.height as i64, truth.resolution);
    let wall = |x: f64, y: f64| {
        let (c, r) = (
            ((x - truth.origin_x) / res).floor() as i64,
            ((y - truth.origin_y) / res).floor() as i64,
        );
        c < 0 || r < 0 || c >= w || r >= h || truth.occupancy[(r * w + c) as usize] >= 65
    };
    let mut map = MapSnapshot {
        occupancy: vec![-1; truth.occupancy.len()],
        ..truth.clone()
    };
    let reveal = |map: &mut MapSnapshot, pose: Pose| {
        let reach = (5. / res).ceil() as i64;
        let (pc, pr) = (
            ((pose.x - truth.origin_x) / res) as i64,
            ((pose.y - truth.origin_y) / res) as i64,
        );
        for r in (pr - reach).max(0)..(pr + reach).min(h) {
            for c in (pc - reach).max(0)..(pc + reach).min(w) {
                let x = truth.origin_x + (c as f64 + 0.5) * res;
                let y = truth.origin_y + (r as f64 + 0.5) * res;
                let (dx, dy) = (x - pose.x, y - pose.y);
                let range = dx.hypot(dy);
                let bearing = (dy.atan2(dx) - pose.yaw + std::f64::consts::PI)
                    .rem_euclid(std::f64::consts::TAU)
                    - std::f64::consts::PI;
                if range > 5. || range < 0.35 || bearing.abs() > 60f64.to_radians() {
                    continue;
                }
                let steps = (range / 0.05).ceil() as usize;
                let blocked = (1..steps).any(|i| {
                    let f = i as f64 / steps as f64;
                    let (sx, sy) = (pose.x + dx * f, pose.y + dy * f);
                    wall(sx, sy) && (sx - x).hypot(sy - y) > 0.08
                });
                if !blocked {
                    let i = (r * w + c) as usize;
                    map.occupancy[i] = truth.occupancy[i];
                }
            }
        }
    };
    let mut arbiter = AutonomyArbiter::default();
    arbiter.set_level(LevelRequest::new(Level::Explore, "explore").with_budget_seconds(seconds));
    let mut pose = Pose {
        x: start[0],
        y: start[1],
        yaw: start[2],
    };
    let mut measured = Twist::default();
    let mut revision = 0u64;
    let mut last_target = None;
    let mut travelled = 0.;
    reveal(&mut map, pose);
    for tick in 1..=(seconds * 100.) as u64 {
        let now = tick as f64 * 0.01;
        if tick % 10 == 0 {
            reveal(&mut map, pose);
            revision += 1;
        }
        let out = arbiter.step(ArbiterInput {
            now,
            pose,
            pose_time: now,
            map: Some(&map),
            map_time: Some(now),
            map_revision: revision,
            measured,
            healthy: true,
        });
        for event in &out.events {
            if event.kind.starts_with("frontier") || event.kind.starts_with("exploration") {
                println!(
                    "t={now:6.2} pose=({:5.2},{:5.2}) {} {}",
                    pose.x, pose.y, event.kind, event.reason
                );
            }
        }
        let target = out
            .exploration
            .as_ref()
            .and_then(|e| e.target.map(|t| (t.x, t.y)));
        if target != last_target || tick % 500 == 0 {
            println!(
                "t={now:6.2} pose=({:5.2},{:5.2},{:5.2}) target={target:?} reason={} route={}",
                pose.x,
                pose.y,
                pose.yaw,
                out.status.reason,
                out.status.global_route.len()
            );
            last_target = target;
        }
        if out
            .exploration
            .as_ref()
            .is_some_and(|e| e.phase == terra_exploration::ExplorePhase::Ended)
        {
            println!("ended: {:?}", out.exploration.unwrap().end_reason);
            break;
        }
        if std::env::var("TERRA_TRACE").is_ok_and(|w| {
            let w: Vec<f64> = w.split(',').map(|s| s.parse().unwrap()).collect();
            now >= w[0] && now <= w[1]
        }) && tick % 25 == 0
        {
            if tick % 50 == 0
                && let Some(trace) = &out.status.planner_trace
            {
                let mut counts = std::collections::BTreeMap::new();
                for c in &trace.candidates {
                    *counts
                        .entry(c.rejection.clone().unwrap_or("ok".into()))
                        .or_insert(0) += 1;
                }
                let ok: Vec<String> = trace
                    .candidates
                    .iter()
                    .step_by(7)
                    .map(|c| {
                        format!(
                            "({:.2},{:.2}){}",
                            c.velocity.linear,
                            c.velocity.angular,
                            if c.rejection.is_some() {
                                let p = c.points_mm.last().unwrap();
                                format!(
                                    "X@({:.2},{:.2})",
                                    trace.origin.x + p[0] as f64 / 1000.,
                                    trace.origin.y + p[1] as f64 / 1000.
                                )
                            } else {
                                String::new()
                            }
                        )
                    })
                    .collect();
                println!(
                    "    age={:.2} origin=({:.2},{:.2},{:.2}) {counts:?} sel={:?} ok={:?}",
                    trace.age,
                    trace.origin.x,
                    trace.origin.y,
                    trace.origin.yaw,
                    trace.selected.map(|i| (
                        trace.candidates[i].velocity.linear,
                        trace.candidates[i].velocity.angular
                    )),
                    ok
                );
            }
            if tick % 100 == 0 {
                println!(
                    "  route {:?}",
                    out.status
                        .global_route
                        .iter()
                        .map(|p| format!("({:.2},{:.2})", p[0], p[1]))
                        .collect::<Vec<_>>()
                );
            }
            println!(
                "  trace t={now:.2} pose=({:.2},{:.2},{:.2}) twist=({:.2},{:.2}) reason={} src={}",
                pose.x,
                pose.y,
                pose.yaw,
                out.twist.linear,
                out.twist.angular,
                out.status.reason,
                out.status.active_source.as_str()
            );
        }
        if std::env::var("TERRA_DUMP")
            .is_ok_and(|t| (now - t.parse::<f64>().unwrap()).abs() < 0.005)
        {
            for i in 0..map.occupancy.len() {
                let (c, r) = ((i as i64) % w, (i as i64) / w);
                let x = truth.origin_x + (c as f64 + 0.5) * res;
                let y = truth.origin_y + (r as f64 + 0.5) * res;
                if (x - pose.x).hypot(y - pose.y) < 0.8 && map.occupancy[i] >= 65 {
                    println!("OCC ({x:.2},{y:.2}) v={}", map.occupancy[i]);
                }
            }
            for r in (0..h).rev() {
                let y = truth.origin_y + (r as f64 + 0.5) * res;
                if (y - pose.y).abs() > 1.2 {
                    continue;
                }
                let line: String =
                    (0..w)
                        .filter_map(|c| {
                            let x = truth.origin_x + (c as f64 + 0.5) * res;
                            if (x - pose.x).abs() > 1.2 {
                                return None;
                            }
                            let v = map.occupancy[(r * w + c) as usize];
                            Some(
                                if (x - pose.x).abs() < res / 2. && (y - pose.y).abs() < res / 2. {
                                    'R'
                                } else if out.status.global_route.iter().any(|p| {
                                    (p[0] - x).abs() < res / 2. && (p[1] - y).abs() < res / 2.
                                }) {
                                    '*'
                                } else if v < 0 {
                                    '?'
                                } else if v >= 65 {
                                    '#'
                                } else {
                                    '.'
                                },
                            )
                        })
                        .collect();
                println!("{y:5.2} {line}");
            }
        }
        measured = out.twist;
        pose.yaw += measured.angular * 0.01;
        let step = measured.linear * 0.01;
        pose.x += step * pose.yaw.cos();
        pose.y += step * pose.yaw.sin();
        travelled += step.abs();
        assert!(!wall(pose.x, pose.y), "drove into a wall at {pose:?}");
    }
    let known = map.occupancy.iter().filter(|v| **v >= 0).count();
    println!("travelled {travelled:.1} m, known cells {known}");
}
