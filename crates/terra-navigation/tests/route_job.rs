//! RouteJob: Hybrid A* waypoint routing toward goals the rover may not have seen,
//! advanced a bounded number of expansions per control tick. The maps copy what
//! the phone builds from one view (as in astar_hidden_goals.rs).
use std::f64::consts::PI;
use terra_mapping::MapSnapshot;
use terra_navigation::{HybridConfig, HybridRoute, Pose, RouteJob, footprint_rejection_at};

/// L3 arrival radius minus a margin, as the arbiter passes it.
const SNAP: f64 = 0.65;
/// Expansions per control tick.
const BUDGET: u32 = 400;

fn blank(fill: i8) -> MapSnapshot {
    MapSnapshot {
        width: 200,
        height: 200,
        resolution: 0.1,
        origin_x: -10.,
        origin_y: -10.,
        occupancy: vec![fill; 40_000],
    }
}

fn centre(i: usize) -> (f64, f64) {
    (
        -10. + ((i % 200) as f64 + 0.5) * 0.1,
        -10. + ((i / 200) as f64 + 0.5) * 0.1,
    )
}

/// A box face at `face_x`, the shadow behind it unknown, the rest of the view free.
fn view_with_box(face_x: f64, half_width: f64) -> MapSnapshot {
    let mut map = blank(-1);
    for i in 0..40_000 {
        let (x, y) = centre(i);
        let seen = (x > -1. && x.hypot(y) < 6.) || x.hypot(y) < 1.;
        if !seen {
            continue;
        }
        if (face_x..face_x + 0.2).contains(&x) && y.abs() <= half_width {
            map.occupancy[i] = 100;
            continue;
        }
        let shadow = x >= face_x + 0.2 && y.abs() <= half_width * x / face_x;
        map.occupancy[i] = if shadow { -1 } else { 0 };
    }
    map
}

fn rover() -> Pose {
    Pose {
        x: 0.,
        y: 0.,
        yaw: 0.,
    }
}

/// Advance a tick at a time; returns the result and the number of ticks.
fn plan(
    map: &MapSnapshot,
    start: Pose,
    x: f64,
    y: f64,
) -> (Result<HybridRoute, &'static str>, usize) {
    let mut job = match RouteJob::new(map, start, x, y, &HybridConfig::default(), SNAP) {
        Ok(job) => job,
        Err(why) => return (Err(why), 0),
    };
    for tick in 1..10_000 {
        if let Some(result) = job.advance(BUDGET) {
            return (result, tick);
        }
    }
    panic!("route job never finished");
}

/// Every sample fits in known-free space (or under the starting body) and no
/// turn is tighter than the minimum radius.
fn assert_drivable(map: &MapSnapshot, start: Pose, route: &HybridRoute) {
    let config = HybridConfig::default();
    assert!(route.waypoints.len() >= 2, "{route:?}");
    let kappa = 1. / config.min_turn_radius;
    for w in &route.waypoints {
        let pose = Pose {
            x: w.x,
            y: w.y,
            yaw: w.yaw,
        };
        assert_eq!(
            footprint_rejection_at(map, &config.footprint, pose, 0., Some(start)),
            None,
            "at {w:?}"
        );
        assert!(w.curvature.abs() <= kappa + 1e-9);
    }
    for pair in route.waypoints.windows(2) {
        let chord = (pair[1].x - pair[0].x).hypot(pair[1].y - pair[0].y);
        assert!(chord > 1e-6, "no turning in place");
        let turn = (pair[1].yaw - pair[0].yaw + PI).rem_euclid(2. * PI) - PI;
        assert!(turn.abs() <= 2. * (kappa * chord / 2.).min(1.).asin() + 1e-6);
    }
}

#[test]
fn goal_behind_a_box_routes_round_a_side() {
    let map = view_with_box(2.0, 0.6);
    let (route, _) = plan(&map, rover(), 2.6, 0.);
    let route = route.expect("goal behind a box must give a route");
    assert_drivable(&map, rover(), &route);
    assert!(!route.reaches_goal, "the goal is in the unseen shadow");
    let end = route.waypoints.last().unwrap();
    assert!(end.y.abs() > 0.6, "should head round a side: {end:?}");
}

#[test]
fn rover_at_the_box_face_still_routes_round() {
    let map = view_with_box(2.0, 0.6);
    let at_face = Pose {
        x: 1.25,
        y: 0.,
        yaw: 0.,
    };
    let (route, _) = plan(&map, at_face, 2.6, 0.);
    let route = route.expect("rover parked at the box face must still get a route");
    assert_drivable(&map, at_face, &route);
    assert!(route.waypoints.last().unwrap().y.abs() > 0.3);
}

#[test]
fn goal_on_the_box_is_refused() {
    let map = view_with_box(2.0, 0.6);
    assert_eq!(plan(&map, rover(), 2.1, 0.).0, Err("route_goal_blocked"));
}

#[test]
fn known_goal_through_an_unseen_band_drives_to_its_edge() {
    let mut map = blank(0);
    for i in 0..40_000 {
        if (3.0..4.0).contains(&centre(i).0) {
            map.occupancy[i] = -1;
        }
    }
    let (route, _) = plan(&map, rover(), 6., 0.);
    let route = route.expect("known goal through unseen space must give a route");
    assert_drivable(&map, rover(), &route);
    assert!(!route.reaches_goal);
    assert!(
        route.waypoints.last().unwrap().x > 2.0,
        "should drive up to the band"
    );
}

#[test]
fn an_enclosed_goal_is_refused() {
    let mut map = blank(-1);
    for i in 0..40_000 {
        let (x, y) = centre(i);
        if x.hypot(y) < 2. {
            map.occupancy[i] = 0;
        }
        let inside = (4.0..7.0).contains(&x) && (-1.5..1.5).contains(&y);
        let wall = (3.8..7.2).contains(&x) && (-1.7..1.7).contains(&y) && !inside;
        if wall {
            map.occupancy[i] = 100;
        } else if inside {
            map.occupancy[i] = 0;
        }
    }
    assert_eq!(plan(&map, rover(), 5.5, 0.).0, Err("route_no_path"));
}

#[test]
fn a_goal_touching_the_box_snaps_to_where_the_body_fits() {
    let mut map = blank(0);
    for i in 0..40_000 {
        let (x, y) = centre(i);
        if (2.0..2.2).contains(&x) && y.abs() <= 0.6 {
            map.occupancy[i] = 100;
        }
    }
    let goal = (2.3, 0.);
    let (route, _) = plan(&map, rover(), goal.0, goal.1);
    let route = route.expect("goal near the box must give a route");
    assert_drivable(&map, rover(), &route);
    let end = route.waypoints.last().unwrap();
    assert!(
        (end.x - goal.0).hypot(end.y - goal.1) <= SNAP + 0.15,
        "end {end:?}"
    );
    assert!(end.x > 2.2, "should end behind the box: {end:?}");
    assert!(
        !route.reaches_goal,
        "a snapped goal is not the requested one"
    );
}

#[test]
fn an_off_map_goal_heads_for_the_map_edge() {
    let map = blank(0);
    let (route, _) = plan(&map, rover(), 25., 0.);
    let route = route.expect("off-map goal must give a route toward it");
    assert_drivable(&map, rover(), &route);
    assert!(!route.reaches_goal);
    assert!(route.waypoints.last().unwrap().x > 9.0);
}

#[test]
fn a_known_route_completes_and_reaches_the_goal() {
    let map = blank(0);
    let (route, ticks) = plan(&map, rover(), 4., 2.);
    let route = route.unwrap();
    assert_drivable(&map, rover(), &route);
    assert!(route.reaches_goal);
    assert!(ticks < 20, "an open-map route takes {ticks} ticks");
}

/// Timing on the saved phone maps: `cargo test --release -p terra-navigation
/// --test route_job per_tick_cost -- --ignored --nocapture` from the repo root.
#[test]
#[ignore]
fn per_tick_cost_on_saved_maps() {
    use std::time::Instant;
    for (path, start, goals) in [
        (
            "../../maps/run2.json",
            (-1.05, 1.65),
            vec![(2.35, 4.85), (-10.05, -1.15), (1.75, -0.95), (-1.25, 6.0)],
        ),
        (
            "../../maps/run1.json",
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
