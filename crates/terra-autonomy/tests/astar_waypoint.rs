use terra_autonomy::*;
use terra_mapping::{MapConfig, MapSnapshot};
use terra_navigation::{LocalPlanner, Pose, Twist};
use terra_waypoint::{GoalCommand, GoalState};
/// Build a free 10 m square grid with a 6 m vertical wall at x = 4 m, leaving a passage around its upper end.
fn map() -> MapSnapshot {
    let mut map = MapSnapshot {
        width: 40,
        height: 40,
        resolution: 0.25,
        origin_x: 0.,
        origin_y: 0.,
        occupancy: vec![0; 1600],
    };
    for row in 0..24 {
        map.occupancy[row * 40 + 16] = 100;
    }
    map
}
/// Integrate waypoint commands around a partial wall; require arrival while the published destination and token stay unchanged.
#[test]
fn l3_follows_corner_route_and_preserves_original_destination() {
    let mut arbiter = AutonomyArbiter::default();
    arbiter.set_level(LevelRequest {
        level: Level::Waypoint,
        token: "astar-mode".into(),
        budget_seconds: None,
        budget_minutes: None,
    });
    assert!(arbiter.accept_goal(
        &GoalCommand::Local {
            x: 7.,
            y: 2.,
            yaw: None,
            token: Some("astar-goal".into())
        },
        20.
    ));
    let map = map();
    let mut pose = Pose {
        x: 2.,
        y: 2.,
        yaw: 0.,
    };
    let mut measured = Twist::default();
    let mut arrived = false;
    for tick in 1..=2000 {
        let now = tick as f64 * 0.1;
        let output = arbiter.step(ArbiterInput {
            now,
            pose,
            pose_time: now,
            map: Some(&map),
            map_time: Some(now),
            map_revision: 1,
            measured,
            healthy: true,
        });
        assert_eq!((output.goal.x, output.goal.y), (7., 2.));
        assert_eq!(output.goal.token.as_deref(), Some("astar-goal"));
        if output.goal.state == GoalState::Arrived {
            arrived = true;
            break;
        }
        assert!(
            !output.status.global_route.is_empty()
                || tick <= 20 && output.status.reason == "route_planning",
            "{}",
            output.status.reason
        );
        measured = output.twist;
        pose.yaw += measured.angular * 0.1;
        pose.x += measured.linear * pose.yaw.cos() * 0.1;
        pose.y += measured.linear * pose.yaw.sin() * 0.1;
    }
    assert!(arrived, "route follower failed to arrive: {pose:?}");
}
/// Extend the fixture wall across the map; require zero motion with an active goal and a no-path reason.
#[test]
fn l3_holds_when_route_is_sealed() {
    let mut arbiter = AutonomyArbiter::default();
    arbiter.set_level(LevelRequest {
        level: Level::Waypoint,
        token: "mode".into(),
        budget_seconds: None,
        budget_minutes: None,
    });
    arbiter.accept_goal(
        &GoalCommand::Local {
            x: 7.,
            y: 2.,
            yaw: None,
            token: None,
        },
        20.,
    );
    let mut map = map();
    for row in 0..40 {
        map.occupancy[row * 40 + 16] = 100;
    }
    let output = settle(&mut arbiter, |dt| input(&map, 0.1 + dt, 1));
    assert_eq!(output.twist, Twist::default());
    assert_eq!(output.status.reason, "route_no_path");
    assert_eq!(output.goal.state, GoalState::Active);
}

/// Seal a previously passable wall and increment map revision; clear the route while retaining the original goal token.
#[test]
fn changed_map_invalidates_route_without_losing_the_goal() {
    let mut arbiter = AutonomyArbiter::default();
    arbiter.set_level(LevelRequest {
        level: Level::Waypoint,
        token: "mode".into(),
        budget_seconds: None,
        budget_minutes: None,
    });
    arbiter.accept_goal(
        &GoalCommand::Local {
            x: 7.,
            y: 2.,
            yaw: None,
            token: Some("keep-goal".into()),
        },
        20.,
    );
    let mut map = map();
    assert!(
        !settle(&mut arbiter, |dt| input(&map, 0.1 + dt, 1))
            .status
            .global_route
            .is_empty()
    );
    for row in 0..40 {
        map.occupancy[row * 40 + 16] = 100;
    }
    // The blocked route is dropped at once: the rover holds while it replans.
    let held = arbiter.step(input(&map, 0.4, 2));
    assert_eq!(held.twist, Twist::default());
    let output = settle(&mut arbiter, |dt| input(&map, 0.41 + dt, 2));
    assert_eq!(output.twist, Twist::default());
    assert_eq!(output.status.reason, "route_no_path");
    assert_eq!(output.goal.token.as_deref(), Some("keep-goal"));
    assert!(output.status.global_route.is_empty());
}

/// Routes are searched a bounded amount per tick: repeat the evidence (10 ms
/// later each time) while the arbiter reports `route_planning` (at most 50 ticks).
fn settle<'a>(
    arbiter: &mut AutonomyArbiter,
    mut evidence: impl FnMut(f64) -> ArbiterInput<'a>,
) -> ArbiterOutput {
    for tick in 0..50 {
        // The arbiter's clock must move forward: 10 ms per repeat.
        let output = arbiter.step(evidence(tick as f64 * 0.01));
        if output.status.reason != "route_planning" {
            return output;
        }
    }
    panic!("route search did not finish within 50 ticks");
}

/// Build fresh healthy evidence at (2 m, 2 m), using the supplied clock time and map revision.
fn input(map: &MapSnapshot, now: f64, revision: u64) -> ArbiterInput<'_> {
    ArbiterInput {
        now,
        pose: Pose {
            x: 2.,
            y: 2.,
            yaw: 0.,
        },
        pose_time: now,
        map: Some(map),
        map_time: Some(now),
        map_revision: revision,
        measured: Twist::default(),
        healthy: true,
    }
}

/// Jump the rover to the other side of a wall without changing map revision; require a replacement route around the wall.
#[test]
fn localization_jump_replans_without_a_map_revision_change() {
    let mut map = MapSnapshot {
        width: 40,
        height: 40,
        resolution: 0.25,
        origin_x: 0.,
        origin_y: 0.,
        occupancy: vec![0; 1600],
    };
    for col in 8..40 {
        map.occupancy[16 * 40 + col] = 100;
    }
    let mut arbiter = AutonomyArbiter::default();
    arbiter.set_level(LevelRequest {
        level: Level::Waypoint,
        token: "mode".into(),
        budget_seconds: None,
        budget_minutes: None,
    });
    arbiter.accept_goal(
        &GoalCommand::Local {
            x: 7.,
            y: 2.,
            yaw: None,
            token: None,
        },
        20.,
    );
    assert!(
        !settle(&mut arbiter, |dt| input(&map, 0.1 + dt, 1))
            .status
            .global_route
            .is_empty()
    );
    // The rover is now far from its route: it drops it and replans from here.
    let jumped = |dt| ArbiterInput {
        pose: Pose {
            x: 5.,
            y: 7.,
            yaw: 0.,
        },
        ..input(&map, 0.6 + dt, 1)
    };
    let output = settle(&mut arbiter, jumped);
    assert!(!output.status.global_route.is_empty());
    assert!(output.status.global_route.iter().any(|p| p[0] < 2.));
}

/// Add an obstacle that blocks the cached rollout while leaving a route segment visible; require a newly admissible command or stop.
#[test]
fn cached_waypoint_command_must_stay_admissible() {
    let map = MapSnapshot {
        width: 120,
        height: 80,
        resolution: 0.1,
        origin_x: 0.,
        origin_y: 0.,
        occupancy: vec![0; 120 * 80],
    };
    let mut arbiter = AutonomyArbiter::default();
    arbiter.planner.config.footprint = terra_navigation::Footprint {
        length: 0.28,
        width: 0.28,
        margin: 0.,
    };
    arbiter.set_level(LevelRequest {
        level: Level::Waypoint,
        token: "cache".into(),
        budget_seconds: None,
        budget_minutes: None,
    });
    assert!(arbiter.accept_goal(
        &GoalCommand::Local {
            x: 8.,
            y: 4.,
            yaw: None,
            token: Some("goal".into()),
        },
        20.,
    ));
    let pose = Pose {
        x: 2.,
        y: 4.,
        yaw: 0.2,
    };
    let first = settle(&mut arbiter, |dt| ArbiterInput {
        now: 0.1 + dt,
        pose,
        pose_time: 0.1 + dt,
        map: Some(&map),
        map_time: Some(0.1 + dt),
        map_revision: 1,
        measured: Twist {
            linear: 1.,
            angular: 0.8,
        },
        healthy: true,
    });
    assert!(first.twist.linear > 0.5, "{:?}", first.twist);
    let planner = LocalPlanner {
        config: arbiter.planner.config.clone(),
    };
    let mut blocking = None;
    for index in 0..map.occupancy.len() {
        let mut blocked = map.clone();
        blocked.occupancy[index] = 100;
        if planner.admissible(&blocked, pose, first.twist) {
            continue;
        }
        let visible = first.status.global_route.iter().any(|point| {
            (point[0] - pose.x).hypot(point[1] - pose.y) > 0.3
                && terra_navigation::route_segment_clear(
                    &blocked,
                    pose,
                    arbiter.planner.config.footprint.circumradius(),
                    [pose.x, pose.y],
                    *point,
                )
        });
        if visible {
            blocking = Some(index);
            break;
        }
    }
    let index = blocking.expect("arc obstacle that leaves the route visible");
    let mut blocked = map;
    blocked.occupancy[index] = 100;
    let second = arbiter.step(ArbiterInput {
        now: 0.15,
        pose,
        pose_time: 0.15,
        map: Some(&blocked),
        map_time: Some(0.15),
        map_revision: 2,
        measured: Twist {
            linear: 1.,
            angular: 0.8,
        },
        healthy: true,
    });
    assert!(
        planner.admissible(&blocked, pose, second.twist) || second.twist == Twist::default(),
        "replayed inadmissible {:?}",
        second.twist
    );
    assert_ne!(second.twist, first.twist);
}

/// Simulate travel across an 18 m by 12 m pitch with a rover-centred map and a 9 m reveal radius; require eventual waypoint arrival.
#[test]
fn l3_crosses_an_18_by_12_pitch_on_the_rolling_map() {
    let config = MapConfig::default();
    assert_eq!(
        (config.width, config.height, config.resolution),
        (200, 200, 0.1)
    );
    let mut arbiter = AutonomyArbiter::default();
    arbiter.set_level(LevelRequest {
        level: Level::Waypoint,
        token: "pitch".into(),
        budget_seconds: None,
        budget_minutes: None,
    });
    let goal = (17., 6.);
    assert!(arbiter.accept_goal(
        &GoalCommand::Local {
            x: goal.0,
            y: goal.1,
            yaw: None,
            token: Some("far-goal".into()),
        },
        30.,
    ));
    let mut pose = Pose {
        x: 1.,
        y: 6.,
        yaw: 0.,
    };
    let mut measured = Twist::default();
    let mut arrived = false;
    let mut saw_progress = false;
    for tick in 1..=2_500 {
        let now = tick as f64 * 0.1;
        let map = rolling_pitch(&config, pose);
        let output = arbiter.step(ArbiterInput {
            now,
            pose,
            pose_time: now,
            map: Some(&map),
            map_time: Some(now),
            map_revision: tick as u64,
            measured,
            healthy: true,
        });
        assert_eq!((output.goal.x, output.goal.y), goal);
        assert_eq!(output.goal.token.as_deref(), Some("far-goal"));
        if output.goal.state == GoalState::Arrived {
            arrived = true;
            break;
        }
        if !saw_progress && output.status.reason == "route_planning" {
            assert!(tick <= 20, "still planning after {tick} ticks");
        } else if !saw_progress {
            assert_ne!(output.twist, Twist::default(), "{}", output.status.reason);
            let end = output.status.global_route.last().copied().unwrap();
            assert!(
                (end[0] - goal.0).hypot(end[1] - goal.1) < (pose.x - goal.0).hypot(pose.y - goal.1)
            );
            saw_progress = true;
        }
        measured = output.twist;
        pose.yaw += measured.angular * 0.1;
        pose.x += measured.linear * pose.yaw.cos() * 0.1;
        pose.y += measured.linear * pose.yaw.sin() * 0.1;
    }
    assert!(saw_progress);
    assert!(arrived, "did not cross the pitch: {pose:?}");
    assert!((pose.x - goal.0).hypot(pose.y - goal.1) <= 0.75);
}

/// Build a rover-centred snapshot, revealing cells within 9 m; pitch interior is free, exterior occupied, and unseen cells unknown.
fn rolling_pitch(config: &MapConfig, pose: Pose) -> MapSnapshot {
    let resolution = config.resolution;
    let origin_x =
        (pose.x / resolution).round() * resolution - config.width as f64 * resolution / 2.0;
    let origin_y =
        (pose.y / resolution).round() * resolution - config.height as f64 * resolution / 2.0;
    let mut occupancy = vec![-1i8; config.width as usize * config.height as usize];
    for row in 0..config.height {
        for col in 0..config.width {
            let x = origin_x + (col as f64 + 0.5) * resolution;
            let y = origin_y + (row as f64 + 0.5) * resolution;
            if (x - pose.x).hypot(y - pose.y) > 9.0 {
                continue;
            }
            let on_pitch = (0.0..18.0).contains(&x) && (0.0..12.0).contains(&y);
            occupancy[(row * config.width + col) as usize] = if on_pitch { 0 } else { 100 };
        }
    }
    MapSnapshot {
        width: config.width,
        height: config.height,
        resolution,
        origin_x,
        origin_y,
        occupancy,
    }
}

/// A wheel stalled in the motor deadband reads zero speed. The planner's
/// window must still grow from its own last command, or L3 never gets past
/// 0.1 m/s.
#[test]
fn stalled_measured_speed_does_not_pin_waypoint_speed() {
    let map = MapSnapshot {
        width: 120,
        height: 80,
        resolution: 0.1,
        origin_x: 0.,
        origin_y: 0.,
        occupancy: vec![0; 120 * 80],
    };
    let mut arbiter = AutonomyArbiter::default();
    arbiter.planner.config.footprint = terra_navigation::Footprint {
        length: 0.28,
        width: 0.28,
        margin: 0.,
    };
    arbiter.set_level(LevelRequest {
        level: Level::Waypoint,
        token: "stall".into(),
        budget_seconds: None,
        budget_minutes: None,
    });
    assert!(arbiter.accept_goal(
        &GoalCommand::Local {
            x: 10.,
            y: 4.,
            yaw: None,
            token: Some("goal".into()),
        },
        20.,
    ));
    let pose = Pose {
        x: 2.,
        y: 4.,
        yaw: 0.,
    };
    let mut linear = 0.;
    for tick in 1..=8 {
        let now = tick as f64 * 0.1;
        let out = arbiter.step(ArbiterInput {
            now,
            pose,
            pose_time: now,
            map: Some(&map),
            map_time: Some(now),
            map_revision: 1,
            measured: Twist::default(),
            healthy: true,
        });
        linear = out.twist.linear;
    }
    assert!(linear > 0.4, "{linear}");
}

/// Issue #109 end to end: a box stands between the rover and an L3 goal just
/// behind it. The map starts empty (unknown) and a simulated camera (120° view,
/// 5 m range, line of sight) reveals the true world as the rover drives, the
/// way the phone map fills in. The rover must reach the goal without ever
/// putting its footprint on the box.
#[test]
fn l3_reaches_a_goal_behind_a_box_as_the_map_fills_in() {
    for goal in [[3.4, 0.0], [5.0, 0.3]] {
        let (arrived, pose, reasons) = drive_behind_box(goal);
        assert!(
            arrived,
            "goal {goal:?} not reached, stopped at {pose:?}; reasons {reasons:?}"
        );
    }
}

/// Run the arbiter against a box at x 2.4..2.6, |y| <= 0.6, revealing the map
/// from the rover's pose each tick. Returns (arrived, final pose, hold reasons).
fn drive_behind_box(goal: [f64; 2]) -> (bool, Pose, Vec<String>) {
    let occupied = |x: f64, y: f64| (2.4..2.6).contains(&x) && y.abs() <= 0.6;
    let mut map = MapSnapshot {
        width: 120,
        height: 120,
        resolution: 0.1,
        origin_x: -6.,
        origin_y: -6.,
        occupancy: vec![-1; 120 * 120],
    };
    let reveal = |map: &mut MapSnapshot, pose: Pose| {
        for row in 0..120 {
            for col in 0..120 {
                let x = -6. + (col as f64 + 0.5) * 0.1;
                let y = -6. + (row as f64 + 0.5) * 0.1;
                let (dx, dy) = (x - pose.x, y - pose.y);
                let range = dx.hypot(dy);
                let bearing = (dy.atan2(dx) - pose.yaw + std::f64::consts::PI)
                    .rem_euclid(std::f64::consts::TAU)
                    - std::f64::consts::PI;
                if range > 5. || bearing.abs() > 60f64.to_radians() {
                    continue;
                }
                // Line of sight: march toward the cell; stop at the first obstacle.
                let steps = (range / 0.05).ceil() as usize;
                let blocked = (1..steps).any(|i| {
                    let f = i as f64 / steps as f64;
                    let (sx, sy) = (pose.x + dx * f, pose.y + dy * f);
                    occupied(sx, sy) && ((sx - x).hypot(sy - y) > 0.08)
                });
                if !blocked {
                    map.occupancy[row * 120 + col] = if occupied(x, y) { 100 } else { 0 };
                }
            }
        }
    };
    let mut arbiter = AutonomyArbiter::default();
    arbiter.set_level(LevelRequest {
        level: Level::Waypoint,
        token: "mode".into(),
        budget_seconds: None,
        budget_minutes: None,
    });
    assert!(arbiter.accept_goal(
        &GoalCommand::Local {
            x: goal[0],
            y: goal[1],
            yaw: None,
            token: None,
        },
        20.
    ));
    let mut pose = Pose::default();
    let mut measured = Twist::default();
    let mut reasons = Vec::new();
    // The rover has looked around where it stands before the goal is sent.
    for yaw in [0., 2.1, -2.1] {
        reveal(&mut map, Pose { yaw, ..pose });
    }
    for tick in 1..=3000 {
        let now = tick as f64 * 0.1;
        reveal(&mut map, pose);
        let output = arbiter.step(ArbiterInput {
            now,
            pose,
            pose_time: now,
            map: Some(&map),
            map_time: Some(now),
            map_revision: tick,
            measured,
            healthy: true,
        });
        if output.goal.state == GoalState::Arrived {
            return (true, pose, reasons);
        }
        if output.twist == Twist::default() && reasons.last() != Some(&output.status.reason) {
            reasons.push(output.status.reason.clone());
        }
        measured = output.twist;
        pose.yaw += measured.angular * 0.1;
        pose.x += measured.linear * pose.yaw.cos() * 0.1;
        pose.y += measured.linear * pose.yaw.sin() * 0.1;
        // Terra's 0.61 m body rectangle, at its heading, never overlaps the box.
        assert!(body_clear_of_box(pose), "body hit the box at {pose:?}");
    }
    (false, pose, reasons)
}

/// Whether Terra's 0.61 m square body at `pose` stays clear of the box
/// x 2.4..2.6, y -0.6..0.6 (separating axes: box axes and body axes).
fn body_clear_of_box(pose: Pose) -> bool {
    let half = 0.61 / 2.;
    let (sin, cos) = pose.yaw.sin_cos();
    let corners = [(1., 1.), (1., -1.), (-1., -1.), (-1., 1.)].map(|(u, v): (f64, f64)| {
        (
            pose.x + half * (u * cos - v * sin),
            pose.y + half * (u * sin + v * cos),
        )
    });
    let box_corners = [(2.4, -0.6), (2.6, -0.6), (2.6, 0.6), (2.4, 0.6)];
    let separated = |axis: (f64, f64)| {
        let project = |p: &(f64, f64)| p.0 * axis.0 + p.1 * axis.1;
        let (a_min, a_max) = corners
            .iter()
            .map(project)
            .fold((f64::MAX, f64::MIN), |(lo, hi), v| (lo.min(v), hi.max(v)));
        let (b_min, b_max) = box_corners
            .iter()
            .map(project)
            .fold((f64::MAX, f64::MIN), |(lo, hi), v| (lo.min(v), hi.max(v)));
        a_max < b_min || b_max < a_min
    };
    [(1., 0.), (0., 1.), (cos, sin), (-sin, cos)]
        .into_iter()
        .any(separated)
}
