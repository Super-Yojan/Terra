use terra_autonomy::*;
use terra_mapping::{MapConfig, MapSnapshot};
use terra_navigation::{LocalPlanner, Pose, Twist};
use terra_waypoint::{GoalCommand, GoalState};
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
            !output.status.global_route.is_empty(),
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
    let output = arbiter.step(ArbiterInput {
        now: 0.1,
        pose: Pose {
            x: 2.,
            y: 2.,
            yaw: 0.,
        },
        pose_time: 0.1,
        map: Some(&map),
        map_time: Some(0.1),
        map_revision: 1,
        measured: Twist::default(),
        healthy: true,
    });
    assert_eq!(output.twist, Twist::default());
    assert_eq!(output.status.reason, "route_no_path");
    assert_eq!(output.goal.state, GoalState::Active);
}

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
        !arbiter
            .step(input(&map, 0.1, 1))
            .status
            .global_route
            .is_empty()
    );
    for row in 0..40 {
        map.occupancy[row * 40 + 16] = 100;
    }
    let output = arbiter.step(input(&map, 0.4, 2));
    assert_eq!(output.twist, Twist::default());
    assert_eq!(output.status.reason, "route_no_path");
    assert_eq!(output.goal.token.as_deref(), Some("keep-goal"));
    assert!(output.status.global_route.is_empty());
}

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
    let mut evidence = input(&map, 0.1, 1);
    assert!(!arbiter.step(evidence).status.global_route.is_empty());
    evidence = input(&map, 0.2, 1);
    evidence.pose = Pose {
        x: 5.,
        y: 7.,
        yaw: 0.,
    };
    assert_eq!(arbiter.step(evidence).status.reason, "route_no_path");
    evidence = input(&map, 0.4, 1);
    evidence.pose = Pose {
        x: 5.,
        y: 7.,
        yaw: 0.,
    };
    let output = arbiter.step(evidence);
    assert!(!output.status.global_route.is_empty());
    assert!(output.status.global_route.iter().any(|p| p[0] < 2.));
}

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
    arbiter.planner.config.radius = 0.2;
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
    let first = arbiter.step(ArbiterInput {
        now: 0.1,
        pose,
        pose_time: 0.1,
        map: Some(&map),
        map_time: Some(0.1),
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
                    arbiter.planner.config.radius,
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
        if tick == 1 {
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
