use terra_autonomy::*;
use terra_mapping::MapSnapshot;
use terra_navigation::{Pose, Twist};
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
