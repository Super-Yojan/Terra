use terra_autonomy::*;
use terra_exploration::Detection;
use terra_mapping::MapSnapshot;
use terra_navigation::{Pose, Twist};
fn input_at(now: f64, pose: Pose, map: &MapSnapshot) -> ArbiterInput<'_> {
    ArbiterInput {
        now,
        pose,
        pose_time: now,
        map: Some(map),
        map_time: Some(now),
        map_revision: 1,
        measured: Twist::default(),
        healthy: true,
    }
}
fn open_map() -> MapSnapshot {
    let mut map = MapSnapshot {
        width: 80,
        height: 80,
        resolution: 0.25,
        origin_x: -10.,
        origin_y: -10.,
        occupancy: vec![-1; 6400],
    };
    for y in 0..80 {
        for x in 0..80 {
            let px = -10. + (x as f64 + 0.5) * 0.25;
            let py = -10. + (y as f64 + 0.5) * 0.25;
            if px.hypot(py) <= 3.0 {
                map.occupancy[y * 80 + x] = 0;
            }
        }
    }
    map
}
fn reveal(map: &mut MapSnapshot, pose: Pose) {
    for y in 0..map.height {
        for x in 0..map.width {
            let px = map.origin_x + (x as f64 + 0.5) * map.resolution;
            let py = map.origin_y + (y as f64 + 0.5) * map.resolution;
            let index = y as usize * map.width as usize + x as usize;
            if (px - pose.x).hypot(py - pose.y) <= 2.2 && map.occupancy[index] < 0 {
                map.occupancy[index] = 0;
            }
        }
    }
}
fn explore(token: &str, seconds: f64) -> LevelRequest {
    LevelRequest::new(Level::Explore, token).with_budget_seconds(seconds)
}
#[test]
fn explore_level_requires_a_budget_and_does_not_wait_for_approval() {
    let mut arbiter = AutonomyArbiter::default();
    arbiter.set_level(LevelRequest::new(Level::Explore, "missing"));
    let rejected = arbiter.step(input_at(0.1, Pose::default(), &open_map()));
    assert_eq!(rejected.status.result.as_deref(), Some("rejected"));
    assert_eq!(
        rejected.status.request_reason.as_deref(),
        Some("budget_required")
    );
    assert_eq!(rejected.status.requested_level, Level::Teleop);
    assert!(rejected.exploration.is_none());
    arbiter.set_exploration_budget(20.);
    arbiter.set_level(LevelRequest::new(Level::Explore, "configured"));
    let started = arbiter.step(input_at(0.2, Pose::default(), &open_map()));
    assert_eq!(started.status.requested_level, Level::Explore);
    assert!(started.proposal.is_none());
    assert_eq!(
        started.exploration.as_ref().map(|status| status.phase),
        Some(ExplorePhase::Running)
    );
    assert!(started.goal.state == terra_waypoint::GoalState::Active);
    assert!(
        started
            .events
            .iter()
            .any(|event| event.kind == "exploration_started")
    );
    assert!(started.status.supported_levels.contains(&Level::Explore));
}
#[test]
fn operator_stop_and_takeover_end_exploration() {
    let map = open_map();
    let mut arbiter = AutonomyArbiter::default();
    arbiter.set_level(explore("go", 30.));
    let _ = arbiter.step(input_at(0.1, Pose::default(), &map));
    arbiter.set_safety(
        SafetyRequest {
            action: SafetyAction::Stop,
            token: "stop-explore".into(),
        },
        true,
    );
    let stopped = arbiter.step(input_at(0.2, Pose::default(), &map));
    assert_eq!(stopped.status.safety, "emergency_stop");
    assert_eq!(stopped.twist, Twist::default());
    assert_eq!(
        stopped
            .exploration
            .as_ref()
            .and_then(|status| status.end_reason),
        Some(EndReason::OperatorStop)
    );
    arbiter.set_safety(
        SafetyRequest {
            action: SafetyAction::Reset,
            token: "reset-explore".into(),
        },
        true,
    );
    let reset = arbiter.step(input_at(0.3, Pose::default(), &map));
    assert_eq!(reset.status.safety, "hold");
    assert_eq!(reset.status.reason, "operator_stop");
    assert_eq!(
        reset
            .exploration
            .as_ref()
            .and_then(|status| status.end_reason),
        Some(EndReason::OperatorStop)
    );
    assert_eq!(reset.twist, Twist::default());
    arbiter.set_level(explore("again", 30.));
    let _ = arbiter.step(input_at(0.4, Pose::default(), &map));
    arbiter.set_level(LevelRequest::new(Level::Teleop, "take-over"));
    arbiter.accept_operator(
        TeleopRequest {
            linear: 0.4,
            angular: 0.,
            run_id: None,
            authority_revision: None,
            operator_session_id: None,
            sequence: None,
        },
        0.5,
    );
    let taken = arbiter.step(input_at(0.5, Pose::default(), &map));
    assert_eq!(taken.status.requested_level, Level::Teleop);
    assert!((taken.twist.linear - 0.4).abs() < 1e-9);
    assert_eq!(
        taken
            .exploration
            .as_ref()
            .and_then(|status| status.end_reason),
        Some(EndReason::OperatorTakeover)
    );
    assert!(
        taken
            .events
            .iter()
            .any(|event| event.kind == "exploration_ended")
    );
}
#[test]
fn no_frontiers_and_map_stale_stop_cleanly() {
    let known = MapSnapshot {
        width: 40,
        height: 40,
        resolution: 0.25,
        origin_x: -5.,
        origin_y: -5.,
        occupancy: vec![0; 1600],
    };
    let mut arbiter = AutonomyArbiter::default();
    arbiter.set_level(explore("full", 30.));
    let done = arbiter.step(input_at(0.1, Pose::default(), &known));
    assert_eq!(done.twist, Twist::default());
    assert_eq!(
        done.exploration
            .as_ref()
            .and_then(|status| status.end_reason),
        Some(EndReason::NoFrontiersLeft)
    );
    assert!(done.proposal.is_none());
    let map = open_map();
    arbiter.set_level(explore("stale", 30.));
    let _ = arbiter.step(input_at(0.2, Pose::default(), &map));
    let stale = arbiter.step(ArbiterInput {
        now: 0.3,
        pose: Pose::default(),
        pose_time: 0.3,
        map: None,
        map_time: None,
        map_revision: 0,
        measured: Twist::default(),
        healthy: true,
    });
    assert_eq!(stale.status.reason, "map_stale");
    assert_eq!(stale.twist, Twist::default());
    assert_eq!(
        stale
            .exploration
            .as_ref()
            .and_then(|status| status.end_reason),
        Some(EndReason::MapStale)
    );
}
#[test]
fn goal_cancel_and_detection_stub_stop_exploration() {
    let map = open_map();
    let mut arbiter = AutonomyArbiter::default();
    arbiter
        .explorer
        .set_objective(Box::new(FindTargetsObjective::new(["survivor"])));
    arbiter.set_level(explore("find", 60.));
    let _ = arbiter.step(input_at(0.1, Pose::default(), &map));
    arbiter.notify_detection(Detection {
        label: "survivor".into(),
        x: 1.,
        y: 1.,
        confidence: 1.,
        observed_at: 0.2,
    });
    let found = arbiter.step(input_at(0.2, Pose::default(), &map));
    assert_eq!(
        found
            .exploration
            .as_ref()
            .and_then(|status| status.end_reason),
        Some(EndReason::ObjectiveComplete)
    );
    arbiter.set_level(explore("cancel", 30.));
    let _ = arbiter.step(input_at(0.3, Pose::default(), &map));
    assert!(arbiter.accept_goal(&terra_waypoint::GoalCommand::Cancel, 50.));
    let cancelled = arbiter.step(input_at(0.4, Pose::default(), &map));
    assert_eq!(
        cancelled
            .exploration
            .as_ref()
            .and_then(|status| status.end_reason),
        Some(EndReason::OperatorStop)
    );
    assert_eq!(cancelled.status.requested_level, Level::Explore);
}
#[test]
fn simulated_rover_explores_until_the_budget_expires() {
    let mut arbiter = AutonomyArbiter::default();
    arbiter.set_level(explore("budget", 3.));
    let mut map = open_map();
    let mut pose = Pose::default();
    let mut measured = Twist::default();
    let mut first_coverage = None;
    let mut coverage = 0.0_f64;
    let mut moved = false;
    let mut ended = None;
    for step in 0..40 {
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
        assert!(output.proposal.is_none());
        if output.twist.linear.abs() > 0.05 || output.twist.angular.abs() > 0.05 {
            moved = true;
        }
        let dt = 0.1;
        pose.yaw += output.twist.angular * dt;
        pose.x += output.twist.linear * pose.yaw.cos() * dt;
        pose.y += output.twist.linear * pose.yaw.sin() * dt;
        measured = output.twist;
        reveal(&mut map, pose);
        if let Some(status) = &output.exploration {
            first_coverage.get_or_insert(status.coverage_m2);
            coverage = coverage.max(status.coverage_m2);
            if status.phase == ExplorePhase::Ended {
                ended = Some((now, status.end_reason, output.twist));
                assert!(
                    output
                        .events
                        .iter()
                        .any(|event| event.kind == "exploration_ended")
                );
                break;
            }
        }
    }
    let (when, reason, twist) = ended.expect("exploration did not stop");
    assert_eq!(reason, Some(EndReason::BudgetExpired));
    assert!((when - 3.).abs() < 1e-9);
    assert_eq!(twist, Twist::default());
    let initial = first_coverage.expect("missing coverage");
    assert!(moved, "rover never left the start");
    assert!(
        coverage > initial + 0.05,
        "coverage {coverage} did not grow past {initial}; pose {pose:?}"
    );
    assert!(pose.x.hypot(pose.y) > 0.2, "pose {pose:?} did not move");
}
