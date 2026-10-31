//! Backing up is always allowed: reverse never waits on a fresh map and is
//! never blocked by mapped obstacles. Stop and sensor checks still hold it.
use terra_autonomy::*;
use terra_mapping::MapSnapshot;
use terra_navigation::{Pose, Twist};

fn tele(linear: f64) -> TeleopRequest {
    TeleopRequest {
        linear,
        angular: 0.,
        run_id: None,
        authority_revision: None,
        operator_session_id: None,
        sequence: None,
    }
}

fn assisted() -> AutonomyArbiter {
    let mut a = AutonomyArbiter::default();
    a.set_level(LevelRequest::new(Level::AssistedTeleop, "l1"));
    a
}

fn input<'a>(now: f64, map: Option<&'a MapSnapshot>, map_time: Option<f64>) -> ArbiterInput<'a> {
    ArbiterInput {
        now,
        pose: Pose::default(),
        pose_time: now,
        map,
        map_time,
        map_revision: 1,
        measured: Twist::default(),
        healthy: true,
    }
}

fn walled() -> MapSnapshot {
    MapSnapshot {
        width: 40,
        height: 40,
        resolution: 0.25,
        origin_x: -5.,
        origin_y: -5.,
        occupancy: vec![100; 1600],
    }
}

#[test]
fn assisted_reverse_runs_without_a_map() {
    let mut a = assisted();
    a.accept_operator(tele(-0.1), 0.);
    let out = a.step(input(0.1, None, None));
    assert!(out.twist.linear < 0., "{:?}", out.status.reason);
    assert_ne!(out.status.reason, "map_stale");

    // Forward still needs the map.
    a.accept_operator(tele(0.1), 0.15);
    let out = a.step(input(0.2, None, None));
    assert_eq!(out.status.reason, "map_stale");
    assert_eq!(out.twist, Twist::default());
}

#[test]
fn assisted_reverse_ignores_a_stale_map() {
    let map = walled();
    let mut a = assisted();
    a.accept_operator(tele(-0.1), 10.);
    let out = a.step(input(10.1, Some(&map), Some(0.)));
    assert!(out.twist.linear < 0.);
}

#[test]
fn assisted_reverse_is_not_blocked_by_mapped_obstacles() {
    let map = walled();
    let mut a = assisted();
    a.accept_operator(tele(-0.1), 0.);
    let out = a.step(input(0.1, Some(&map), Some(0.1)));
    assert!(out.twist.linear < 0., "{:?}", out.status.reason);

    a.accept_operator(tele(0.1), 0.15);
    let out = a.step(input(0.2, Some(&map), Some(0.2)));
    assert_eq!(out.status.reason, "obstacle_blocked");
    assert_eq!(out.twist, Twist::default());
}

#[test]
fn reverse_is_capped_and_still_obeys_stop_and_sensors() {
    let mut a = assisted();
    a.accept_operator(tele(-2.), 0.);
    let out = a.step(input(0.1, None, None));
    assert!(out.twist.linear < 0. && out.twist.linear >= -0.5);

    let mut unhealthy = input(0.2, None, None);
    unhealthy.healthy = false;
    a.accept_operator(tele(-0.1), 0.15);
    let out = a.step(unhealthy);
    assert_eq!(out.status.reason, "sensor_unhealthy");
    assert_eq!(out.twist, Twist::default());

    a.set_safety(
        SafetyRequest {
            action: SafetyAction::Stop,
            token: "stop".into(),
        },
        true,
    );
    let out = a.step(input(0.3, None, None));
    assert_eq!(out.status.safety, "emergency_stop");
    assert_eq!(out.twist, Twist::default());
}

#[test]
fn autonomous_levels_still_hold_on_a_stale_map() {
    let mut a = AutonomyArbiter::default();
    a.set_level(LevelRequest::new(Level::Waypoint, "l3"));
    assert_eq!(a.step(input(0.1, None, None)).status.reason, "map_stale");
}
