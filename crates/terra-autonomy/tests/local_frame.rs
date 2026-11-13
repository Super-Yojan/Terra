//! A new local frame ends local-frame missions; a level change keeps the WGS84 origin.
use terra_autonomy::*;
use terra_navigation::{Pose, Twist};
use terra_waypoint::{GoalCommand, GoalState};

fn input(now: f64) -> ArbiterInput<'static> {
    ArbiterInput {
        now,
        pose: Pose::default(),
        pose_time: now,
        map: None,
        map_time: None,
        map_revision: 0,
        measured: Twist::default(),
        healthy: true,
    }
}

fn level(a: &mut AutonomyArbiter, level: Level, token: &str) {
    a.set_level(LevelRequest::new(level, token));
}

fn geo(token: &str) -> GoalCommand {
    GoalCommand::Wgs84 {
        latitude: 38.8298,
        longitude: -77.3075,
        yaw: None,
        token: Some(token.into()),
    }
}

#[test]
fn a_level_change_keeps_the_geographic_origin() {
    let mut a = AutonomyArbiter::default();
    assert!(a.set_origin(38.8297, -77.3075));
    level(&mut a, Level::Waypoint, "l1");
    level(&mut a, Level::WaypointDirect, "l2");
    assert!(a.accept_goal(&geo("g1")));
}

#[test]
fn a_local_frame_reset_ends_the_goal_and_drops_the_origin() {
    let mut a = AutonomyArbiter::default();
    assert!(a.set_origin(38.8297, -77.3075));
    level(&mut a, Level::WaypointDirect, "l1");
    assert!(a.accept_goal(&GoalCommand::Local {
        x: 3.,
        y: 0.,
        yaw: None,
        token: Some("g1".into()),
    }));
    a.reset_local_frame();
    let out = a.step(input(0.1));
    assert_eq!(out.goal.state, GoalState::Idle);
    assert!(out.events.iter().any(|e| e.kind == "local_frame_reset"));
    assert!(!a.accept_goal(&geo("g2")));
    assert!(a.set_origin(38.8297, -77.3075));
    assert!(a.accept_goal(&geo("g3")));
}
