use terra_autonomy::*;
use terra_navigation::{Pose, Twist};
fn tick(a: &mut AutonomyArbiter, t: f64) -> ArbiterOutput {
    a.step(ArbiterInput {
        now: t,
        pose: Pose::default(),
        pose_time: t,
        map: None,
        map_time: None,
        map_revision: 0,
        measured: Twist::default(),
        healthy: true,
    })
}
#[test]
fn fleet_is_limited_local_manual_is_unchanged() {
    let mut a = AutonomyArbiter::default();
    a.accept_operator_from(
        decode_teleop(br#"{"linear":0,"angular":1}"#).unwrap(),
        0.,
        CommandOrigin::FleetTeleop,
    );
    assert_eq!(tick(&mut a, 0.01).twist, Twist::default());
    a.accept_operator(decode_teleop(br#"{"linear":0,"angular":1}"#).unwrap(), 0.02);
    assert_eq!(tick(&mut a, 0.03).twist.angular, 1.);
    a.accept_operator_from(
        decode_teleop(br#"{"linear":0.3,"angular":1}"#).unwrap(),
        0.04,
        CommandOrigin::FleetTeleop,
    );
    let out = tick(&mut a, 0.05);
    assert!(out.twist.angular <= 0.00500001);
    assert_eq!(out.command_origin, CommandOrigin::FleetTeleop);
}
#[test]
fn forged_exemption_rejected_and_stop_immediate() {
    assert!(
        decode_teleop(br#"{"linear":1,"angular":1,"command_origin":"local_manual"}"#).is_none()
    );
    let mut a = AutonomyArbiter::default();
    a.accept_operator_from(
        decode_teleop(br#"{"linear":0.3,"angular":1}"#).unwrap(),
        0.,
        CommandOrigin::FleetTeleop,
    );
    tick(&mut a, 0.01);
    a.hold_emergency_stop();
    assert_eq!(tick(&mut a, 0.02).twist, Twist::default());
}
#[test]
fn heading_uses_checked_multipoint_segments_and_measured_yaw() {
    let mut a = AutonomyArbiter::default();
    a.set_level(decode_level(br#"{"level":"waypoint_direct","token":"heading-mode"}"#).unwrap());
    assert!(a.accept_goal(
        &terra_waypoint::GoalCommand::Local {
            x: 0.,
            y: 0.,
            yaw: Some(3.0),
            token: Some("heading".into())
        },
        100.
    ));
    let map = terra_mapping::MapSnapshot {
        width: 80,
        height: 80,
        resolution: 0.25,
        origin_x: -10.,
        origin_y: -10.,
        occupancy: vec![0; 6400],
    };
    let out = a.step(ArbiterInput {
        now: 0.,
        pose: Pose::default(),
        pose_time: 0.,
        map: Some(&map),
        map_time: Some(0.),
        map_revision: 0,
        measured: Twist::default(),
        healthy: true,
    });
    assert_ne!(out.twist, Twist::default());
    let trace = out.status.planner_trace.unwrap();
    assert!(
        trace
            .maneuver_segments
            .iter()
            .any(|s| s.direction == "reverse")
    );
    assert_eq!(
        trace.candidates[trace.selected.unwrap()].velocity,
        out.twist
    );
    let out = a.step(ArbiterInput {
        now: 0.05,
        pose: Pose {
            yaw: 3.,
            ..Default::default()
        },
        pose_time: 0.05,
        map: Some(&map),
        map_time: Some(0.05),
        map_revision: 0,
        measured: Twist::default(),
        healthy: true,
    });
    assert_eq!(out.goal.state, terra_waypoint::GoalState::Arrived);
    assert_eq!(out.twist, Twist::default());
}
#[test]
fn three_point_turn_completes_from_measured_motion_not_elapsed_time() {
    let mut a = AutonomyArbiter::default();
    a.set_level(decode_level(br#"{"level":"waypoint_direct","token":"turn-mode"}"#).unwrap());
    a.accept_goal(
        &terra_waypoint::GoalCommand::Local {
            x: 0.,
            y: 0.,
            yaw: Some(3.),
            token: Some("turn".into()),
        },
        100.,
    );
    let map = terra_mapping::MapSnapshot {
        width: 80,
        height: 80,
        resolution: 0.25,
        origin_x: -10.,
        origin_y: -10.,
        occupancy: vec![0; 6400],
    };
    let mut pose = Pose::default();
    let mut measured = Twist::default();
    let mut reverse = false;
    let mut forward = false;
    for n in 0..2400 {
        let now = n as f64 * 0.05;
        let out = a.step(ArbiterInput {
            now,
            pose,
            pose_time: now,
            map: Some(&map),
            map_time: Some(now),
            map_revision: 1,
            measured,
            healthy: true,
        });
        if out.goal.state == terra_waypoint::GoalState::Arrived {
            assert!(reverse && forward);
            return;
        }
        reverse |= out.twist.linear < 0.;
        forward |= out.twist.linear > 0.;
        if let Some(trace) = out.status.planner_trace {
            if let Some(i) = trace.selected {
                assert_eq!(trace.candidates[i].velocity, out.twist);
            }
        }
        measured = out.twist;
        pose.yaw += measured.angular * 0.05;
        pose.x += measured.linear * pose.yaw.cos() * 0.05;
        pose.y += measured.linear * pose.yaw.sin() * 0.05;
    }
    panic!("did not complete from measured pose: {pose:?}");
}
#[test]
fn local_and_wgs84_waypoints_share_turning_policy() {
    let mut local = AutonomyArbiter::default();
    let mut geo = AutonomyArbiter::default();
    for a in [&mut local, &mut geo] {
        a.set_level(decode_level(br#"{"level":"waypoint_direct","token":"geo-mode"}"#).unwrap());
        a.set_origin(38.8, -77.3);
    }
    let origin = terra_waypoint::GeoOrigin::new(38.8, -77.3).unwrap();
    let (lat, lon) = origin.from_local(1., 0.2);
    local.accept_goal(
        &terra_waypoint::GoalCommand::Local {
            x: 1.,
            y: 0.2,
            yaw: None,
            token: None,
        },
        100.,
    );
    geo.accept_goal(
        &terra_waypoint::GoalCommand::Wgs84 {
            latitude: lat,
            longitude: lon,
            yaw: None,
            token: None,
        },
        100.,
    );
    let l = tick(&mut local, 0.);
    let g = tick(&mut geo, 0.);
    assert!((l.twist.angular - g.twist.angular).abs() < 1e-6);
    assert!((l.twist.linear - g.twist.linear).abs() < 1e-6);
    assert_eq!(g.goal.latitude, Some(lat));
}
