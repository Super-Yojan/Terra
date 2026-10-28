//! Bucket docking through the arbiter: approval by level, takeover and stops.
use terra_autonomy::*;
use terra_mapping::MapSnapshot;
use terra_navigation::{Pose, Twist};

fn map() -> MapSnapshot {
    MapSnapshot {
        width: 80,
        height: 80,
        resolution: 0.1,
        origin_x: -4.,
        origin_y: -4.,
        occupancy: vec![0; 6400],
    }
}

fn bucket() -> DockTarget {
    DockTarget {
        object_id: Some(1),
        color: Some("orange".into()),
        x: 1.8,
        y: 0.,
    }
}

fn start(a: &mut AutonomyArbiter, token: &str) -> bool {
    let r =
        decode_dock(format!(r#"{{"action":"start","object_id":1,"token":"{token}"}}"#).as_bytes())
            .unwrap();
    a.apply_dock_request(r, Some(bucket()), Pose::default(), 0.)
}

/// Drive a perfect unicycle under the arbiter until `until` holds or time runs out.
fn run(
    a: &mut AutonomyArbiter,
    pose: &mut Pose,
    t: &mut f64,
    seconds: f64,
    until: impl Fn(&ArbiterOutput) -> bool,
) -> (ArbiterOutput, bool) {
    let m = map();
    let mut twist = Twist::default();
    let mut gate = false;
    let end = *t + seconds;
    loop {
        *t += 0.01;
        if ((*t * 100.).round() as u64).is_multiple_of(20) {
            // The camera hits the near wall of a bucket centred at (1.8, 0).
            let d = (1.8 - pose.x).hypot(-pose.y);
            let k = (d - 0.1524) / d;
            a.observe_dock(
                "bucket",
                Some("orange"),
                pose.x + (1.8 - pose.x) * k,
                pose.y - pose.y * k,
                *pose,
                *t,
            );
        }
        let out = a.step(ArbiterInput {
            now: *t,
            pose: *pose,
            pose_time: *t,
            map: Some(&m),
            map_time: Some(*t),
            map_revision: 1,
            measured: twist,
            healthy: true,
        });
        gate |= out.gate_open;
        twist = out.twist;
        pose.yaw += twist.angular * 0.01;
        pose.x += twist.linear * pose.yaw.cos() * 0.01;
        pose.y += twist.linear * pose.yaw.sin() * 0.01;
        if until(&out) || *t > end {
            return (out, gate);
        }
    }
}

fn phase(o: &ArbiterOutput) -> Option<DockPhase> {
    o.status.dock.as_ref().map(|d| d.phase)
}

#[test]
fn teleop_dock_waits_for_operator_then_dumps() {
    let mut a = AutonomyArbiter::default();
    assert!(start(&mut a, "d1"));
    let (mut pose, mut t) = (Pose::default(), 0.);
    let (out, gate) = run(&mut a, &mut pose, &mut t, 40., |o| {
        phase(o) == Some(DockPhase::Aligned)
    });
    assert_eq!(
        phase(&out),
        Some(DockPhase::Aligned),
        "{:?}",
        out.status.dock
    );
    assert!(!gate);
    let dock = out.status.dock.unwrap();
    assert!(dock.awaiting_approval);
    assert_eq!(dock.approval, DumpApproval::Operator);
    assert_eq!(out.status.active_source, "dock");
    // A dump for another dock id is refused.
    let wrong = decode_dock(br#"{"action":"dump","dock_id":9,"token":"x1"}"#).unwrap();
    assert!(!a.apply_dock_request(wrong, None, pose, t));
    let ok = decode_dock(
        format!(
            r#"{{"action":"dump","dock_id":{},"token":"x2"}}"#,
            dock.dock_id
        )
        .as_bytes(),
    )
    .unwrap();
    assert!(a.apply_dock_request(ok, None, pose, t));
    let (out, gate) = run(&mut a, &mut pose, &mut t, 5., |o| {
        phase(o) == Some(DockPhase::Done)
    });
    assert_eq!(phase(&out), Some(DockPhase::Done));
    assert!(gate);
    assert!(!out.gate_open);
    assert!(out.events.iter().any(|e| e.kind == "dock_done") || gate);
}

#[test]
fn explore_dock_dumps_on_its_own() {
    let mut a = AutonomyArbiter::default();
    a.set_level(LevelRequest {
        level: Level::Explore,
        token: "l4".into(),
        budget_seconds: Some(300.),
        budget_minutes: None,
    });
    assert_eq!(a.dump_approval(), DumpApproval::Auto);
    assert!(start(&mut a, "d2"));
    let (mut pose, mut t) = (Pose::default(), 0.);
    let (out, gate) = run(&mut a, &mut pose, &mut t, 60., |o| {
        phase(o) == Some(DockPhase::Done)
    });
    assert_eq!(phase(&out), Some(DockPhase::Done), "{:?}", out.status.dock);
    assert!(gate);
}

#[test]
fn operator_driving_cancels_the_dock() {
    let mut a = AutonomyArbiter::default();
    assert!(start(&mut a, "d3"));
    let (mut pose, mut t) = (Pose::default(), 0.);
    run(&mut a, &mut pose, &mut t, 1., |_| false);
    a.accept_operator(
        decode_teleop(br#"{"linear":0.2,"angular":0.0}"#).unwrap(),
        t,
    );
    let (out, _) = run(&mut a, &mut pose, &mut t, 0.05, |_| false);
    assert_eq!(phase(&out), Some(DockPhase::Cancelled));
    assert_eq!(out.status.dock.unwrap().reason, "operator_takeover");
}

#[test]
fn emergency_stop_cancels_and_refuses_new_docks() {
    let mut a = AutonomyArbiter::default();
    assert!(start(&mut a, "d4"));
    a.set_safety(
        decode_safety(br#"{"action":"stop","token":"s1"}"#).unwrap(),
        true,
    );
    assert!(!a.dock_active());
    assert!(!start(&mut a, "d5"));
    assert_eq!(a.request_receipt(), Some(("d5", false)));
}

#[test]
fn unknown_bucket_is_rejected() {
    let mut a = AutonomyArbiter::default();
    let r = decode_dock(br#"{"action":"start","object_id":7,"token":"d6"}"#).unwrap();
    assert!(!a.apply_dock_request(r, None, Pose::default(), 0.));
    assert!(a.dock_status(0.).is_none());
}
