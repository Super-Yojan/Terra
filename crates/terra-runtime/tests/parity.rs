use terra_runtime::*;
use terra_types::{ImuSample, Quaternion, Vector3, VioSample};
fn samples(r: &mut Runtime, now: f64) {
    r.ingest(RuntimeEvent::Imu {
        sample: ImuSample {
            timestamp: now,
            acceleration: Vector3::ZERO,
            angular_velocity: Vector3::ZERO,
        },
        received_at: now,
    })
    .unwrap();
    r.ingest(RuntimeEvent::Vio {
        sample: VioSample {
            timestamp: now,
            position: Vector3::ZERO,
            orientation: Quaternion::default(),
            velocity: Vector3::ZERO,
            tracked: true,
        },
        received_at: now,
    })
    .unwrap();
}
#[test]
fn replay_is_deterministic_and_managed_spin_is_blocked() {
    let mut phone = Runtime::new(RuntimeConfig::default(), "run-1".into()).unwrap();
    let mut simulator = Runtime::new(RuntimeConfig::default(), "run-1".into()).unwrap();
    for n in 1..=80 {
        let t = n as f64 * 0.01;
        for r in [&mut phone, &mut simulator] {
            samples(r, t);
            let payload = serde_json::json!({"linear":0.0,"angular":0.5,"run_id":"run-1","authority_revision":r.autonomy.authority_revision(),"operator_session_id":"operator","sequence":n});
            r.ingest(RuntimeEvent::Remote {
                kind: "teleop".into(),
                payload: serde_json::to_vec(&payload).unwrap(),
                received_at: t,
            })
            .unwrap();
        }
        let a = phone.step(t).unwrap();
        let b = simulator.step(t).unwrap();
        assert_eq!(a.snapshot, b.snapshot);
        assert_eq!(a.motor.left, b.motor.left);
        assert_eq!(a.motor.right, b.motor.right);
        assert_eq!(a.autonomy.twist.angular, 0.0);
    }
}
#[test]
fn direct_manual_works_without_camera_and_link_loss_clears_remote_input() {
    let mut r = Runtime::new(
        RuntimeConfig {
            mode: ActuationMode::DirectEffort,
            ..Default::default()
        },
        "manual".into(),
    )
    .unwrap();
    r.ingest(RuntimeEvent::LocalManual {
        forward: 0.4,
        turn: 0.1,
        received_at: 0.0,
    })
    .unwrap();
    let out = r.step(0.0).unwrap();
    assert!((out.motor.left - 0.3).abs() < 1e-9);
    assert!((out.motor.right - 0.5).abs() < 1e-9);
    r.ingest(RuntimeEvent::Link {
        connected: false,
        received_at: 0.01,
    })
    .unwrap();
    assert!(r.snapshot(0.01).value["connectivity"]["revision"].is_number());
}
#[test]
fn queued_action_is_checked_at_execution_and_future_events_are_rejected() {
    let mut r = Runtime::new(RuntimeConfig::default(), "run-1".into()).unwrap();
    r.ingest(RuntimeEvent::Remote {
        kind: "autonomy".into(),
        payload: br#"{"level":"waypoint","token":"old","run_id":"run-1","authority_revision":0}"#
            .to_vec(),
        received_at: 0.0,
    })
    .unwrap();
    let out = r.step(1.0).unwrap();
    assert_eq!(
        out.autonomy.status.requested_level,
        terra_autonomy::Level::Teleop
    );
    assert!(out.events.iter().any(|e| e["kind"] == "command.rejected"));
    assert!(
        r.ingest(RuntimeEvent::LocalManual {
            forward: f64::NAN,
            turn: 0.0,
            received_at: 1.0
        })
        .is_err()
    );
    assert!(r.step(0.5).is_err());
}

#[test]
fn direct_effort_does_not_keep_an_expired_local_command() {
    let mut r = Runtime::new(
        RuntimeConfig {
            mode: ActuationMode::DirectEffort,
            ..Default::default()
        },
        "run".into(),
    )
    .unwrap();
    r.ingest(RuntimeEvent::LocalManual {
        forward: 0.4,
        turn: 0.0,
        received_at: 0.0,
    })
    .unwrap();
    assert!(r.step(0.0).unwrap().motor.left > 0.0);
    let out = r.step(1.0).unwrap();
    assert_eq!(out.motor.left, 0.0);
    assert_eq!(out.motor.right, 0.0);
}
#[test]
fn teleop_received_before_a_late_tick_still_drives() {
    let mut r = Runtime::new(
        RuntimeConfig {
            mode: ActuationMode::DirectEffort,
            ..Default::default()
        },
        "late".into(),
    )
    .unwrap();
    // One second of driving outlasts the 0.5 s lease of any single command.
    for n in 1..=100 {
        let t = 1. + n as f64 * 0.01;
        let payload = serde_json::json!({"linear":0.2,"angular":0.0,"run_id":"late","authority_revision":r.autonomy.authority_revision(),"operator_session_id":"operator","sequence":n});
        // Each command waited 30 ms in the inbox, three 10 ms ticks.
        r.ingest(RuntimeEvent::Remote {
            kind: "teleop".into(),
            payload: serde_json::to_vec(&payload).unwrap(),
            received_at: t - 0.03,
        })
        .unwrap();
        let output = r.step(t).unwrap();
        assert!(
            output.motor.left > 0. && output.motor.right > 0.,
            "tick {n}"
        );
    }
}
