//! The `dock` topic: object lookup, authority, and the gate servo mapping.
use terra_autonomy::objects::ObjectSighting;
use terra_runtime::*;
use terra_types::{ImuSample, Quaternion, Vector3, VioSample};

fn pose_at(r: &mut Runtime, now: f64) {
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

fn sight(r: &mut Runtime, frame: u64, color: &str, now: f64) {
    r.observe_object(
        ObjectSighting {
            class: "bucket".into(),
            color: Some(color.into()),
            confidence: 0.8,
            frame_id: frame,
            timestamp: now,
            x: 1.65,
            y: 0.,
            geo: None,
        },
        now,
    )
    .unwrap();
}

#[test]
fn dock_starts_only_on_a_confirmed_bucket() {
    let mut r = Runtime::new(RuntimeConfig::default(), "run-1".into()).unwrap();
    pose_at(&mut r, 0.1);
    sight(&mut r, 1, "orange", 0.1);
    let id = r.objects.objects()[0].id;
    let start = format!(r#"{{"action":"start","object_id":{id},"token":"d1"}}"#);
    // One sighting is not a confirmed object yet.
    assert!(r.local_action("dock", start.as_bytes(), 0.1).is_err());
    sight(&mut r, 2, "orange", 0.2);
    sight(&mut r, 3, "orange", 0.3);
    let start = format!(r#"{{"action":"start","object_id":{id},"token":"d2"}}"#);
    r.local_action("dock", start.as_bytes(), 0.3).unwrap();
    assert!(r.autonomy.dock_active());
    let status = r.autonomy.dock_status(0.3).unwrap();
    assert_eq!(status.color.as_deref(), Some("orange"));
    assert!(
        r.local_action("dock", br#"{"action":"cancel","token":"d3"}"#, 0.4)
            .is_ok()
    );
    assert!(!r.autonomy.dock_active());
}

#[test]
fn remote_dock_needs_current_authority_but_cancel_does_not() {
    let r = Runtime::new(RuntimeConfig::default(), "run-1".into()).unwrap();
    assert_eq!(
        r.remote_action(
            "dock",
            br#"{"action":"start","object_id":1,"token":"a"}"#,
            0.
        ),
        Err("current_authority_required")
    );
    assert!(
        r.remote_action("dock", br#"{"action":"cancel","token":"b"}"#, 9.)
            .is_ok()
    );
}

#[test]
fn gate_servo_follows_the_profile() {
    let mut r = Runtime::new(RuntimeConfig::default(), "run-1".into()).unwrap();
    assert_eq!(r.gate_servo(true), None);
    let mut profile = rover_profile::RoverProfile::terra();
    profile.dock.gate_servo = Some(8);
    profile.dock.gate_open = 0.6;
    profile.dock.gate_closed = -0.4;
    profile.dock.outlet_ahead = Some(0.7);
    r.apply_rover_profile(profile).unwrap();
    assert_eq!(r.gate_servo(true), Some((8, 0.6)));
    assert_eq!(r.gate_servo(false), Some((8, -0.4)));
    assert_eq!(r.autonomy.dock_config().outlet_ahead, 0.7);
    // A new run keeps the profile's dock geometry.
    r.begin_run("run-2".into()).unwrap();
    assert_eq!(r.autonomy.dock_config().outlet_ahead, 0.7);
}
