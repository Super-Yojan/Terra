//! The dashboard sees when the rover backs up, and when its wheels push
//! without the rover moving (stalling while backing up, 2026-11-06).
use terra_runtime::*;
use terra_types::*;

fn vio(r: &mut Runtime, t: f64, speed: f64) {
    r.ingest(RuntimeEvent::Imu {
        sample: ImuSample {
            timestamp: t,
            acceleration: Vector3::ZERO,
            angular_velocity: Vector3::ZERO,
        },
        received_at: t,
    })
    .unwrap();
    r.ingest(RuntimeEvent::Vio {
        sample: VioSample {
            timestamp: t,
            position: Vector3::ZERO,
            orientation: Quaternion::default(),
            velocity: Vector3 {
                x: speed,
                y: 0.,
                z: 0.,
            },
            tracked: true,
        },
        received_at: t,
    })
    .unwrap();
}

fn drive(r: &mut Runtime, t: f64, forward: f64) -> serde_json::Value {
    r.ingest(RuntimeEvent::LocalManual {
        forward,
        turn: 0.,
        received_at: t,
    })
    .unwrap();
    r.step(t).unwrap().snapshot.value["drive"].clone()
}

#[test]
fn reports_backing_up_and_how_long_it_has_stalled() {
    let mut r = Runtime::new(
        RuntimeConfig {
            mode: ActuationMode::DirectEffort,
            ..Default::default()
        },
        "run".into(),
    )
    .unwrap();
    vio(&mut r, 0., 0.);
    let d = drive(&mut r, 0., -0.1);
    assert_eq!(d["reversing"], true, "{d}");
    assert!(d["commanded_linear"].as_f64().unwrap() < 0.);
    assert!(d["left_effort"].as_f64().unwrap() < 0.);
    assert_eq!(d["stalled_s"], 0.);

    // Wheels push backward, the rover does not move.
    vio(&mut r, 0.4, 0.);
    let d = drive(&mut r, 0.4, -0.1);
    assert!((d["stalled_s"].as_f64().unwrap() - 0.4).abs() < 1e-9, "{d}");

    // It moves: no longer stalled, still backing up.
    vio(&mut r, 0.5, -0.1);
    let d = drive(&mut r, 0.5, -0.1);
    assert_eq!(d["stalled_s"], 0.);
    assert_eq!(d["reversing"], true);

    // Forward is not reversing.
    let d = drive(&mut r, 0.6, 0.2);
    assert_eq!(d["reversing"], false);
}
