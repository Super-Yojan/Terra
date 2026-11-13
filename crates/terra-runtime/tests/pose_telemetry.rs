//! The `pose` topic comes with the rest of a tick's telemetry, and only while
//! the pose is ready.
use terra_runtime::*;
use terra_types::*;

fn sample(r: &mut Runtime, t: f64) {
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
            position: Vector3 {
                x: 1.5,
                y: -0.5,
                z: 0.,
            },
            orientation: Quaternion::default(),
            velocity: Vector3::ZERO,
            tracked: true,
        },
        received_at: t,
    })
    .unwrap();
}

fn pose_topic(output: &RuntimeOutput) -> Option<serde_json::Value> {
    output
        .telemetry()
        .into_iter()
        .find(|(topic, _)| *topic == telemetry::POSE)
        .map(|(_, body)| serde_json::from_str(&body).unwrap())
}

#[test]
fn pose_is_published_only_while_ready() {
    let mut r = Runtime::new(RuntimeConfig::default(), "run".into()).unwrap();
    r.set_rover_id(Some(7));
    assert_eq!(pose_topic(&r.step(0.).unwrap()), None);

    let mut t = 0.1;
    while t < 1.0 {
        sample(&mut r, t);
        t += 0.05;
    }
    let output = r.step(t).unwrap();
    let pose = pose_topic(&output).expect("ready pose published");
    assert_eq!(pose["rover_id"], 7);
    assert_eq!(pose["sequence"], r.sequence());
    assert_eq!(
        (pose["x"].as_f64(), pose["y"].as_f64()),
        (Some(1.5), Some(-0.5))
    );

    // Stale once samples stop.
    assert_eq!(pose_topic(&r.step(t + 1.).unwrap()), None);
}
