//! The one per-tick entry point: sensors in, a gated motor action out.
use terra_mobile::{
    DriveSource, ImuReading, MobileController, MotorAction, MotorLink, PhoneSensors, SafetyState,
    StickInput, VioReading, default_control_settings,
};

const LAYOUT: &str = r#"{"schema_version":1,"revision":1,"actuators":[
    {"id":0,"name":"left","port":"M0","kind":"dc_motor","inverted":false,
     "limits":{"min":-1.0,"max":1.0},
     "calibration":{"type":"dc_motor","max_power_fraction":1.0},
     "route":{"type":"left_effort"},"safe":{"type":"zero"}},
    {"id":1,"name":"right","port":"M1","kind":"dc_motor","inverted":false,
     "limits":{"min":-1.0,"max":1.0},
     "calibration":{"type":"dc_motor","max_power_fraction":1.0},
     "route":{"type":"right_effort"},"safe":{"type":"zero"}}
]}"#;

fn link(mount_confirmed: bool) -> MotorLink {
    MotorLink {
        connected: true,
        ready: true,
        mount_confirmed,
        tracked: true,
        feedback_compatible: true,
    }
}

fn sensors(now: f64, source: DriveSource, stick: Option<(f64, f64)>, motors: MotorLink) -> PhoneSensors {
    PhoneSensors {
        now,
        imu: Some(ImuReading {
            timestamp: now,
            acceleration_forward: 0.,
            acceleration_left: 0.,
            acceleration_up: 0.,
            gyro_roll: 0.,
            gyro_pitch: 0.,
            gyro_yaw: 0.,
        }),
        vio: Some(VioReading {
            timestamp: now,
            position_x: 0.,
            position_y: 0.,
            position_z: 0.,
            quaternion_x: 0.,
            quaternion_y: 0.,
            quaternion_z: 0.,
            quaternion_w: 1.,
            velocity_x: 0.,
            velocity_y: 0.,
            velocity_z: 0.,
            tracked: true,
        }),
        gps: None,
        heading: None,
        course: None,
        stick: stick.map(|(forward, turn)| StickInput {
            forward,
            turn,
            timestamp: now,
        }),
        motors,
        source,
    }
}

#[test]
fn local_stick_drives_and_expires() {
    let c = MobileController::new(default_control_settings()).unwrap();
    let out = c
        .update(sensors(0., DriveSource::Simulation, Some((1., 0.3)), link(true)))
        .unwrap();
    assert_eq!(out.safety, SafetyState::Active);
    assert!(out.right_effort > out.left_effort, "{out:?}");
    let stale = c
        .update(sensors(1., DriveSource::Simulation, None, link(true)))
        .unwrap();
    assert_eq!((stale.left_effort, stale.right_effort), (0., 0.));
}

#[test]
fn debug_stick_routes_a_frame_without_a_mount() {
    let c = MobileController::new(default_control_settings()).unwrap();
    c.set_actuator_layout(LAYOUT.into()).unwrap();
    c.set_actuation_mode(false).unwrap();
    let frames: Vec<_> = (0..=10)
        .map(|i| {
            let t = 10. + i as f64 * 0.01;
            c.update(sensors(t, DriveSource::BluetoothManual, Some((0.5, 0.)), link(false)))
                .unwrap()
                .action
        })
        .collect();
    let frame = frames
        .iter()
        .find_map(|a| match a {
            MotorAction::Drive { frame_json } => Some(frame_json.clone()),
            _ => None,
        })
        .expect("a drive frame");
    assert!(frame.contains("\"id\":1"), "{frame}");
}

#[test]
fn phone_drive_holds_until_the_mount_is_confirmed() {
    let c = MobileController::new(default_control_settings()).unwrap();
    c.set_actuator_layout(LAYOUT.into()).unwrap();
    let held = c
        .update(sensors(0., DriveSource::Phone, None, link(false)))
        .unwrap()
        .action;
    assert!(matches!(held, MotorAction::Stop { .. }), "{held:?}");
    // Reported once; then idle until confirmed.
    let next = c
        .update(sensors(0.01, DriveSource::Phone, None, link(false)))
        .unwrap()
        .action;
    assert!(matches!(next, MotorAction::Idle), "{next:?}");
}

#[test]
fn emergency_stop_blocks_the_stick() {
    let c = MobileController::new(default_control_settings()).unwrap();
    c.set_actuator_layout(LAYOUT.into()).unwrap();
    c.set_actuation_mode(false).unwrap();
    c.update(sensors(0., DriveSource::BluetoothManual, Some((0., 0.)), link(true)))
        .unwrap();
    c.autonomy_request("safety".into(), r#"{"action":"stop","token":"t"}"#.into(), 0.005)
        .unwrap();
    let out = c
        .update(sensors(0.01, DriveSource::BluetoothManual, Some((1., 0.)), link(true)))
        .unwrap();
    assert!(matches!(out.action, MotorAction::Stop { ref reason } if reason.starts_with("Emergency stop")), "{out:?}");
    assert_eq!((out.left_effort, out.right_effort), (0., 0.));
}
