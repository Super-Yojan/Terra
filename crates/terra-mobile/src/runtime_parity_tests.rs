//! Adapter parity: replay through the actual UniFFI facade and the pure runtime.
use super::*;
use terra_runtime::{Runtime, RuntimeConfig, RuntimeEvent};
#[test]
fn phone_facade_matches_pure_runtime_for_timestamped_sensor_and_manual_stream() {
    let phone = MobileController::new(default_control_settings()).unwrap();
    {
        let mut b = phone.brain.lock().unwrap();
        b.runtime.run_id = "replay".into();
        b.runtime.autonomy.run_id = "replay".into();
    }
    let mut pure = Runtime::new(RuntimeConfig::default(), "replay".into()).unwrap();
    for n in 1..=100 {
        let t = n as f64 * 0.01;
        phone
            .push_imu(ImuReading {
                timestamp: t,
                acceleration_forward: 0.,
                acceleration_left: 0.,
                acceleration_up: 0.,
                gyro_roll: 0.,
                gyro_pitch: 0.,
                gyro_yaw: 0.,
            })
            .unwrap();
        pure.ingest(RuntimeEvent::Imu {
            sample: ImuSample {
                timestamp: t,
                acceleration: Vector3::ZERO,
                angular_velocity: Vector3::ZERO,
            },
            received_at: t,
        })
        .unwrap();
        phone
            .push_vio(VioReading {
                timestamp: t,
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
            })
            .unwrap();
        pure.ingest(RuntimeEvent::Vio {
            sample: VioSample {
                timestamp: t,
                position: Vector3::ZERO,
                orientation: Quaternion::default(),
                velocity: Vector3::ZERO,
                tracked: true,
            },
            received_at: t,
        })
        .unwrap();
        phone
            .set_target(TwistSetpoint {
                timestamp: t,
                forward: 0.2,
                yaw_rate: 0.1,
            })
            .unwrap();
        pure.ingest(RuntimeEvent::LocalVelocity {
            target: VelocityTarget {
                timestamp: t,
                forward: 0.2,
                yaw_rate: 0.1,
            },
            received_at: t,
        })
        .unwrap();
        let a = phone.step(t).unwrap();
        let b = pure.step(t).unwrap();
        assert!((a.left_effort - b.motor.left).abs() <= 1e-6);
        assert!((a.right_effort - b.motor.right).abs() <= 1e-6);
        let actual: serde_json::Value = serde_json::from_str(&phone.autonomy_status()).unwrap();
        assert_eq!(actual["status"], b.snapshot.value["status"]);
        assert_eq!(actual["twist"], b.snapshot.value["twist"]);
    }
}
#[test]
fn phone_direct_effort_uses_shared_mixer_and_needs_no_camera() {
    let phone = MobileController::new(default_control_settings()).unwrap();
    phone.set_actuation_mode(false).unwrap();
    phone.set_manual_input(0.4, 0.1, 0.).unwrap();
    let out = phone.step(0.).unwrap();
    assert!((out.left_effort - 0.3).abs() < 1e-6);
    assert!((out.right_effort - 0.5).abs() < 1e-6);
    let stopped = phone.step(1.).unwrap();
    assert_eq!(stopped.left_effort, 0.);
    assert_eq!(stopped.right_effort, 0.);
}

#[test]
fn actuator_frame_routes_the_tick_efforts_through_the_layout() {
    let phone = MobileController::new(default_control_settings()).unwrap();
    phone.set_actuation_mode(false).unwrap();
    phone.set_manual_input(0.4, 0.1, 0.).unwrap();
    assert_eq!(phone.step(0.).unwrap().actuator_frame, None);
    phone
        .set_actuator_layout(
            r#"{"schema_version":1,"revision":1,"actuators":[
            {"id":0,"kind":"dc_motor","limits":{"min":-1.0,"max":1.0},"route":{"type":"left_effort"},"safe":{"type":"zero"}},
            {"id":1,"kind":"dc_motor","limits":{"min":-1.0,"max":1.0},"route":{"type":"right_effort"},"safe":{"type":"zero"}}]}"#
                .into(),
        )
        .unwrap();
    phone.set_manual_input(0.4, 0.1, 0.01).unwrap();
    let out = phone.step(0.01).unwrap();
    let frame: serde_json::Value =
        serde_json::from_str(out.actuator_frame.as_deref().unwrap()).unwrap();
    assert!((frame[0]["value"].as_f64().unwrap() - out.left_effort).abs() < 1e-6);
    assert!((frame[1]["value"].as_f64().unwrap() - out.right_effort).abs() < 1e-6);
    assert!(phone.set_actuator_layout("{}".into()).is_err());
    assert_eq!(phone.step(0.02).unwrap().actuator_frame, None);
}

#[test]
fn reset_starts_a_new_clock_and_discards_queued_remote_commands() {
    let phone = MobileController::new(default_control_settings()).unwrap();
    phone.set_actuation_mode(false).unwrap();
    phone.set_manual_input(0.4, 0.0, 10.0).unwrap();
    phone.step(10.0).unwrap();
    {
        let mut brain = phone.brain.lock().unwrap();
        brain
            .runtime
            .ingest(terra_runtime::RuntimeEvent::Remote {
                kind: "safety".into(),
                payload: br#"{"action":"stop","token":"old"}"#.to_vec(),
                received_at: 10.0,
            })
            .unwrap();
    }
    phone.reset().unwrap();
    phone.set_manual_input(0.4, 0.0, 0.0).unwrap();
    let out = phone.step(0.0).unwrap();
    assert!(out.left_effort > 0.0);
    assert!(out.right_effort > 0.0);
}

#[test]
fn a_new_recording_does_not_execute_an_old_queued_stop() {
    let phone = MobileController::new(default_control_settings()).unwrap();
    phone.set_actuation_mode(false).unwrap();
    {
        let mut brain = phone.brain.lock().unwrap();
        brain
            .runtime
            .ingest(terra_runtime::RuntimeEvent::Remote {
                kind: "safety".into(),
                payload: br#"{"action":"stop","token":"old-run-stop"}"#.to_vec(),
                received_at: 0.0,
            })
            .unwrap();
    }
    let dir = tempfile::tempdir().unwrap();
    phone
        .begin_recording(
            dir.path().join("run").to_string_lossy().into_owned(),
            "new-run".into(),
        )
        .unwrap();
    phone.set_manual_input(0.4, 0.0, 0.0).unwrap();
    let out = phone.step(0.0).unwrap();
    assert!(out.left_effort > 0.0);
    phone.end_recording().unwrap();
}
