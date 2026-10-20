use terra_mobile::*;
#[test]
fn degraded_pose_is_explicit_and_blocks_local_and_geographic_goal_admission() {
    let c = MobileController::new(default_control_settings()).unwrap();
    c.step(1.).unwrap();
    let status: serde_json::Value = serde_json::from_str(&c.autonomy_status()).unwrap();
    assert_eq!(status["pose_quality"]["degraded"], true);
    assert_eq!(
        status["pose_quality"]["pose_age_s"],
        serde_json::Value::Null
    );
    for goal in [
        r#"{"frame":"local","x":1,"y":0}"#,
        r#"{"frame":"wgs84","latitude":38.8,"longitude":-77.3}"#,
    ] {
        assert!(c.autonomy_request("goal".into(), goal.into(), 1.).is_err());
    }
    assert!(
        c.autonomy_request("goal".into(), r#"{"cancel":true}"#.into(), 1.)
            .is_ok()
    );
}

#[test]
fn active_goal_stops_at_central_pose_freshness_threshold() {
    let mut settings = default_control_settings();
    settings.vio_timeout = 1.;
    let c = MobileController::new(settings).unwrap();
    let imu = |t| ImuReading {
        timestamp: t,
        acceleration_forward: 0.,
        acceleration_left: 0.,
        acceleration_up: 0.,
        gyro_roll: 0.,
        gyro_pitch: 0.,
        gyro_yaw: 0.,
    };
    c.push_imu(imu(0.)).unwrap();
    c.push_vio(VioReading {
        timestamp: 0.,
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
    c.autonomy_request(
        "autonomy".into(),
        r#"{"level":"waypoint_direct","token":"a"}"#.into(),
        0.,
    )
    .unwrap();
    c.autonomy_request(
        "goal".into(),
        r#"{"frame":"local","x":5,"y":0,"token":"b"}"#.into(),
        0.,
    )
    .unwrap();
    c.step(0.).unwrap();
    for i in 1..=8 {
        let t = i as f64 * 0.05;
        c.push_imu(imu(t)).unwrap();
        c.step(t).unwrap();
    }
    let state: serde_json::Value = serde_json::from_str(&c.autonomy_status()).unwrap();
    assert_eq!(state["pose_quality"]["degraded"], true);
    assert_eq!(state["twist"]["linear"], 0.);
    assert_eq!(state["status"]["safety"], "hold");
}

#[test]
fn manual_authority_remains_available_without_camera_or_imu() {
    let c = MobileController::new(default_control_settings()).unwrap();
    c.set_target(TwistSetpoint { timestamp: 1., forward: 0.2, yaw_rate: 0. }).unwrap();
    c.step(1.01).unwrap();
    let status: serde_json::Value = serde_json::from_str(&c.autonomy_status()).unwrap();
    assert_eq!(status["pose_quality"]["degraded"], true);
    assert_eq!(status["status"]["safety"], "clear");
    assert_eq!(status["status"]["effective_level"], "teleop");
    assert_eq!(status["twist"]["linear"], 0.2);
    c.step(2.).unwrap();
    let expired: serde_json::Value = serde_json::from_str(&c.autonomy_status()).unwrap();
    assert_eq!(expired["twist"]["linear"], 0.);
}

#[test]
fn manual_emergency_stop_requires_explicit_reset_but_not_camera_tracking() {
    let c = MobileController::new(default_control_settings()).unwrap();
    c.set_target(TwistSetpoint { timestamp: 1., forward: 0.2, yaw_rate: 0. }).unwrap();
    c.autonomy_request("safety".into(), r#"{"action":"stop","token":"stop"}"#.into(), 1.01).unwrap();
    c.step(1.02).unwrap();
    let stopped: serde_json::Value = serde_json::from_str(&c.autonomy_status()).unwrap();
    assert_eq!(stopped["status"]["safety"], "emergency_stop");
    assert_eq!(stopped["twist"]["linear"], 0.);
    c.autonomy_request("safety".into(), r#"{"action":"reset","token":"reset"}"#.into(), 1.03).unwrap();
    c.step(1.04).unwrap();
    let reset: serde_json::Value = serde_json::from_str(&c.autonomy_status()).unwrap();
    assert_eq!(reset["status"]["safety"], "clear");
    assert_eq!(reset["twist"]["linear"], 0.);
}
