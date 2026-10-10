//! Characterize the existing router API used by sensor-free Bluetooth manual mode.
use std::time::{Duration, Instant};
use terra_mobile::{MobileController, default_control_settings};
use zenoh::Wait;

#[test]
fn sensor_free_dashboard_publishes_status_without_pose_or_motor_output() {
    let socket = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let endpoint = format!("tcp/127.0.0.1:{}", socket.local_addr().unwrap().port());
    drop(socket);
    let mut config = zenoh::Config::default();
    config.insert_json5("mode", "\"router\"").unwrap();
    config
        .insert_json5(
            "listen/endpoints",
            &serde_json::json!([endpoint]).to_string(),
        )
        .unwrap();
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .unwrap();
    let router = zenoh::open(config).wait().unwrap();
    let subscriber = router
        .declare_subscriber("manual-check/7/**")
        .wait()
        .unwrap();
    let controller = MobileController::new(default_control_settings()).unwrap();
    controller
        .connect_dashboard(endpoint, "manual-check".into(), 7)
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(3);
    let mut status_seen = false;
    let mut time = 1.0;
    while !status_seen {
        time += 0.1;
        let output = controller.step(time).unwrap();
        assert_eq!((output.left_effort, output.right_effort), (0.0, 0.0));
        while let Some(sample) = subscriber.try_recv().unwrap() {
            let topic = sample.key_expr().as_str();
            assert!(
                !topic.ends_with("/pose"),
                "missing tracking must not publish a fake pose"
            );
            status_seen |= topic.ends_with("/autonomy/status");
        }
        assert!(
            Instant::now() < deadline,
            "manual dashboard telemetry did not arrive"
        );
        std::thread::sleep(Duration::from_millis(10));
    }
    router
        .put(
            "manual-check/7/safety",
            r#"{"action":"stop","token":"manual-stop"}"#,
        )
        .wait()
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        time += 0.1;
        let output = controller.step(time).unwrap();
        assert_eq!((output.left_effort, output.right_effort), (0.0, 0.0));
        let state: serde_json::Value = serde_json::from_str(&controller.autonomy_status()).unwrap();
        if state["status"]["safety"] == "emergency_stop" {
            break;
        }
        assert!(
            Instant::now() < deadline,
            "sensor-free remote stop was not accepted"
        );
        std::thread::sleep(Duration::from_millis(10));
    }
    controller.disconnect_dashboard().unwrap();
    assert_eq!(controller.dashboard_status(), "disconnected");
    let state: serde_json::Value = serde_json::from_str(&controller.autonomy_status()).unwrap();
    // step refreshes the snapshot after detach.
    controller.step(time + 0.1).unwrap();
    let updated: serde_json::Value = serde_json::from_str(&controller.autonomy_status()).unwrap();
    assert_eq!(updated["status"]["requested_level"], "teleop");
    assert!(state["goal"]["token"].is_null());
}

#[test]
fn dashboard_drive_reaches_the_real_mobile_arbiter_with_queue_age() {
    use terra_mobile::{ImuReading, VioReading};
    let socket = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let endpoint = format!("tcp/127.0.0.1:{}", socket.local_addr().unwrap().port()); drop(socket);
    let mut config = zenoh::Config::default();
    config.insert_json5("mode", "\"router\"").unwrap();
    config.insert_json5("listen/endpoints", &serde_json::json!([endpoint]).to_string()).unwrap();
    config.insert_json5("scouting/multicast/enabled", "false").unwrap();
    let router = zenoh::open(config).wait().unwrap();
    let controller = MobileController::new(default_control_settings()).unwrap();
    controller.connect_dashboard(endpoint, "drive-check".into(), 7).unwrap();
    let start = Instant::now();
    let mut sequence = 0;
    let mut accepted = false;
    while start.elapsed() < Duration::from_secs(3) {
        let now = 100. + start.elapsed().as_secs_f64();
        controller.push_imu(ImuReading {timestamp:now,acceleration_forward:0.,acceleration_left:0.,acceleration_up:0.,gyro_roll:0.,gyro_pitch:0.,gyro_yaw:0.}).unwrap();
        controller.push_vio(VioReading {timestamp:now,position_x:0.,position_y:0.,position_z:0.,quaternion_x:0.,quaternion_y:0.,quaternion_z:0.,quaternion_w:1.,velocity_x:0.,velocity_y:0.,velocity_z:0.,tracked:true}).unwrap();
        controller.step(now).unwrap();
        let status: serde_json::Value = serde_json::from_str(&controller.autonomy_status()).unwrap();
        sequence += 1;
        router.put("drive-check/7/teleop",serde_json::json!({"linear":0.3,"angular":0.,"run_id":status["status"]["run_id"],"authority_revision":status["status"]["revision"],"operator_session_id":"test","sequence":sequence}).to_string()).wait().unwrap();
        std::thread::sleep(Duration::from_millis(20));
        let now = 100. + start.elapsed().as_secs_f64();
        controller.push_imu(ImuReading {timestamp:now,acceleration_forward:0.,acceleration_left:0.,acceleration_up:0.,gyro_roll:0.,gyro_pitch:0.,gyro_yaw:0.}).unwrap();
        controller.push_vio(VioReading {timestamp:now,position_x:0.,position_y:0.,position_z:0.,quaternion_x:0.,quaternion_y:0.,quaternion_z:0.,quaternion_w:1.,velocity_x:0.,velocity_y:0.,velocity_z:0.,tracked:true}).unwrap();
        controller.step(now).unwrap();
        let status: serde_json::Value = serde_json::from_str(&controller.autonomy_status()).unwrap();
        if status["status"]["active_source"] == "operator" {
            assert_eq!(status["twist"]["linear"], 0.3, "selected manual intent must reach the phone adapter");
            assert_eq!(status["twist"]["angular"], 0.0);
            accepted=true;break
        }
    }
    assert!(accepted,"routed, fresh teleop was not accepted by the mobile controller");
}
