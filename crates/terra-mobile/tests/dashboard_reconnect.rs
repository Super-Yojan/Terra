//! Loopback router-loss and explicit reconnect regression scenario.
//! Network scenarios require local TCP sockets and use bounded receipt waits.
use std::time::{Duration, Instant};
use terra_mobile::{MobileController, default_control_settings};
use zenoh::Wait;
/// Open a discovery-free loopback router on the supplied endpoint for reconnect scenarios.
fn router(endpoint: &str) -> zenoh::Session {
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
    zenoh::open(config).wait().unwrap()
}
#[test]
/// Accepts a routed waypoint, closes the router, observes failure, explicitly clears
/// dashboard intent, then reconnects and verifies the old goal is idle rather than replayed.
fn accepted_goal_does_not_resume_after_router_loss_and_explicit_reconnect() {
    let socket = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let endpoint = format!("tcp/127.0.0.1:{}", socket.local_addr().unwrap().port());
    drop(socket);
    let first = router(&endpoint);
    let controller = MobileController::new(default_control_settings()).unwrap();
    controller
        .connect_dashboard(endpoint.clone(), "phone-reconnect".into(), 7)
        .unwrap();
    controller
        .autonomy_request(
            "autonomy".into(),
            r#"{"level":"waypoint","token":"level"}"#.into(),
            1.,
        )
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(3);
    let mut now = 1.;
    loop {
        first
            .put(
                "phone-reconnect/7/goal",
                r#"{"frame":"local","x":1,"y":0,"token":"old-goal"}"#,
            )
            .wait()
            .unwrap();
        now += 0.01;
        controller
            .push_imu(terra_mobile::ImuReading {
                timestamp: now,
                acceleration_forward: 0.,
                acceleration_left: 0.,
                acceleration_up: 0.,
                gyro_roll: 0.,
                gyro_pitch: 0.,
                gyro_yaw: 0.,
            })
            .unwrap();
        controller
            .push_vio(terra_mobile::VioReading {
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
            })
            .unwrap();
        controller.step(now).unwrap();
        let state: serde_json::Value = serde_json::from_str(&controller.autonomy_status()).unwrap();
        if state["goal"]["token"] == "old-goal" {
            break;
        }
        assert!(Instant::now() < deadline, "goal acknowledgement timed out");
        std::thread::sleep(Duration::from_millis(10));
    }
    first.close().wait().unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    while controller.dashboard_status() != "failed" {
        assert!(Instant::now() < deadline, "loss detection timed out");
        std::thread::sleep(Duration::from_millis(20));
    }
    let output = controller.step(now + 0.01).unwrap();
    assert_eq!((output.left_effort, output.right_effort), (0., 0.));
    // The phone lifecycle handles observed loss by detaching and clearing intent.
    controller.disconnect_dashboard().unwrap();
    let _replacement = router(&endpoint);
    controller
        .connect_dashboard(endpoint, "phone-reconnect".into(), 7)
        .unwrap();
    std::thread::sleep(Duration::from_millis(150));
    controller.step(now + 0.02).unwrap();
    let state: serde_json::Value = serde_json::from_str(&controller.autonomy_status()).unwrap();
    assert_eq!(state["goal"]["state"], "idle");
    assert!(state["goal"]["token"].is_null());
    assert_eq!(state["status"]["requested_level"], "teleop");
    assert_eq!(controller.dashboard_status(), "connected");
}
