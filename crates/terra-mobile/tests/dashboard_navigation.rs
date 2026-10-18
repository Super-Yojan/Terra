//! Real loopback transport. No sensors, Bluetooth, or motor authority.
use std::time::{Duration, Instant};
use terra_mobile::{MobileController, default_control_settings};
use zenoh::Wait;
#[test]
/// Routes geometry settings through loopback with fresh disarmed hardware and checks
/// configuration revision/acknowledgement while all wheel efforts remain zero.
fn remote_navigation_settings_reach_the_real_controller_without_motor_output() {
    let socket = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let endpoint = format!("tcp/127.0.0.1:{}", socket.local_addr().unwrap().port());
    drop(socket);
    let mut cfg = zenoh::Config::default();
    cfg.insert_json5("mode", "\"router\"").unwrap();
    cfg.insert_json5(
        "listen/endpoints",
        &serde_json::json!([endpoint]).to_string(),
    )
    .unwrap();
    cfg.insert_json5("scouting/multicast/enabled", "false")
        .unwrap();
    let router = zenoh::open(cfg).wait().unwrap();
    let status = router
        .declare_subscriber("navigation-check/7/autonomy/status")
        .wait()
        .unwrap();
    let c = MobileController::new(default_control_settings()).unwrap();
    c.connect_dashboard(endpoint, "navigation-check".into(), 7)
        .unwrap();
    c.step(1.).unwrap();
    let ready_deadline = Instant::now() + Duration::from_secs(3);
    let mut ready_time = 1.0;
    loop {
        ready_time += 0.01;
        c.step(ready_time).unwrap();
        if status.try_recv().unwrap().is_some() {
            break;
        }
        assert!(
            Instant::now() < ready_deadline,
            "initial telemetry did not arrive"
        );
        std::thread::sleep(Duration::from_millis(10));
    }
    let before: serde_json::Value = serde_json::from_str(&c.autonomy_status()).unwrap();
    let request = serde_json::json!({"token":"navigation-wire-one","run_id":before["status"]["run_id"],"authority_revision":before["status"]["revision"],"settings_revision":0,"parameters":{"length":0.8,"width":0.6,"clearance":0.1,"max_linear":0.5,"max_angular":1.,"horizon":2.5}});
    router
        .put(
            "navigation-check/7/navigation/settings",
            request.to_string(),
        )
        .wait()
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(3);
    let mut t = ready_time;
    loop {
        c.publish_hardware_status(r#"{"ready":true,"armed":false,"arming":false}"#.into())
            .unwrap();
        t += 0.01;
        let out = c.step(t).unwrap();
        assert_eq!((out.left_effort, out.right_effort), (0., 0.));
        let v: serde_json::Value = serde_json::from_str(&c.autonomy_status()).unwrap();
        assert!(
            !v["navigation"]["result"]
                .as_str()
                .is_some_and(|s| s.starts_with("rejected")),
            "configuration rejected: {}",
            v["navigation"]["result"]
        );
        if v["navigation"]["result"] == "accepted" {
            assert_eq!(v["navigation"]["parameters"]["length"], 0.8);
            assert_eq!(v["navigation"]["revision"], 1);
            break;
        }
        assert!(
            Instant::now() < deadline,
            "navigation settings did not arrive"
        );
        std::thread::sleep(Duration::from_millis(10));
    }
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        t += 0.01;
        c.step(t).unwrap();
        while let Some(sample) = status.try_recv().unwrap() {
            let v: serde_json::Value =
                serde_json::from_slice(&sample.payload().to_bytes()).unwrap();
            if v["navigation"]["token"] == "navigation-wire-one"
                && v["navigation"]["result"] == "accepted"
            {
                assert_eq!(v["navigation"]["schema_version"], 1);
                c.disconnect_dashboard().unwrap();
                return;
            }
        }
        assert!(
            Instant::now() < deadline,
            "configuration acknowledgement not published"
        );
        std::thread::sleep(Duration::from_millis(10));
    }
}
