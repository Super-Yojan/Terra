//! WebRTC signaling stays on its own queue and size cap.
use std::{
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};
use terra_transport::ControlPlane;
use zenoh::Wait;

fn router() -> (zenoh::Session, String) {
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
    (zenoh::open(config).wait().unwrap(), endpoint)
}

fn client(endpoint: &str) -> zenoh::Session {
    let mut config = zenoh::Config::default();
    config.insert_json5("mode", "\"client\"").unwrap();
    config
        .insert_json5(
            "connect/endpoints",
            &serde_json::json!([endpoint]).to_string(),
        )
        .unwrap();
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .unwrap();
    zenoh::open(config).wait().unwrap()
}

fn wait_for(mut predicate: impl FnMut() -> bool) {
    let deadline = Instant::now() + Duration::from_secs(5);
    while !predicate() {
        assert!(Instant::now() < deadline, "condition timed out");
        std::thread::sleep(Duration::from_millis(20));
    }
}

#[test]
fn signaling_bypasses_the_2048_byte_control_inbox_and_cannot_publish_motion() {
    let (_router, endpoint) = router();
    let operator = client(&endpoint);
    let seen = Arc::new(Mutex::new(Vec::<(String, Vec<u8>)>::new()));
    let collect = seen.clone();
    let _signals = operator
        .declare_subscriber("fpv-test/3/webrtc/**")
        .callback(move |sample| {
            collect.lock().unwrap().push((
                sample.key_expr().as_str().to_string(),
                sample.payload().to_bytes().to_vec(),
            ));
        })
        .wait()
        .unwrap();
    let service = ControlPlane::connect(&endpoint, "fpv-test", 3).unwrap();

    let small_teleop = r#"{"linear":0.1,"angular":0}"#;
    let oversized_teleop = format!("{{\"linear\":0,\"pad\":\"{}\"}}", "x".repeat(3000));
    assert!(oversized_teleop.len() > 2048);
    let sdp = "v=0\r\n".repeat(400);
    let request = format!(
        r#"{{"v":1,"action":"start","session":"11111111-1111-4111-8111-111111111111","sdp":"{sdp}"}}"#
    );
    assert!(request.len() > 2048);
    assert!(request.len() <= terra_transport::WEBRTC_SIGNALING_MAX_BYTES);
    let oversized_signal = "y".repeat(terra_transport::WEBRTC_SIGNALING_MAX_BYTES + 1);

    operator
        .put("fpv-test/3/teleop", oversized_teleop.as_str())
        .wait()
        .unwrap();
    operator
        .put("fpv-test/3/teleop", small_teleop)
        .wait()
        .unwrap();
    operator
        .put(
            "fpv-test/3/hardware",
            r#"{"action":"disarm","token":"fpv-must-not-mix","run_id":"run","authority_revision":0}"#,
        )
        .wait()
        .unwrap();
    operator
        .put("fpv-test/3/webrtc/request", oversized_signal.as_str())
        .wait()
        .unwrap();
    operator
        .put("fpv-test/3/webrtc/request", request.as_str())
        .wait()
        .unwrap();

    let mut actions = Vec::new();
    let mut signals = Vec::new();
    wait_for(|| {
        actions.extend(service.take_actions());
        signals.extend(service.take_signaling());
        actions.iter().any(|action| action.0 == "teleop")
            && actions.iter().any(|action| action.0 == "hardware")
            && signals.iter().any(|signal| signal.0 == "webrtc/request")
    });
    assert!(
        actions
            .iter()
            .all(|action| action.0 == "teleop" || action.0 == "hardware")
    );
    assert!(actions.iter().all(|action| action.1.len() <= 2048));
    assert!(signals.iter().all(|signal| signal.0 == "webrtc/request"));
    assert!(signals.iter().all(|signal| signal.1.len() > 2048));
    assert!(
        signals
            .iter()
            .all(|signal| signal.1.len() <= terra_transport::WEBRTC_SIGNALING_MAX_BYTES)
    );

    assert!(!service.publish_signaling("hardware", r#"{"action":"arm","token":"no"}"#));
    assert!(!service.publish_signaling("teleop", r#"{"linear":1,"angular":0}"#));
    assert!(!service.publish_signaling("webrtc/offer", "not-json"));
    assert!(service.take_actions().is_empty());

    service.publish(
        "pose",
        r#"{"rover_id":3,"sequence":1,"x":0,"y":0,"yaw":0}"#.into(),
    );
    service.publish(
        "webrtc/offer",
        r#"{"v":1,"session":"11111111-1111-4111-8111-111111111111","sdp":"latched"}"#.into(),
    );
    assert!(service.publish_signaling(
        "webrtc/state",
        r#"{"v":1,"session":"11111111-1111-4111-8111-111111111111","state":"connecting"}"#,
    ));
    wait_for(|| {
        seen.lock()
            .unwrap()
            .iter()
            .any(|(key, _)| key.ends_with("/webrtc/state"))
    });
    std::thread::sleep(Duration::from_millis(400));
    let published = seen.lock().unwrap().clone();
    assert!(
        published
            .iter()
            .any(|(key, body)| key.ends_with("/webrtc/state")
                && body.windows(10).any(|w| w == b"connecting"))
    );
    assert!(
        published
            .iter()
            .all(|(key, body)| !(key.ends_with("/webrtc/offer")
                && body.windows(7).any(|w| w == b"latched")))
    );
}
