use terra_transport::ControlPlane;
use zenoh::Wait;
#[test]
fn phone_control_plane_receives_intent_and_publishes_state() {
    let socket = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let endpoint = format!("tcp/127.0.0.1:{}", socket.local_addr().unwrap().port());
    drop(socket);
    let service = ControlPlane::listen(&endpoint, "terra/rover", 7).unwrap();
    let mut config = zenoh::Config::default();
    config
        .insert_json5(
            "connect/endpoints",
            &serde_json::json!([endpoint]).to_string(),
        )
        .unwrap();
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .unwrap();
    let client = zenoh::open(config).wait().unwrap();
    let sub = client
        .declare_subscriber("terra/rover/7/autonomy/status")
        .wait()
        .unwrap();
    service.publish("autonomy/status", r#"{"requested_level":"teleop"}"#.into());
    assert!(
        sub.recv_timeout(std::time::Duration::from_secs(2))
            .unwrap()
            .is_some()
    );
    client
        .put(
            "terra/rover/7/autonomy",
            r#"{"level":"teleop","token":"test"}"#,
        )
        .wait()
        .unwrap();
    let start = std::time::Instant::now();
    loop {
        let incoming = service.take_actions();
        if !incoming.is_empty() {
            assert_eq!(incoming[0].0, "autonomy");
            break;
        }
        assert!(start.elapsed().as_secs() < 2);
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
}
#[test]
fn listener_is_explicitly_loopback_only() {
    assert!(ControlPlane::listen("tcp/0.0.0.0:7447", "terra/rover", 1).is_err());
}
#[test]
fn emergency_stop_survives_burst_and_is_drained_last() {
    let mut q = std::collections::VecDeque::new();
    terra_transport::enqueue_control_action(
        &mut q,
        "safety".into(),
        br#"{"action":"stop","token":"stop"}"#.to_vec(),
        std::time::Instant::now(),
    );
    for _ in 0..600 {
        terra_transport::enqueue_control_action(
            &mut q,
            "teleop".into(),
            br#"{"linear":1,"angular":0}"#.to_vec(),
            std::time::Instant::now(),
        );
    }
    let actions = terra_transport::drain_control_actions(&mut q);
    assert_eq!(actions.last().unwrap().0, "safety");
    assert!(actions.len() <= 256);
}
#[test]
fn delayed_or_malformed_stop_cannot_displace_distinct_new_stop() {
    let mut q = std::collections::VecDeque::new();
    for payload in [
        br#"{"action":"stop","token":"new"}"#.as_slice(),
        br#"{"action":"stop","token":"old"}"#,
        br#"{"action":"stop"}"#,
    ] {
        terra_transport::enqueue_control_action(
            &mut q,
            "safety".into(),
            payload.to_vec(),
            std::time::Instant::now(),
        );
    }
    assert!(
        terra_transport::drain_control_actions(&mut q)
            .iter()
            .any(|a| serde_json::from_slice::<serde_json::Value>(&a.1).unwrap()["token"] == "new")
    );
}
