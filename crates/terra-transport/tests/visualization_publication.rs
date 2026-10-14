use std::time::Duration;
use terra_transport::ControlPlane;
use zenoh::Wait;

#[test]
fn visualization_is_sent_once_but_control_state_still_replays() {
    let socket = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let endpoint = format!("tcp/127.0.0.1:{}", socket.local_addr().unwrap().port());
    drop(socket);
    let service = ControlPlane::listen(&endpoint, "visualization", 7).unwrap();
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
    let status = client
        .declare_subscriber("visualization/7/autonomy/status")
        .wait()
        .unwrap();
    service.publish("autonomy/status", "{}".into());
    for _ in 0..2 {
        assert!(
            status
                .recv_timeout(Duration::from_secs(2))
                .unwrap()
                .is_some()
        );
    }

    for kind in [
        "map/occupancy",
        "pointcloud",
        "map/occupancy/heartbeat",
        "pointcloud/heartbeat",
    ] {
        let sub = client
            .declare_subscriber(format!("visualization/7/{kind}"))
            .wait()
            .unwrap();
        // Each explicit producer observation routes once, including identical
        // heartbeat identity. Wait for receipt so bounded latest-state coalescing
        // does not deliberately replace an unconsumed producer observation.
        let count = if kind.ends_with("/heartbeat") { 3 } else { 1 };
        for _ in 0..count {
            service.publish(kind, r#"{"sequence":1}"#.into());
            let sample = sub.recv_timeout(Duration::from_secs(2)).unwrap().unwrap();
            assert_eq!(sample.payload().to_bytes().as_ref(), br#"{"sequence":1}"#);
            assert!(
                sub.recv_timeout(Duration::from_millis(250))
                    .unwrap()
                    .is_none(),
                "{kind} replayed without a new producer observation"
            );
        }
    }
}

#[test]
fn visualization_enqueue_reports_rejection_and_accepts_retry_with_topic_caps() {
    let socket = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let endpoint = format!("tcp/127.0.0.1:{}", socket.local_addr().unwrap().port());
    drop(socket);
    let service = ControlPlane::listen(&endpoint, "visualization-size", 7).unwrap();
    assert!(!service.publish_visualization("map/occupancy", "x".repeat(2_000_001)));
    assert!(service.publish_visualization("map/occupancy", "x".repeat(2_000_000)));
    assert!(!service.publish_visualization("pointcloud", "x".repeat(262145)));
    assert!(service.publish_visualization("pointcloud", "x".repeat(262144)));
    assert!(!service.publish_visualization("pointcloud/heartbeat", "x".repeat(65537)));
    assert!(service.publish_visualization("pointcloud/heartbeat", "x".repeat(65536)));
    assert!(!service.publish_visualization("autonomy/status", "{}".into()));
}
