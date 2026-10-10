use std::time::{Duration, Instant};
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
fn outbound_phone_routes_telemetry_and_commands_without_replay() {
    let (_router, endpoint) = router();
    let operator = client(&endpoint);
    let pose = operator
        .declare_subscriber("phone-test/7/pose")
        .wait()
        .unwrap();
    let service = ControlPlane::connect(&endpoint, "phone-test", 7).unwrap();
    assert!(!service.failed());
    service.publish(
        "pose",
        r#"{"rover_id":7,"sequence":1,"x":2,"y":3,"yaw":0}"#.into(),
    );
    assert!(pose.recv_timeout(Duration::from_secs(2)).unwrap().is_some());
    let mut actions = vec![];
    wait_for(|| {
        operator
            .put(
                "phone-test/7/goal",
                r#"{"frame":"local","x":1,"y":0,"token":"one"}"#,
            )
            .wait()
            .unwrap();
        actions = service.take_actions();
        !actions.is_empty()
    });
    assert!(actions.iter().all(|a| a.0 == "goal"));
    // Leave another command queued in the old session, then replace it.
    operator
        .put(
            "phone-test/7/goal",
            r#"{"frame":"local","x":9,"y":0,"token":"queued"}"#,
        )
        .wait()
        .unwrap();
    std::thread::sleep(Duration::from_millis(100));
    drop(service);
    let replacement = ControlPlane::connect(&endpoint, "phone-test", 7).unwrap();
    std::thread::sleep(Duration::from_millis(150));
    assert!(replacement.take_actions().is_empty());
}
#[test]
fn cached_telemetry_expires_when_control_ticks_stop() {
    let (_router, endpoint) = router();
    let operator = client(&endpoint);
    let pose = operator
        .declare_subscriber("phone-expiry/7/pose")
        .wait()
        .unwrap();
    let service = ControlPlane::connect(&endpoint, "phone-expiry", 7).unwrap();
    service.publish("pose", "{}".into());
    assert!(pose.recv_timeout(Duration::from_secs(2)).unwrap().is_some());
    std::thread::sleep(Duration::from_millis(650));
    while pose.try_recv().unwrap().is_some() {}
    assert!(
        pose.recv_timeout(Duration::from_millis(300))
            .unwrap()
            .is_none()
    );
}
#[test]
fn router_loss_is_a_latched_failure() {
    let (router, endpoint) = router();
    let service = ControlPlane::connect(&endpoint, "phone-loss", 7).unwrap();
    router.close().wait().unwrap();
    wait_for(|| service.failed());
    assert!(service.take_actions().is_empty());
}
#[test]
fn unreachable_router_returns_an_error() {
    let socket = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let endpoint = format!("tcp/127.0.0.1:{}", socket.local_addr().unwrap().port());
    drop(socket);
    assert!(ControlPlane::connect(&endpoint, "phone-unreachable", 7).is_err());
}

#[test]
fn sends_real_sized_cloud_and_occupancy_snapshots_but_bounds_other_topics() {
    let (_router, endpoint) = router();
    let operator = client(&endpoint);
    let cloud = operator
        .declare_subscriber("phone-large/7/pointcloud")
        .wait()
        .unwrap();
    let map = operator
        .declare_subscriber("phone-large/7/map/occupancy")
        .wait()
        .unwrap();
    let pose = operator
        .declare_subscriber("phone-large/7/pose")
        .wait()
        .unwrap();
    let service = ControlPlane::connect(&endpoint, "phone-large", 7).unwrap();
    let points =
        serde_json::json!({"version":1,"points":vec![[1.123456789,2.123456789,3.123456789];1800]})
            .to_string();
    let cells = serde_json::json!({"schema_version":1,"width":200,"height":200,"occupancy":vec![100;40000]}).to_string();
    assert!(points.len() > 65536 && points.len() <= 262144);
    assert!(cells.len() > 65536 && cells.len() <= 262144);
    service.publish("pointcloud", points.clone());
    service.publish("map/occupancy", cells.clone());
    let received = cloud
        .recv_timeout(Duration::from_secs(2))
        .unwrap()
        .expect("large cloud dropped");
    assert_eq!(received.payload().to_bytes().as_ref(), points.as_bytes());
    let received = map
        .recv_timeout(Duration::from_secs(2))
        .unwrap()
        .expect("large occupancy dropped");
    assert_eq!(received.payload().to_bytes().as_ref(), cells.as_bytes());
    std::thread::sleep(Duration::from_millis(600));
    while cloud.try_recv().unwrap().is_some() {}
    service.publish("pointcloud", "x".repeat(262145));
    service.publish("pose", "x".repeat(65537));
    assert!(
        cloud
            .recv_timeout(Duration::from_millis(300))
            .unwrap()
            .is_none()
    );
    assert!(
        pose.recv_timeout(Duration::from_millis(300))
            .unwrap()
            .is_none()
    );
}
