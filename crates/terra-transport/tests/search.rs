use std::{collections::VecDeque, time::Instant};
use terra_transport::*;
#[test]
fn search_command_topics_are_explicit() {
    for s in ["search", "search/action", "search/report/ack"] {
        assert!(is_control_topic(s));
    }
    assert!(!is_control_topic("search/observation"));
    assert!(!is_control_topic("search/status"));
}
#[test]
fn stops_dominate_search_start_and_resume() {
    let mut q = VecDeque::new();
    enqueue_control_action(
        &mut q,
        "safety".into(),
        br#"{"action":"stop","token":"s"}"#.to_vec(),
        Instant::now(),
    );
    for _ in 0..300 {
        enqueue_control_action(&mut q, "search".into(), b"{}".to_vec(), Instant::now());
    }
    let a = drain_control_actions(&mut q);
    assert_eq!(a.last().unwrap().0, "safety");
    assert!(a.len() <= 256);
}
#[test]
fn remote_search_round_trip_retains_authoritative_state() {
    use std::time::Duration;
    let socket = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let endpoint = format!("tcp/127.0.0.1:{}", socket.local_addr().unwrap().port());
    drop(socket);
    let service = ControlPlane::listen(&endpoint, "search-test", 7).unwrap();
    let client = RoverConnection::connect(&endpoint, "search-test", 7).unwrap();
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        service.publish(
            "autonomy/status",
            r#"{"requested_level":"target_search","supported_levels":["target_search"]}"#.into(),
        );
        service.publish(
            "search/status",
            r#"{"search_id":"s","phase":"completed","report":{"report_id":"s:report"}}"#.into(),
        );
        let v: serde_json::Value =
            serde_json::from_str(&client.autonomy_status()).unwrap_or_default();
        if v["search"]["phase"] == "completed" {
            break;
        }
        assert!(Instant::now() < deadline, "search status unavailable");
        std::thread::sleep(Duration::from_millis(30));
    }
    client
        .send_action(
            "search/action",
            r#"{"version":1,"run_id":"run","search_id":"s","token":"cancel","action":"cancel"}"#,
        )
        .unwrap();
    loop {
        if service
            .take_actions()
            .iter()
            .any(|a| a.0 == "search/action")
        {
            break;
        }
        assert!(Instant::now() < deadline, "search action unavailable");
        std::thread::sleep(Duration::from_millis(20));
    }
    client.disconnect();
}
#[test]
fn all_pending_reports_are_replayed_on_contract_topic() {
    use std::time::Duration;
    use zenoh::Wait;
    let socket = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let endpoint = format!("tcp/127.0.0.1:{}", socket.local_addr().unwrap().port());
    drop(socket);
    let service = ControlPlane::listen(&endpoint, "reports", 7).unwrap();
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
        .declare_subscriber("reports/7/search/report")
        .wait()
        .unwrap();
    let mut seen = std::collections::BTreeSet::new();
    let start = Instant::now();
    while seen.len() < 2 {
        service.publish("autonomy/status", "{}".into());
        service.publish_batch(
            "search/report",
            vec![
                r#"{"report_id":"first"}"#.into(),
                r#"{"report_id":"second"}"#.into(),
            ],
        );
        while let Some(sample) = sub.try_recv().unwrap() {
            let v: serde_json::Value =
                serde_json::from_slice(&sample.payload().to_bytes()).unwrap();
            seen.insert(v["report_id"].as_str().unwrap().to_string());
        }
        assert!(
            start.elapsed() < Duration::from_secs(3),
            "only received {seen:?}"
        );
        std::thread::sleep(Duration::from_millis(30));
    }
}
