//! Search status, actions, and report replay over a loopback control plane.
//! Network scenarios require local TCP sockets and use bounded receipt waits.
use std::time::{Duration, Instant};
use terra_transport::ControlPlane;
use zenoh::Wait;

fn loopback() -> String {
    let socket = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let endpoint = format!("tcp/127.0.0.1:{}", socket.local_addr().unwrap().port());
    drop(socket);
    endpoint
}

fn client(endpoint: &str) -> zenoh::Session {
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
    zenoh::open(config).wait().unwrap()
}

#[test]
/// Exchanges search status and a cancel action between real loopback sessions.
fn remote_search_round_trip_retains_authoritative_state() {
    let endpoint = loopback();
    let service = ControlPlane::listen(&endpoint, "search-test", 7).unwrap();
    let client = client(&endpoint);
    let sub = client
        .declare_subscriber("search-test/7/search/status")
        .wait()
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(3);
    'status: loop {
        service.publish(
            "search/status",
            r#"{"search_id":"s","phase":"completed","report":{"report_id":"s:report"}}"#.into(),
        );
        while let Some(sample) = sub.try_recv().unwrap() {
            let v: serde_json::Value =
                serde_json::from_slice(&sample.payload().to_bytes()).unwrap();
            if v["phase"] == "completed" {
                break 'status;
            }
        }
        assert!(Instant::now() < deadline, "search status unavailable");
        std::thread::sleep(Duration::from_millis(30));
    }
    loop {
        client
            .put(
                "search-test/7/search/action",
                r#"{"version":1,"run_id":"run","search_id":"s","token":"cancel","action":"cancel"}"#,
            )
            .wait()
            .unwrap();
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
}

#[test]
/// Refreshes two outstanding search reports and verifies both report identities
/// are replayed on the shared contractual report topic.
fn all_pending_reports_are_replayed_on_contract_topic() {
    let endpoint = loopback();
    let service = ControlPlane::listen(&endpoint, "reports", 7).unwrap();
    let client = client(&endpoint);
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
