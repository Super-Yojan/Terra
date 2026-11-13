use super::*;
use std::collections::VecDeque;

#[test]
/// Search and task intents are control topics; their observations and status are not.
fn command_topics_are_explicit() {
    for s in [
        "search",
        "search/action",
        "search/report/ack",
        "task",
        "task/action",
    ] {
        assert!(is_control_topic(s));
    }
    for s in ["search/observation", "search/status", "task/status"] {
        assert!(!is_control_topic(s));
    }
}

#[test]
/// Floods a queue with 600 teleop messages and checks the distinct stop survives
/// within the 256-entry bound and is drained last.
fn emergency_stop_survives_burst_and_is_drained_last() {
    let mut q = VecDeque::new();
    enqueue_control_action(
        &mut q,
        "safety".into(),
        br#"{"action":"stop","token":"stop"}"#.to_vec(),
        Instant::now(),
    );
    for _ in 0..600 {
        enqueue_control_action(
            &mut q,
            "teleop".into(),
            br#"{"linear":1,"angular":0}"#.to_vec(),
            Instant::now(),
        );
    }
    let actions = drain_control_actions(&mut q);
    assert_eq!(actions.last().unwrap().0, "safety");
    assert!(actions.len() <= 256);
}

#[test]
/// Floods search intent around a safety stop and verifies the bounded queue still drains the stop last.
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
/// Queues distinct and malformed safety stops and verifies the newer stop token is retained.
fn delayed_or_malformed_stop_cannot_displace_distinct_new_stop() {
    let mut q = VecDeque::new();
    for payload in [
        br#"{"action":"stop","token":"new"}"#.as_slice(),
        br#"{"action":"stop","token":"old"}"#,
        br#"{"action":"stop"}"#,
    ] {
        enqueue_control_action(&mut q, "safety".into(), payload.to_vec(), Instant::now());
    }
    assert!(
        drain_control_actions(&mut q)
            .iter()
            .any(|a| serde_json::from_slice::<serde_json::Value>(&a.1).unwrap()["token"] == "new")
    );
}
