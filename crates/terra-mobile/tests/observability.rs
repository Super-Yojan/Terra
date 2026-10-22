use serde_json::json;
use std::time::Duration;
use terra_mobile::{MobileController, default_control_settings};
use terra_observability::{EventInput, JournalReader};
#[test]
fn router_and_lifecycle_diagnostics_survive_a_locked_journal() {
    let d = tempfile::tempdir().unwrap();
    let path = d.path().join("events.db");
    let c = MobileController::new(default_control_settings()).unwrap();
    c.start_observability(json!({"path":path,"producer_id":"phone","rover_id":"4"}).to_string())
        .unwrap();
    let db = rusqlite::Connection::open(&path).unwrap();
    db.execute_batch("BEGIN IMMEDIATE").unwrap();
    let event = EventInput::new(
        "4",
        "phone",
        "lifecycle.stop",
        json!({"keep_bluetooth":false,"password":"never-log-me","unrelated_payload":"never-copy-me"}),
    );
    c.record_observation(serde_json::to_string(&event).unwrap());
    assert_eq!(c.dashboard_status(), "disconnected");
    let start = std::time::Instant::now();
    loop {
        let text = std::fs::read_to_string(path.with_extension("router-diagnostics.jsonl"))
            .unwrap_or_default();
        if text.contains("lifecycle.stop") && text.contains("disconnected") {
            assert!(!text.contains("never-log-me"));
            assert!(!text.contains("never-copy-me"));
            break;
        }
        assert!(
            start.elapsed() < Duration::from_secs(3),
            "independent router trace missing"
        );
        std::thread::sleep(Duration::from_millis(10));
    }
    db.execute_batch("ROLLBACK").unwrap();
}
#[test]
fn logging_failure_does_not_change_control() {
    let plain = MobileController::new(default_control_settings()).unwrap();
    let observed = MobileController::new(default_control_settings()).unwrap();
    assert!(
        observed
            .start_observability(
                json!({"path":"/dev/null/no.db","producer_id":"test","rover_id":"4"}).to_string()
            )
            .is_err()
    );
    for t in [1.0, 1.01, 1.02] {
        let a = plain.step(t).unwrap();
        let b = observed.step(t).unwrap();
        assert_eq!(format!("{:?}", a), format!("{:?}", b));
    }
}
#[test]
fn rover_switch_keeps_buffer_identity() {
    let d = tempfile::tempdir().unwrap();
    let path = d.path().join("db");
    let c = MobileController::new(default_control_settings()).unwrap();
    c.start_observability(json!({"path":path,"producer_id":"phone","rover_id":"4"}).to_string())
        .unwrap();
    let mut e = EventInput::new(
        "4",
        "bluetooth",
        "connection.lost",
        json!({"reason":"radio"}),
    );
    e.session_id = Some("old".into());
    c.record_observation(serde_json::to_string(&e).unwrap());
    c.set_observability_rover("5".into());
    c.step(1.0).unwrap();
    c.flush_observability(2.0).unwrap();
    let events = JournalReader::open(&path).unwrap().export().unwrap();
    assert!(
        events
            .iter()
            .any(|x| x.input.rover_id == "4" && x.input.session_id.as_deref() == Some("old"))
    );
    assert!(events.iter().any(|x| x.input.rover_id == "5"));
}
#[test]
fn stop_retains_first_cause() {
    let d = tempfile::tempdir().unwrap();
    let path = d.path().join("db");
    let c = MobileController::new(default_control_settings()).unwrap();
    c.start_observability(json!({"path":path,"producer_id":"phone","rover_id":"4"}).to_string())
        .unwrap();
    c.step(1.0).unwrap();
    c.step(1.01).unwrap();
    c.step(1.02).unwrap();
    c.flush_observability(2.0).unwrap();
    let events = JournalReader::open(&path).unwrap().export().unwrap();
    let decisions: Vec<_> = events
        .iter()
        .filter(|x| x.input.kind == "autonomy.transition")
        .collect();
    assert_eq!(decisions.len(), 1);
    assert!(decisions[0].input.attributes["reason"].is_string());
    assert!(
        events
            .iter()
            .any(|e| e.input.kind == "control.output" && e.input.attributes["left_effort"] == 0.0)
    );
    std::thread::sleep(Duration::from_millis(1));
}
#[test]
fn invalid_local_command_retains_rejection() {
    let d = tempfile::tempdir().unwrap();
    let path = d.path().join("db");
    let c = MobileController::new(default_control_settings()).unwrap();
    c.start_observability(json!({"path":path,"producer_id":"phone","rover_id":"4"}).to_string())
        .unwrap();
    assert!(
        c.autonomy_request("autonomy".into(), "malformed".into(), 1.0)
            .is_err()
    );
    c.flush_observability(2.0).unwrap();
    let events = JournalReader::open(&path).unwrap().export().unwrap();
    assert!(events.iter().any(|e| e.input.kind == "command.rejected"));
}

#[test]
fn rover_selection_before_observer_initialization_is_not_overwritten() {
    let d = tempfile::tempdir().unwrap();
    let path = d.path().join("events.db");
    let c = MobileController::new(default_control_settings()).unwrap();
    c.set_observability_rover("5".into());
    c.start_observability(json!({"path":path,"producer_id":"phone","rover_id":"4"}).to_string())
        .unwrap();
    c.step(1.).unwrap();
    c.flush_observability(2.).unwrap();
    let events = JournalReader::open(&path).unwrap().export().unwrap();
    assert!(events.iter().all(|e| e.input.rover_id == "5"));
}
