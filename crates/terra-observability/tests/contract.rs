//! The Rust contract constants agree with the fixture the rover-pi journal tests also read.
use terra_observability::*;

const FIXTURE: &str = include_str!("../../../tests/fixtures/observability/contract.json");

#[test]
fn constants_match_the_shared_contract() {
    let fixture: serde_json::Value = serde_json::from_str(FIXTURE).unwrap();
    assert_eq!(fixture["schema_version"], SCHEMA_VERSION);
    assert_eq!(fixture["max_event_bytes"], MAX_EVENT_BYTES);
    assert_eq!(fixture["max_ingest_events"], MAX_INGEST_EVENTS);
    assert_eq!(fixture["max_upload_batch_bytes"], MAX_UPLOAD_BATCH_BYTES);
    assert_eq!(fixture["producer_rover"], PRODUCER_ROVER);
    assert_eq!(fixture["gap_kind"], GAP_KIND);
    assert_eq!(fixture["upload_batch_events"], UPLOAD_BATCH_EVENTS);
    assert_eq!(fixture["max_attachment_bytes"], MAX_ATTACHMENT_BYTES);
    assert_eq!(fixture["max_field_bytes"], MAX_FIELD_BYTES);
    assert_eq!(fixture["severities"], serde_json::json!(Severity::ALL));
    assert_eq!(
        fixture["secret_key_fragments"],
        serde_json::json!(SECRET_KEY_FRAGMENTS)
    );
    assert_eq!(
        fixture["sensitive_journal_words"],
        serde_json::json!(SENSITIVE_JOURNAL_WORDS)
    );
    assert_eq!(
        fixture["sensitive_journal_placeholder"],
        SENSITIVE_JOURNAL_PLACEHOLDER
    );
}

#[test]
fn sensitive_journal_messages_match_the_shared_cases() {
    let fixture: serde_json::Value = serde_json::from_str(FIXTURE).unwrap();
    for case in fixture["journal_messages"].as_array().unwrap() {
        let message = case["message"].as_str().unwrap();
        assert_eq!(
            sensitive_journal_message(message),
            case["sensitive"].as_bool().unwrap(),
            "{message}"
        );
    }
}

#[test]
fn admission_rejects_unknown_severity_and_long_fields() {
    let event = |severity: &str, kind: String| {
        serde_json::from_value::<EventInput>(serde_json::json!({
            "rover_id": "4", "session_id": null, "component": "c", "kind": kind,
            "severity": severity, "wall_time_ns": 0, "monotonic_ns": 0,
            "cause_id": null, "attributes": {}
        }))
    };
    assert!(event("warning", "k".into()).unwrap().validate());
    assert!(event("fatal", "k".into()).is_err());
    assert!(
        !event("info", "k".repeat(MAX_FIELD_BYTES + 1))
            .unwrap()
            .validate()
    );
}

#[test]
fn redaction_leaves_nothing_the_collector_rejects() {
    let mut attributes = serde_json::json!({
        "Authorization": "Bearer x",
        "nested": [{"wifi_password": "x", "ok": 1}],
        "kept": {"refresh_token_hint": "x", "value": 2}
    });
    assert!(contains_secret(&attributes));
    redact(&mut attributes);
    assert!(!contains_secret(&attributes));
    assert_eq!(
        attributes,
        serde_json::json!({"nested": [{"ok": 1}], "kept": {"value": 2}})
    );
}
