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
    assert_eq!(
        fixture["secret_key_fragments"],
        serde_json::json!(SECRET_KEY_FRAGMENTS)
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
