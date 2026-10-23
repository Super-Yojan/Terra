use serde_json::json;
use terra_observability::{JournalReader, RuntimeObserver};
#[test]
fn sender_and_receiver_keep_correlation() {
    let d = tempfile::tempdir().unwrap();
    let path = d.path().join("events.db");
    let o = RuntimeObserver::open(&path, "dashboard").unwrap();
    o.record("4", "router", "command.sent", json!({"sequence":44}));
    o.record("4", "router", "command.received", json!({"sequence":44}));
    o.flush().unwrap();
    let rows = JournalReader::open(&path).unwrap().export().unwrap();
    assert_eq!(rows.len(), 2);
    assert_eq!(
        rows[0].input.attributes["sequence"],
        rows[1].input.attributes["sequence"]
    );
}
#[test]
fn authentication_fields_redacted() {
    let d = tempfile::tempdir().unwrap();
    let path = d.path().join("events.db");
    let o = RuntimeObserver::open(&path, "dashboard").unwrap();
    o.record(
        "4",
        "router",
        "state",
        json!({"password":"sensitive","nested":{"authorization":"Bearer credential"},"sequence":3}),
    );
    o.flush().unwrap();
    let raw =
        serde_json::to_string(&JournalReader::open(&path).unwrap().export().unwrap()).unwrap();
    assert!(!raw.contains("sensitive"));
    assert!(!raw.contains("Bearer credential"));
    assert!(raw.contains("sequence"));
}

#[test]
fn saved_upload_configuration_is_bounded_and_resolves_private_token_path() {
    let d = tempfile::tempdir().unwrap();
    let path = d.path().join("upload.json");
    assert!(
        terra_observability::load_upload_config(&path)
            .unwrap()
            .is_none()
    );
    std::fs::write(
        &path,
        r#"{"endpoint":"https://example.com/observability","token_file":"upload.token"}"#,
    )
    .unwrap();
    let config = terra_observability::load_upload_config(&path)
        .unwrap()
        .unwrap();
    assert_eq!(config.token_file, d.path().join("upload.token"));
    assert_eq!(config.endpoint, "https://example.com/observability");
    std::fs::write(&path, vec![b'x'; 65537]).unwrap();
    assert!(terra_observability::load_upload_config(&path).is_err());
}
