use serde_json::json;
use terra_observability::{EventEnvelope, EventId, EventInput};
use terra_observability_collector::{Scope, Store, StoreConfig};
fn event(seq: u64) -> EventEnvelope {
    EventEnvelope {
        schema_version: 1,
        id: EventId {
            producer_id: "phone".into(),
            boot_id: "boot".into(),
            sequence: seq,
        },
        input: EventInput::new("4", "control", "pause", json!({"why":"test"})),
    }
}
fn scope() -> Scope {
    Scope {
        producer_id: "phone".into(),
        rovers: vec!["4".into()],
        read: false,
    }
}
fn db() -> (tempfile::TempDir, Store) {
    let d = tempfile::tempdir().unwrap();
    let s = Store::open(StoreConfig {
        path: d.path().join("collector.db"),
        ..Default::default()
    })
    .unwrap();
    (d, s)
}
#[test]
fn lost_response_retry_is_idempotent() {
    let (_d, s) = db();
    let e = event(1);
    s.ingest(&scope(), std::slice::from_ref(&e)).unwrap();
    s.ingest(&scope(), &[e]).unwrap();
    assert_eq!(s.events(&scope(), None, 0, 100).unwrap().len(), 1);
}
#[test]
fn conflicting_id_rejected() {
    let (_d, s) = db();
    let mut e = event(1);
    s.ingest(&scope(), std::slice::from_ref(&e)).unwrap();
    e.input.kind = "different".into();
    assert!(s.ingest(&scope(), &[e]).is_err());
}
#[test]
fn scope_cannot_be_overridden() {
    let (_d, s) = db();
    let mut e = event(1);
    e.input.rover_id = "99".into();
    assert!(s.ingest(&scope(), &[e]).is_err());
    assert!(s.events(&scope(), None, 0, 100).unwrap().is_empty());
}
#[test]
fn uncommitted_batch_not_acknowledged() {
    let (_d, s) = db();
    let e = event(1);
    s.ingest(&scope(), std::slice::from_ref(&e)).unwrap();
    let mut conflict = e;
    conflict.input.kind = "conflict".into();
    assert!(s.ingest(&scope(), &[event(2), conflict]).is_err());
    assert_eq!(s.events(&scope(), None, 0, 100).unwrap().len(), 1);
}
#[test]
fn expired_records_leave_gap() {
    let (_d, s) = db();
    s.ingest(&scope(), &[event(1)]).unwrap();
    s.prune(u64::MAX).unwrap();
    assert!(
        s.events(&scope(), None, 0, 100)
            .unwrap()
            .iter()
            .any(|e| e.event.input.kind == "observability.gap")
    );
}
#[test]
fn secrets_rejected() {
    let (_d, s) = db();
    let mut e = event(1);
    e.input.attributes = json!({"nested":{"password":"secret"}});
    assert!(s.ingest(&scope(), &[e]).is_err());
}
#[test]
fn retention_gap_counts_every_evicted_event() {
    let (_d, s) = db();
    for seq in 0..5 {
        s.ingest(&scope(), &[event(seq)]).unwrap();
    }
    s.prune(u64::MAX).unwrap();
    let rows = s.events(&scope(), None, 0, 100).unwrap();
    let lost: u64 = rows
        .iter()
        .filter(|e| e.event.input.kind == "observability.gap")
        .map(|e| e.event.input.attributes["evicted_events"].as_u64().unwrap())
        .sum();
    assert_eq!(lost, 5);
}
