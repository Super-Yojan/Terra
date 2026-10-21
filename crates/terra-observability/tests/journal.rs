use serde_json::json;
use std::time::{Duration, Instant};
use terra_observability::{EventInput, Journal, JournalConfig, JournalReader, RecordOutcome};
fn event() -> EventInput {
    EventInput::new(
        "4",
        "test",
        "motion.pause",
        json!({"reason":"heading", "age":0.61}),
    )
}
fn setup(path: &std::path::Path) -> JournalConfig {
    JournalConfig {
        path: path.into(),
        producer_id: "phone".into(),
        ..Default::default()
    }
}
#[test]
fn records_survive_reopen() {
    let d = tempfile::tempdir().unwrap();
    let path = d.path().join("events.db");
    let j = Journal::open(setup(&path)).unwrap();
    let original = event();
    assert_eq!(j.try_record(original.clone()), RecordOutcome::Queued);
    j.flush(Duration::from_secs(2)).unwrap();
    drop(j);
    let rows = JournalReader::open(&path).unwrap().batch(10).unwrap();
    assert_eq!(rows.len(), 1);
    assert_eq!(rows[0].input, original);
}
#[test]
fn new_boot_keeps_old_identity() {
    let d = tempfile::tempdir().unwrap();
    let path = d.path().join("events.db");
    for _ in 0..2 {
        let j = Journal::open(setup(&path)).unwrap();
        j.try_record(event());
        j.flush(Duration::from_secs(2)).unwrap();
    }
    let rows = JournalReader::open(&path).unwrap().batch(10).unwrap();
    assert_eq!(rows.len(), 2);
    assert_ne!(rows[0].id.boot_id, rows[1].id.boot_id);
}
#[test]
fn disk_stall_does_not_block_try_record() {
    let d = tempfile::tempdir().unwrap();
    let path = d.path().join("events.db");
    let j = Journal::open(setup(&path)).unwrap();
    let db = rusqlite::Connection::open(&path).unwrap();
    db.execute_batch("BEGIN IMMEDIATE").unwrap();
    let start = Instant::now();
    for _ in 0..100 {
        j.try_record(event());
    }
    assert!(start.elapsed() < Duration::from_millis(200));
    db.execute_batch("ROLLBACK").unwrap();
    j.flush(Duration::from_secs(5)).unwrap();
}
#[test]
fn overflow_reports_gap() {
    let d = tempfile::tempdir().unwrap();
    let path = d.path().join("events.db");
    let j = Journal::open(JournalConfig {
        queue_capacity: 1,
        ..setup(&path)
    })
    .unwrap();
    let db = rusqlite::Connection::open(&path).unwrap();
    db.execute_batch("BEGIN IMMEDIATE").unwrap();
    for _ in 0..10000 {
        j.try_record(event());
    }
    assert!(j.health().dropped > 0);
    db.execute_batch("ROLLBACK").unwrap();
    j.flush(Duration::from_secs(5)).unwrap();
    let rows = JournalReader::open(&path).unwrap().batch(100).unwrap();
    assert!(rows.iter().any(|e| e.input.kind == "observability.gap"));
}
#[test]
fn ack_eviction_precedes_pending_eviction() {
    let d = tempfile::tempdir().unwrap();
    let path = d.path().join("events.db");
    let j = Journal::open(JournalConfig {
        max_bytes: 5000,
        ..setup(&path)
    })
    .unwrap();
    let mut large = event();
    large.attributes["padding"] = "x".repeat(3700).into();
    j.try_record(large);
    j.flush(Duration::from_secs(2)).unwrap();
    let r = JournalReader::open(&path).unwrap();
    let first = r.batch(1).unwrap()[0].id.clone();
    r.ack(std::slice::from_ref(&first)).unwrap();
    for _ in 0..6 {
        j.try_record(event());
    }
    j.flush(Duration::from_secs(2)).unwrap();
    let db = rusqlite::Connection::open(&path).unwrap();
    assert_eq!(
        db.query_row(
            "SELECT COUNT(*) FROM events WHERE producer=? AND boot=? AND seq=?",
            rusqlite::params![first.producer_id, first.boot_id, first.sequence],
            |r| r.get::<_, u64>(0)
        )
        .unwrap(),
        0
    );
    assert!(
        !r.batch(100)
            .unwrap()
            .iter()
            .any(|e| e.input.kind == "observability.gap")
    );
}
