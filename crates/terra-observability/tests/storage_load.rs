use serde_json::json;
use std::time::{Duration, Instant};
use terra_observability::*;
fn config(path: &std::path::Path) -> JournalConfig {
    JournalConfig {
        path: path.into(),
        producer_id: "load-test".into(),
        ..Default::default()
    }
}
fn event() -> EventInput {
    EventInput::new("0", "control", "state", json!({}))
}
fn legacy_history(path: &std::path::Path) {
    let db = rusqlite::Connection::open(path).unwrap();
    db.execute_batch("CREATE TABLE events(rowid INTEGER PRIMARY KEY,producer TEXT NOT NULL,boot TEXT NOT NULL,seq INTEGER NOT NULL,payload TEXT NOT NULL,bytes INTEGER NOT NULL,stored_ns INTEGER NOT NULL,ack INTEGER NOT NULL DEFAULT 0,UNIQUE(producer,boot,seq));CREATE INDEX events_pending ON events(ack,rowid);").unwrap();
    let tx = db.unchecked_transaction().unwrap();
    let mut e = EventEnvelope {
        schema_version: 1,
        id: EventId {
            producer_id: "history".into(),
            boot_id: "old".into(),
            sequence: 0,
        },
        input: event(),
    };
    e.input.attributes = json!({"padding":"x".repeat(100*1024)});
    for i in 0..1024 {
        e.id.sequence = i;
        let p = serde_json::to_string(&e).unwrap();
        tx.execute("INSERT INTO events(producer,boot,seq,payload,bytes,stored_ns) VALUES('history','old',?,?,?,?)",rusqlite::params![i,p,p.len(),wall_time_ns()]).unwrap();
    }
    tx.commit().unwrap();
}
#[test]
fn large_history_and_two_producers_do_not_starve_acknowledgments() {
    let d = tempfile::tempdir().unwrap();
    let p = d.path().join("events.db");
    legacy_history(&p);
    let a = Journal::open(config(&p)).unwrap();
    let b = Journal::open(JournalConfig {
        producer_id: "second".into(),
        ..config(&p)
    })
    .unwrap();
    let reader = JournalReader::open(&p).unwrap();
    let ids: Vec<_> = reader
        .batch(64)
        .unwrap()
        .into_iter()
        .map(|e| e.id)
        .collect();
    let start = Instant::now();
    for _ in 0..128 {
        assert_eq!(a.try_record(event()), RecordOutcome::Queued);
        assert_eq!(b.try_record(event()), RecordOutcome::Queued);
    }
    assert!(start.elapsed() < Duration::from_millis(200));
    let start = Instant::now();
    reader.ack(&ids).unwrap();
    assert!(
        start.elapsed() < Duration::from_secs(2),
        "acknowledgment starved behind event writes"
    );
    a.flush(Duration::from_secs(4)).unwrap();
    b.flush(Duration::from_secs(4)).unwrap();
    assert_eq!(reader.pending_count().unwrap(), 1024 - 64 + 256);
    assert_eq!(a.health().failures + b.health().failures, 0);
    let db = rusqlite::Connection::open(&p).unwrap();
    let logical: u64 = db
        .query_row(
            "SELECT logical_bytes FROM storage_meta WHERE id=1",
            [],
            |r| r.get(0),
        )
        .unwrap();
    let actual: u64 = db
        .query_row("SELECT SUM(bytes) FROM events", [], |r| r.get(0))
        .unwrap();
    assert_eq!(logical, actual);
}
#[test]
fn simultaneous_policy_conflicts_are_explicit() {
    let d = tempfile::tempdir().unwrap();
    let p = d.path().join("events.db");
    let _a = Journal::open(config(&p)).unwrap();
    assert!(
        Journal::open(JournalConfig {
            max_bytes: 4096,
            ..config(&p)
        })
        .is_err()
    );
}
#[test]
fn canonical_paths_share_policy_and_accounting_survives_rollback() {
    let d = tempfile::tempdir().unwrap();
    let p = d.path().join("events.db");
    let j = Journal::open(config(&p)).unwrap();
    let alias = d.path().join(".").join("events.db");
    assert!(
        Journal::open(JournalConfig {
            retention_seconds: 60,
            ..config(&alias)
        })
        .is_err()
    );
    j.try_record(event());
    j.flush(Duration::from_secs(2)).unwrap();
    let db = rusqlite::Connection::open(&p).unwrap();
    db.execute_batch("BEGIN;DELETE FROM events;ROLLBACK;")
        .unwrap();
    let logical: i64 = db
        .query_row(
            "SELECT logical_bytes FROM storage_meta WHERE id=1",
            [],
            |r| r.get(0),
        )
        .unwrap();
    let actual: i64 = db
        .query_row("SELECT SUM(bytes) FROM events", [], |r| r.get(0))
        .unwrap();
    assert_eq!(logical, actual);
}
#[test]
fn abrupt_exit_rolls_back_event_and_byte_mutations() {
    let d = tempfile::tempdir().unwrap();
    let p = d.path().join("crash.db");
    let status = std::process::Command::new(std::env::current_exe().unwrap())
        .args(["--exact", "storage_exit_helper", "--ignored"])
        .env("TERRA_STORAGE_EXIT_DB", &p)
        .status()
        .unwrap();
    assert_eq!(status.code(), Some(23));
    let reader = JournalReader::open(&p).unwrap();
    assert_eq!(reader.pending_count().unwrap(), 20);
    let db = rusqlite::Connection::open(&p).unwrap();
    let logical: i64 = db
        .query_row(
            "SELECT logical_bytes FROM storage_meta WHERE id=1",
            [],
            |r| r.get(0),
        )
        .unwrap();
    let actual: i64 = db
        .query_row("SELECT SUM(bytes) FROM events", [], |r| r.get(0))
        .unwrap();
    assert_eq!(logical, actual);
}
#[test]
#[ignore = "subprocess fixture, invoked by abrupt_exit_rolls_back_event_and_byte_mutations"]
fn storage_exit_helper() {
    let path = std::env::var_os("TERRA_STORAGE_EXIT_DB").expect("subprocess only");
    let p = std::path::PathBuf::from(path);
    let j = Journal::open(config(&p)).unwrap();
    for _ in 0..20 {
        j.try_record(event());
    }
    j.flush(Duration::from_secs(2)).unwrap();
    let db = rusqlite::Connection::open(&p).unwrap();
    db.execute_batch("BEGIN IMMEDIATE;UPDATE events SET ack=1;DELETE FROM events;")
        .unwrap();
    std::process::exit(23);
}
#[test]
fn dropping_last_handle_does_not_retire_a_draining_writer() {
    let d = tempfile::tempdir().unwrap();
    let p = d.path().join("events.db");
    let journal = Journal::open(config(&p)).unwrap();
    let lock = rusqlite::Connection::open(&p).unwrap();
    lock.execute_batch("BEGIN IMMEDIATE").unwrap();
    for _ in 0..256 {
        assert_eq!(journal.try_record(event()), RecordOutcome::Queued);
    }
    drop(journal);
    let start = Instant::now();
    let reopened = Journal::open(config(&p));
    lock.execute_batch("ROLLBACK").unwrap();
    let reopened =
        reopened.expect("reopening must reuse the draining writer without touching locked SQLite");
    assert!(start.elapsed() < Duration::from_millis(500));
    reopened.flush(Duration::from_secs(5)).unwrap();
    assert_eq!(
        JournalReader::open(&p).unwrap().pending_count().unwrap(),
        256
    );
}
#[test]
fn oversized_legacy_quota_cleanup_is_bounded_and_services_acknowledgments() {
    let d = tempfile::tempdir().unwrap();
    let p = d.path().join("events.db");
    legacy_history(&p);
    let j = Journal::open(JournalConfig {
        max_bytes: 1024 * 1024,
        ..config(&p)
    })
    .unwrap();
    let reader = JournalReader::open(&p).unwrap();
    let ids: Vec<_> = (1000..1024)
        .map(|sequence| EventId {
            producer_id: "history".into(),
            boot_id: "old".into(),
            sequence,
        })
        .collect();
    for _ in 0..5 {
        let start = Instant::now();
        reader.ack(&ids).unwrap();
        assert!(start.elapsed() < Duration::from_secs(2));
        j.try_record(event());
    }
    j.flush(Duration::from_secs(5)).unwrap();
    let text = std::fs::read_to_string(p.with_extension("journal-diagnostics.jsonl")).unwrap();
    let passes: Vec<serde_json::Value> = text
        .lines()
        .filter_map(|l| serde_json::from_str::<serde_json::Value>(l).ok())
        .filter(|v| v["kind"] == "quota.pass")
        .collect();
    assert!(
        !passes.is_empty(),
        "quota cleanup must run in independently committed passes"
    );
    assert!(
        passes
            .iter()
            .all(|v| v["attributes"]["removed_rows"].as_u64().unwrap() <= 64)
    );
}
#[test]
fn sustained_twenty_hertz_production_and_repeated_acknowledgments_make_progress() {
    let d = tempfile::tempdir().unwrap();
    let p = d.path().join("events.db");
    legacy_history(&p);
    let journal = std::sync::Arc::new(Journal::open(config(&p)).unwrap());
    let reader = JournalReader::open(&p).unwrap();
    let writer = journal.clone();
    let producer = std::thread::spawn(move || {
        let until = Instant::now() + Duration::from_secs(2);
        let mut count = 0;
        while Instant::now() < until {
            assert_eq!(writer.try_record(event()), RecordOutcome::Queued);
            count += 1;
            std::thread::sleep(Duration::from_millis(50));
        }
        count
    });
    let until = Instant::now() + Duration::from_secs(2);
    let mut acked = 0;
    let mut rounds = 0;
    while Instant::now() < until {
        let ids: Vec<_> = reader
            .batch(16)
            .unwrap()
            .into_iter()
            .map(|v| v.id)
            .collect();
        let start = Instant::now();
        reader.ack(&ids).unwrap();
        assert!(start.elapsed() < Duration::from_secs(1));
        acked += ids.len() as u64;
        rounds += 1;
        std::thread::sleep(Duration::from_millis(50));
    }
    let produced = producer.join().unwrap();
    journal.flush(Duration::from_secs(2)).unwrap();
    assert!(rounds >= 20);
    assert_eq!(reader.pending_count().unwrap(), 1024 + produced - acked);
    assert_eq!(journal.health().failures, 0);
}
