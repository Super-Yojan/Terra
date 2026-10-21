use crate::{EventEnvelope, EventId, EventInput, JournalConfig, wall_time_ns};
use rusqlite::{Connection, params};
use serde::Serialize;
use std::{
    path::{Path, PathBuf},
    sync::{
        Arc,
        atomic::{AtomicU64, Ordering},
        mpsc::{self, SyncSender, TrySendError},
    },
    thread,
    time::{Duration, Instant},
};
pub type JournalError = Box<dyn std::error::Error + Send + Sync>;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RecordOutcome {
    Queued,
    Dropped,
    Invalid,
}
#[derive(Default)]
struct Counters {
    dropped: AtomicU64,
    pending_gap: AtomicU64,
    persisted: AtomicU64,
    failures: AtomicU64,
    sequence: AtomicU64,
}
#[derive(Clone, Debug, Serialize)]
pub struct JournalHealth {
    pub dropped: u64,
    pub persisted: u64,
    pub failures: u64,
}
enum Message {
    Event(Box<EventEnvelope>),
    Flush(mpsc::Sender<()>),
}
pub struct Journal {
    sender: SyncSender<Message>,
    counters: Arc<Counters>,
    producer: String,
    boot: String,
}
impl Journal {
    pub fn open(config: JournalConfig) -> Result<Self, JournalError> {
        if config.queue_capacity == 0
            || config.queue_capacity > 65536
            || config.max_bytes < 1024
            || config.retention_seconds == 0
            || config.producer_id.is_empty()
            || config.producer_id.len() > 256
        {
            return Err("invalid journal configuration".into());
        }
        let db = open_db(&config.path)?;
        let (sender, receiver) = mpsc::sync_channel(config.queue_capacity);
        let counters = Arc::new(Counters::default());
        let state = counters.clone();
        let boot = uuid::Uuid::new_v4().to_string();
        let producer = config.producer_id.clone();
        let worker_boot = boot.clone();
        let worker_producer = producer.clone();
        thread::Builder::new()
            .name("observation-journal".into())
            .spawn(move || {
                loop {
                    let msg = receiver.recv_timeout(Duration::from_millis(250));
                    // This counter is outside the bounded queue, so overflow itself remains visible.
                    let gap = state.pending_gap.swap(0, Ordering::AcqRel);
                    if gap > 0 {
                        let envelope = EventEnvelope {
                            schema_version: 1,
                            id: EventId {
                                producer_id: worker_producer.clone(),
                                boot_id: worker_boot.clone(),
                                sequence: state.sequence.fetch_add(1, Ordering::Relaxed),
                            },
                            input: EventInput::new(
                                "_producer",
                                "observability",
                                "observability.gap",
                                serde_json::json!({"dropped":gap}),
                            ),
                        };
                        if persist(&db, &envelope, &config).is_err() {
                            state.pending_gap.fetch_add(gap, Ordering::Relaxed);
                        }
                    }
                    match msg {
                        Ok(Message::Event(e)) => {
                            if persist(&db, &e, &config).is_ok() {
                                state.persisted.fetch_add(1, Ordering::Relaxed);
                            } else {
                                state.failures.fetch_add(1, Ordering::Relaxed);
                                state.dropped.fetch_add(1, Ordering::Relaxed);
                                state.pending_gap.fetch_add(1, Ordering::Relaxed);
                            }
                        }
                        Ok(Message::Flush(reply)) => {
                            let _ = reply.send(());
                        }
                        Err(mpsc::RecvTimeoutError::Timeout) => continue,
                        Err(mpsc::RecvTimeoutError::Disconnected) => break,
                    }
                }
            })?;
        Ok(Self {
            sender,
            counters,
            producer,
            boot,
        })
    }
    /// Bounded in-memory admission only. Never waits on disk, network or a mutex.
    pub fn try_record(&self, input: EventInput) -> RecordOutcome {
        if !input.validate() || serde_json::to_vec(&input).map_or(true, |b| b.len() > 128 * 1024) {
            return RecordOutcome::Invalid;
        }
        let event = EventEnvelope {
            schema_version: 1,
            id: EventId {
                producer_id: self.producer.clone(),
                boot_id: self.boot.clone(),
                sequence: self.counters.sequence.fetch_add(1, Ordering::Relaxed),
            },
            input,
        };
        match self.sender.try_send(Message::Event(Box::new(event))) {
            Ok(()) => RecordOutcome::Queued,
            Err(_) => {
                self.counters.dropped.fetch_add(1, Ordering::Relaxed);
                self.counters.pending_gap.fetch_add(1, Ordering::Relaxed);
                RecordOutcome::Dropped
            }
        }
    }
    pub fn health(&self) -> JournalHealth {
        JournalHealth {
            dropped: self.counters.dropped.load(Ordering::Relaxed),
            persisted: self.counters.persisted.load(Ordering::Relaxed),
            failures: self.counters.failures.load(Ordering::Relaxed),
        }
    }
    /// Administrative barrier, not called from a control tick. Failures remain in health/gap records.
    pub fn flush(&self, timeout: Duration) -> Result<(), JournalError> {
        let deadline = Instant::now() + timeout;
        let (tx, rx) = mpsc::channel();
        let mut msg = Message::Flush(tx);
        loop {
            match self.sender.try_send(msg) {
                Ok(()) => break,
                Err(TrySendError::Full(m)) => {
                    msg = m;
                    if Instant::now() >= deadline {
                        return Err("flush queue timeout".into());
                    }
                    thread::sleep(Duration::from_millis(1));
                }
                Err(_) => return Err("journal worker stopped".into()),
            }
        }
        rx.recv_timeout(deadline.saturating_duration_since(Instant::now()))?;
        Ok(())
    }
}
fn open_db(path: &Path) -> Result<Connection, JournalError> {
    if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
        std::fs::create_dir_all(parent)?;
    }
    let db = Connection::open(path)?;
    db.busy_timeout(Duration::from_secs(2))?;
    db.execute_batch("PRAGMA journal_mode=WAL; PRAGMA synchronous=FULL; PRAGMA journal_size_limit=1048576; CREATE TABLE IF NOT EXISTS events (rowid INTEGER PRIMARY KEY, producer TEXT NOT NULL, boot TEXT NOT NULL, seq INTEGER NOT NULL, payload TEXT NOT NULL, bytes INTEGER NOT NULL, stored_ns INTEGER NOT NULL, ack INTEGER NOT NULL DEFAULT 0, UNIQUE(producer,boot,seq)); CREATE INDEX IF NOT EXISTS events_pending ON events(ack,rowid);")?;
    Ok(db)
}
fn persist(
    db: &Connection,
    event: &EventEnvelope,
    config: &JournalConfig,
) -> Result<(), JournalError> {
    let payload = serde_json::to_string(event)?;
    let now = wall_time_ns();
    let tx = db.unchecked_transaction()?;
    tx.execute(
        "INSERT INTO events(producer,boot,seq,payload,bytes,stored_ns) VALUES(?,?,?,?,?,?)",
        params![
            event.id.producer_id,
            event.id.boot_id,
            event.id.sequence,
            payload,
            payload.len(),
            now
        ],
    )?;
    let cutoff = now.saturating_sub(config.retention_seconds.saturating_mul(1_000_000_000));
    // Retention and pressure always remove acknowledged rows before pending rows.
    tx.execute("DELETE FROM events WHERE ack=1 AND stored_ns<?", [cutoff])?;
    let mut total: u64 = tx.query_row("SELECT COALESCE(SUM(bytes),0) FROM events", [], |r| {
        r.get(0)
    })?;
    let mut lost = 0u64;
    loop {
        let expired: bool = tx.query_row(
            "SELECT EXISTS(SELECT 1 FROM events WHERE stored_ns<?)",
            [cutoff],
            |r| r.get(0),
        )?;
        if total <= config.max_bytes && !expired {
            break;
        }
        let (row, bytes, ack): (i64, u64, bool) = tx.query_row(
            "SELECT rowid,bytes,ack FROM events ORDER BY ack DESC,rowid LIMIT 1",
            [],
            |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?)),
        )?;
        tx.execute("DELETE FROM events WHERE rowid=?", [row])?;
        total = total.saturating_sub(bytes);
        if !ack {
            lost += 1;
        }
    }
    if lost > 0 {
        let mut gap = event.clone();
        gap.input = EventInput::new(
            &event.input.rover_id,
            "observability",
            "observability.gap",
            serde_json::json!({"evicted_events":lost,"reason":"retention_or_quota"}),
        );
        // Keep a loss marker with its own storage identity, never reuse the source event's ID.
        gap.id.boot_id = uuid::Uuid::new_v4().to_string();
        gap.id.sequence = 0;
        let data = serde_json::to_string(&gap)?;
        tx.execute(
            "INSERT INTO events(producer,boot,seq,payload,bytes,stored_ns) VALUES(?,?,?,?,?,?)",
            params![gap.id.producer_id, gap.id.boot_id, 0, data, data.len(), now],
        )?;
    }
    tx.commit()?;
    Ok(())
}
#[derive(Clone)]
pub struct JournalReader {
    path: PathBuf,
}
impl JournalReader {
    pub fn open(path: &Path) -> Result<Self, JournalError> {
        let _ = open_db(path)?;
        Ok(Self { path: path.into() })
    }
    pub fn path(&self) -> &Path {
        &self.path
    }
    pub fn batch(&self, limit: usize) -> Result<Vec<EventEnvelope>, JournalError> {
        let db = open_db(&self.path)?;
        let mut stmt =
            db.prepare("SELECT payload FROM events WHERE ack=0 ORDER BY rowid LIMIT ?")?;
        let rows = stmt.query_map([limit.min(256)], |r| r.get::<_, String>(0))?;
        rows.map(|r| Ok(serde_json::from_str(&r?)?)).collect()
    }
    pub fn ack(&self, ids: &[EventId]) -> Result<(), JournalError> {
        let db = open_db(&self.path)?;
        let tx = db.unchecked_transaction()?;
        for id in ids {
            tx.execute(
                "UPDATE events SET ack=1 WHERE producer=? AND boot=? AND seq=?",
                params![id.producer_id, id.boot_id, id.sequence],
            )?;
        }
        tx.commit()?;
        Ok(())
    }
    pub fn export(&self) -> Result<Vec<EventEnvelope>, JournalError> {
        let db = open_db(&self.path)?;
        let mut stmt = db.prepare("SELECT payload FROM events ORDER BY rowid")?;
        let rows = stmt.query_map([], |r| r.get::<_, String>(0))?;
        rows.map(|r| Ok(serde_json::from_str(&r?)?)).collect()
    }
}
