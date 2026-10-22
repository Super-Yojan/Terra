use crate::storage::{StorageHandle, Work, read_connection};
use crate::{EventEnvelope, EventId, EventInput, JournalConfig, wall_time_ns};
use serde::Serialize;
use std::{
    path::{Path, PathBuf},
    sync::{
        Arc,
        atomic::{AtomicU64, Ordering},
        mpsc::{self, TrySendError},
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
pub(crate) struct Counters {
    pub(crate) producer: String,
    pub(crate) boot: String,
    pub(crate) queued: AtomicU64,
    pub(crate) dropped: AtomicU64,
    pub(crate) pending_gap: AtomicU64,
    pub(crate) persisted: AtomicU64,
    pub(crate) failures: AtomicU64,
    sequence: AtomicU64,
}
impl Counters {
    pub(crate) fn gap(&self, lost: u64) -> EventEnvelope {
        EventEnvelope {
            schema_version: 1,
            id: EventId {
                producer_id: self.producer.clone(),
                boot_id: self.boot.clone(),
                sequence: self.sequence.fetch_add(1, Ordering::Relaxed),
            },
            input: EventInput::new(
                "_producer",
                "observability",
                "observability.gap",
                serde_json::json!({"dropped":lost}),
            ),
        }
    }
}
#[derive(Clone, Debug, Serialize)]
pub struct JournalHealth {
    pub queued: u64,
    pub dropped: u64,
    pub persisted: u64,
    pub failures: u64,
}
pub struct Journal {
    storage: Arc<StorageHandle>,
    counters: Arc<Counters>,
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
        let storage = StorageHandle::open(&config.path, Some(&config))?;
        let counters = Arc::new(Counters {
            producer: config.producer_id,
            boot: uuid::Uuid::new_v4().to_string(),
            queued: 0.into(),
            dropped: 0.into(),
            pending_gap: 0.into(),
            persisted: 0.into(),
            failures: 0.into(),
            sequence: 0.into(),
        });
        storage.register(counters.clone())?;
        Ok(Self { storage, counters })
    }
    /// Bounded in-memory admission only. Never waits on disk, network or a mutex.
    pub fn try_record(&self, mut input: EventInput) -> RecordOutcome {
        crate::redact(&mut input.attributes);
        if !input.validate() || serde_json::to_vec(&input).map_or(true, |b| b.len() > 128 * 1024) {
            return RecordOutcome::Invalid;
        }
        let event = EventEnvelope {
            schema_version: 1,
            id: EventId {
                producer_id: self.counters.producer.clone(),
                boot_id: self.counters.boot.clone(),
                sequence: self.counters.sequence.fetch_add(1, Ordering::Relaxed),
            },
            input,
        };
        self.counters.queued.fetch_add(1, Ordering::Relaxed);
        match self.storage.events.try_send(Work::Event(
            Box::new(event),
            Instant::now(),
            self.counters.clone(),
        )) {
            Ok(()) => RecordOutcome::Queued,
            Err(_) => {
                self.counters.queued.fetch_sub(1, Ordering::Relaxed);
                self.counters.dropped.fetch_add(1, Ordering::Relaxed);
                self.counters.pending_gap.fetch_add(1, Ordering::Relaxed);
                RecordOutcome::Dropped
            }
        }
    }
    pub fn health(&self) -> JournalHealth {
        JournalHealth {
            queued: self.counters.queued.load(Ordering::Relaxed),
            dropped: self.counters.dropped.load(Ordering::Relaxed),
            persisted: self.counters.persisted.load(Ordering::Relaxed),
            failures: self.counters.failures.load(Ordering::Relaxed),
        }
    }
    /// Administrative barrier; never called from a control tick. Failures remain visible in health/gaps.
    pub fn flush(&self, timeout: Duration) -> Result<(), JournalError> {
        let deadline = Instant::now() + timeout;
        let (tx, rx) = mpsc::channel();
        let mut message = Work::Flush(tx);
        loop {
            match self.storage.events.try_send(message) {
                Ok(()) => break,
                Err(TrySendError::Full(m)) => {
                    message = m;
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
#[derive(Clone)]
pub struct JournalReader {
    path: PathBuf,
    storage: Arc<StorageHandle>,
}
impl JournalReader {
    pub(crate) fn attachment_gap(
        &self,
        source: &EventEnvelope,
        hash: &str,
        reason: &str,
    ) -> Result<(), JournalError> {
        use sha2::{Digest, Sha256};
        let identity = format!(
            "{:x}",
            Sha256::digest(format!("{:?}:{hash}:{reason}", source.id).as_bytes())
        );
        let mut event = source.clone();
        event.id.boot_id = format!("delivery-{identity}");
        event.id.sequence = 0;
        event.input.cause_id = Some(source.id.clone());
        event.input.kind = "observability.gap".into();
        event.input.component = "observability".into();
        event.input.wall_time_ns = wall_time_ns();
        event.input.monotonic_ns = 0;
        event.input.attributes =
            serde_json::json!({"reason":reason,"attachment_sha256":hash,"time_basis":"wall_only"});
        self.storage.insert(event)
    }
    pub fn open(path: &Path) -> Result<Self, JournalError> {
        let storage = StorageHandle::open(path, None)?;
        Ok(Self {
            path: storage.path.clone(),
            storage,
        })
    }
    pub(crate) fn delivery_session(
        &self,
    ) -> Result<std::sync::MutexGuard<'_, crate::uploader::UploadSession>, JournalError> {
        self.storage
            .delivery
            .lock()
            .map_err(|_| "upload delivery state poisoned".into())
    }
    pub fn path(&self) -> &Path {
        &self.path
    }
    pub fn pending_count(&self) -> Result<u64, JournalError> {
        let db = read_connection(&self.path)?;
        Ok(db.query_row("SELECT COUNT(*) FROM events WHERE ack=0", [], |r| r.get(0))?)
    }
    pub fn batch(&self, limit: usize) -> Result<Vec<EventEnvelope>, JournalError> {
        let db = read_connection(&self.path)?;
        let mut stmt =
            db.prepare("SELECT payload FROM events WHERE ack=0 ORDER BY rowid LIMIT ?")?;
        let rows = stmt.query_map([limit.min(256)], |r| r.get::<_, String>(0))?;
        rows.map(|r| Ok(serde_json::from_str(&r?)?)).collect()
    }
    pub fn ack(&self, ids: &[EventId]) -> Result<(), JournalError> {
        self.storage.ack(ids)
    }
    pub fn export(&self) -> Result<Vec<EventEnvelope>, JournalError> {
        let db = read_connection(&self.path)?;
        let mut stmt = db.prepare("SELECT payload FROM events ORDER BY rowid")?;
        let rows = stmt.query_map([], |r| r.get::<_, String>(0))?;
        rows.map(|r| Ok(serde_json::from_str(&r?)?)).collect()
    }
}
