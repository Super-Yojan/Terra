//! One bounded mutation worker per canonical database path; readers never initialize schema.
use crate::journal::Counters;
use crate::{EventEnvelope, EventId, EventInput, JournalConfig, JournalError, wall_time_ns};
use rusqlite::{Connection, OptionalExtension, params};
use std::{
    collections::HashMap,
    path::{Path, PathBuf},
    sync::{
        Arc, Mutex, OnceLock, Weak,
        mpsc::{self, SyncSender, TrySendError},
    },
    thread,
    time::{Duration, Instant},
};

static WRITERS: OnceLock<Mutex<HashMap<PathBuf, Weak<StorageHandle>>>> = OnceLock::new();
type Reply = mpsc::Sender<Result<(), JournalError>>;
pub(crate) enum Work {
    Event(Box<EventEnvelope>, Instant, Arc<Counters>),
    Flush(mpsc::Sender<()>),
}
enum Admin {
    Register(Arc<Counters>),
    Ack(Vec<EventId>, Reply),
    Insert(Box<EventEnvelope>, Reply),
}
pub(crate) struct StorageHandle {
    pub(crate) path: PathBuf,
    pub(crate) events: SyncSender<Work>,
    admin: SyncSender<Admin>,
    policy: JournalConfig,
    pub(crate) delivery: Mutex<crate::uploader::UploadSession>,
}
impl StorageHandle {
    pub(crate) fn open(
        path: &Path,
        requested: Option<&JournalConfig>,
    ) -> Result<Arc<Self>, JournalError> {
        if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
            std::fs::create_dir_all(parent)?;
        }
        let mut options = std::fs::OpenOptions::new();
        options.write(true).create_new(true);
        #[cfg(unix)]
        {
            use std::os::unix::fs::OpenOptionsExt;
            options.mode(0o600);
        }
        match options.open(path) {
            Ok(_) => {}
            Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => {}
            Err(e) => return Err(e.into()),
        }
        let path = path.canonicalize()?;
        let mut registry = WRITERS
            .get_or_init(Default::default)
            .lock()
            .map_err(|_| "storage registry poisoned")?;
        registry.retain(|_, v| v.strong_count() > 0);
        if let Some(existing) = registry.get(&path).and_then(Weak::upgrade) {
            if requested.is_some_and(|p| {
                p.max_bytes != existing.policy.max_bytes
                    || p.retention_seconds != existing.policy.retention_seconds
                    || p.queue_capacity != existing.policy.queue_capacity
            }) {
                return Err("conflicting storage policy for active journal".into());
            }
            return Ok(existing);
        }
        let (db, mut policy) = initialize(&path, requested)?;
        policy.path = path.clone();
        let (events, rx) = mpsc::sync_channel(policy.queue_capacity);
        let (admin, commands) = mpsc::sync_channel(32);
        let worker_policy = policy.clone();
        let handle = Arc::new(Self {
            path: path.clone(),
            events,
            admin,
            policy,
            delivery: Mutex::new(Default::default()),
        });
        let owner = handle.clone();
        thread::Builder::new()
            .name("observation-storage".into())
            .spawn(move || worker(db, worker_policy, rx, commands, owner))?;
        registry.insert(path, Arc::downgrade(&handle));
        Ok(handle)
    }
    pub(crate) fn register(&self, state: Arc<Counters>) -> Result<(), JournalError> {
        self.send(Admin::Register(state))
    }
    fn send(&self, mut message: Admin) -> Result<(), JournalError> {
        let deadline = Instant::now() + Duration::from_secs(2);
        loop {
            match self.admin.try_send(message) {
                Ok(()) => return Ok(()),
                Err(TrySendError::Full(m)) => {
                    message = m;
                    if Instant::now() >= deadline {
                        return Err("storage administrative queue timeout".into());
                    }
                    thread::sleep(Duration::from_millis(1));
                }
                Err(_) => return Err("storage worker stopped".into()),
            }
        }
    }
    pub(crate) fn ack(&self, ids: &[EventId]) -> Result<(), JournalError> {
        let (tx, rx) = mpsc::channel();
        self.send(Admin::Ack(ids.to_vec(), tx))?;
        rx.recv_timeout(Duration::from_secs(5))?
    }
    pub(crate) fn insert(&self, event: EventEnvelope) -> Result<(), JournalError> {
        let (tx, rx) = mpsc::channel();
        self.send(Admin::Insert(Box::new(event), tx))?;
        rx.recv_timeout(Duration::from_secs(5))?
    }
}
pub(crate) fn read_connection(path: &Path) -> Result<Connection, JournalError> {
    let db = Connection::open_with_flags(
        path,
        rusqlite::OpenFlags::SQLITE_OPEN_READ_ONLY | rusqlite::OpenFlags::SQLITE_OPEN_NO_MUTEX,
    )?;
    db.busy_timeout(Duration::from_secs(2))?;
    Ok(db)
}
fn initialize(
    path: &Path,
    requested: Option<&JournalConfig>,
) -> Result<(Connection, JournalConfig), JournalError> {
    let mut db = Connection::open(path)?;
    db.busy_timeout(Duration::from_secs(2))?;
    db.execute_batch(
        "PRAGMA journal_mode=WAL;PRAGMA synchronous=FULL;PRAGMA journal_size_limit=1048576;",
    )?;
    let tx = db.transaction()?;
    tx.execute_batch("CREATE TABLE IF NOT EXISTS events(rowid INTEGER PRIMARY KEY,producer TEXT NOT NULL,boot TEXT NOT NULL,seq INTEGER NOT NULL,payload TEXT NOT NULL,bytes INTEGER NOT NULL,stored_ns INTEGER NOT NULL,ack INTEGER NOT NULL DEFAULT 0,UNIQUE(producer,boot,seq));
 CREATE INDEX IF NOT EXISTS events_pending ON events(ack,rowid);
 CREATE INDEX IF NOT EXISTS events_retention ON events(stored_ns);
 CREATE INDEX IF NOT EXISTS events_ack_retention ON events(ack,stored_ns);
 CREATE INDEX IF NOT EXISTS events_eviction ON events(ack DESC,rowid);
 CREATE TABLE IF NOT EXISTS storage_meta(id INTEGER PRIMARY KEY CHECK(id=1),logical_bytes INTEGER NOT NULL,max_bytes INTEGER NOT NULL,retention_seconds INTEGER NOT NULL,queue_capacity INTEGER NOT NULL);")?;
    let saved = tx
        .query_row(
            "SELECT max_bytes,retention_seconds,queue_capacity FROM storage_meta WHERE id=1",
            [],
            |r| {
                Ok((
                    r.get::<_, u64>(0)?,
                    r.get::<_, u64>(1)?,
                    r.get::<_, usize>(2)?,
                ))
            },
        )
        .optional()?;
    let mut policy = requested.cloned().unwrap_or_default();
    if requested.is_none() {
        if let Some((bytes, retention, capacity)) = saved {
            policy.max_bytes = bytes;
            policy.retention_seconds = retention;
            policy.queue_capacity = capacity;
        }
    }
    if saved.is_none() {
        tx.execute(
            "INSERT INTO storage_meta SELECT 1,COALESCE(SUM(bytes),0),?,?,? FROM events",
            params![
                policy.max_bytes,
                policy.retention_seconds,
                policy.queue_capacity
            ],
        )?;
    } else if requested.is_some() {
        tx.execute(
            "UPDATE storage_meta SET max_bytes=?,retention_seconds=?,queue_capacity=? WHERE id=1",
            params![
                policy.max_bytes,
                policy.retention_seconds,
                policy.queue_capacity
            ],
        )?;
    }
    tx.execute_batch("CREATE TRIGGER IF NOT EXISTS events_bytes_insert AFTER INSERT ON events BEGIN UPDATE storage_meta SET logical_bytes=logical_bytes+NEW.bytes WHERE id=1;END;
 CREATE TRIGGER IF NOT EXISTS events_bytes_delete AFTER DELETE ON events BEGIN UPDATE storage_meta SET logical_bytes=logical_bytes-OLD.bytes WHERE id=1;END;
 CREATE TRIGGER IF NOT EXISTS events_bytes_update AFTER UPDATE OF bytes ON events BEGIN UPDATE storage_meta SET logical_bytes=logical_bytes+NEW.bytes-OLD.bytes WHERE id=1;END;")?;
    tx.commit()?;
    Ok((db, policy))
}
fn insert(db: &Connection, e: &EventEnvelope) -> Result<(), JournalError> {
    let payload = serde_json::to_string(e)?;
    db.execute("INSERT OR IGNORE INTO events(producer,boot,seq,payload,bytes,stored_ns) VALUES(?,?,?,?,?,?)",params![e.id.producer_id,e.id.boot_id,e.id.sequence,payload,payload.len(),wall_time_ns()])?;
    Ok(())
}
fn maintain(
    db: &Connection,
    policy: &JournalConfig,
    source: &EventEnvelope,
) -> Result<u64, JournalError> {
    let cutoff =
        wall_time_ns().saturating_sub(policy.retention_seconds.saturating_mul(1_000_000_000));
    db.execute("DELETE FROM events WHERE rowid IN (SELECT rowid FROM events WHERE ack=1 AND stored_ns<? ORDER BY stored_ns LIMIT 64)", [cutoff])?;
    let expired: u64 = db.query_row(
        "SELECT COUNT(*) FROM (SELECT rowid FROM events WHERE ack=0 AND stored_ns<? ORDER BY stored_ns LIMIT 64)",
        [cutoff],
        |r| r.get(0),
    )?;
    db.execute("DELETE FROM events WHERE rowid IN (SELECT rowid FROM events WHERE ack=0 AND stored_ns<? ORDER BY stored_ns LIMIT 64)", [cutoff])?;
    let mut lost = expired;
    let mut removed_rows = 0;
    for _ in 0..64 {
        let total: u64 = db.query_row(
            "SELECT logical_bytes FROM storage_meta WHERE id=1",
            [],
            |r| r.get(0),
        )?;
        if total <= policy.max_bytes {
            break;
        }
        let (row, ack): (i64, bool) = db.query_row(
            "SELECT rowid,ack FROM events ORDER BY ack DESC,rowid LIMIT 1",
            [],
            |r| Ok((r.get(0)?, r.get(1)?)),
        )?;
        db.execute("DELETE FROM events WHERE rowid=?", [row])?;
        removed_rows += 1;
        if !ack {
            lost += 1;
        }
    }
    if lost > 0 {
        let mut gap = source.clone();
        gap.id.boot_id = uuid::Uuid::new_v4().to_string();
        gap.id.sequence = 0;
        gap.input = EventInput::new(
            &source.input.rover_id,
            "observability",
            "observability.gap",
            serde_json::json!({"evicted_events":lost,"reason":"retention_or_quota"}),
        );
        insert(db, &gap)?;
    }
    Ok(removed_rows)
}
fn persist(
    db: &mut Connection,
    policy: &JournalConfig,
    events: &[EventEnvelope],
) -> Result<(), JournalError> {
    let tx = db.transaction()?;
    for event in events {
        insert(&tx, event)?;
    }
    if let Some(last) = events.last() {
        maintain(&tx, policy, last)?;
    }
    tx.commit()?;
    Ok(())
}
fn admin(
    db: &mut Connection,
    policy: &JournalConfig,
    command: Admin,
    clients: &mut Vec<Arc<Counters>>,
) {
    match command {
        Admin::Register(client) => clients.push(client),
        Admin::Ack(ids, reply) => {
            let result = (|| {
                let tx = db.transaction()?;
                for id in ids {
                    tx.execute(
                        "UPDATE events SET ack=1 WHERE producer=? AND boot=? AND seq=?",
                        params![id.producer_id, id.boot_id, id.sequence],
                    )?;
                }
                tx.commit()?;
                Ok(())
            })();
            let _ = reply.send(result);
        }
        Admin::Insert(event, reply) => {
            let _ = reply.send(persist(db, policy, &[*event]));
        }
    }
}
fn worker(
    mut db: Connection,
    policy: JournalConfig,
    rx: mpsc::Receiver<Work>,
    commands: mpsc::Receiver<Admin>,
    owner: Arc<StorageHandle>,
) {
    use std::sync::atomic::Ordering;
    let mut diagnostics = crate::diagnostics::Diagnostics::new(&policy.path, "journal");
    let mut clients: Vec<Arc<Counters>> = Vec::new();
    let mut next_checkpoint = Instant::now();
    diagnostics.record("journal.started",serde_json::json!({"shared_writer":true,"queue_capacity":policy.queue_capacity,"max_bytes":policy.max_bytes}));
    loop {
        // One administrative mutation per event batch prevents either side starving the other.
        if let Ok(command) = commands.try_recv() {
            admin(&mut db, &policy, command, &mut clients);
        }
        let checkpoint = Instant::now() >= next_checkpoint;
        if checkpoint {
            next_checkpoint = Instant::now() + Duration::from_secs(5);
            for c in &clients {
                diagnostics.record("journal.checkpoint",serde_json::json!({"boot_id":c.boot,"queued":c.queued.load(Ordering::Relaxed),"dropped":c.dropped.load(Ordering::Relaxed),"persisted":c.persisted.load(Ordering::Relaxed),"failures":c.failures.load(Ordering::Relaxed)}));
            }
        }
        for c in &clients {
            let lost = c.pending_gap.swap(0, Ordering::AcqRel);
            if lost > 0 {
                let e = c.gap(lost);
                if persist(&mut db, &policy, &[e]).is_err() {
                    c.pending_gap.fetch_add(lost, Ordering::Relaxed);
                }
            }
        }
        clients.retain(|c| Arc::strong_count(c) > 1 || c.pending_gap.load(Ordering::Relaxed) > 0);
        // Reclaim quota in bounded commits, serving administrative acknowledgments
        // between passes. Never commit another event batch while already over quota.
        let over_quota = db
            .query_row(
                "SELECT logical_bytes>? FROM storage_meta WHERE id=1",
                [policy.max_bytes],
                |r| r.get::<_, bool>(0),
            )
            .unwrap_or(false);
        if over_quota {
            let start = Instant::now();
            let result = (|| -> Result<u64, JournalError> {
                let raw: String = db.query_row(
                    "SELECT payload FROM events ORDER BY rowid LIMIT 1",
                    [],
                    |r| r.get(0),
                )?;
                let source: EventEnvelope = serde_json::from_str(&raw)?;
                let tx = db.transaction()?;
                let removed = maintain(&tx, &policy, &source)?;
                tx.commit()?;
                Ok(removed)
            })();
            match result {
                Ok(removed)=>diagnostics.record("quota.pass",serde_json::json!({"removed_rows":removed,"duration_ms":start.elapsed().as_millis() as u64})),
                Err(e)=>{diagnostics.record("quota.failed",crate::diagnostics::error_fields(e.as_ref()));thread::sleep(Duration::from_millis(20));}
            }
            continue;
        }
        let first = match rx.recv_timeout(Duration::from_millis(20)) {
            Ok(m) => m,
            Err(mpsc::RecvTimeoutError::Timeout) => {
                // Keep registry ownership while draining. Retirement and reopen are
                // serialized so callers cannot acquire a handle to an exiting worker.
                let mut registry = WRITERS.get().unwrap().lock().unwrap();
                if Arc::strong_count(&owner) == 1 {
                    registry.remove(&owner.path);
                    break;
                }
                continue;
            }
            Err(_) => break,
        };
        let mut batch = Vec::new();
        let mut barrier = None;
        let mut message = Some(first);
        while let Some(m) = message.take() {
            match m {
                Work::Event(e, at, c) => {
                    c.queued.fetch_sub(1, Ordering::Relaxed);
                    batch.push((e, at, c));
                }
                Work::Flush(reply) => {
                    barrier = Some(reply);
                    break;
                }
            }
            if batch.len() >= 64 {
                break;
            }
            message = rx.try_recv().ok();
        }
        if !batch.is_empty() {
            let events: Vec<_> = batch.iter().map(|(e, _, _)| (**e).clone()).collect();
            let queue_wait_ms = batch[0].1.elapsed().as_millis() as u64;
            let start = Instant::now();
            if checkpoint {
                diagnostics.record("persist.started",serde_json::json!({"event_kind":events[0].input.kind,"batch_size":events.len()}));
            }
            let result = persist(&mut db, &policy, &events);
            let mut fields = serde_json::json!({"boot_id":events[0].id.boot_id,"event_kind":events[0].input.kind,"batch_size":batch.len(),"queue_wait_ms":queue_wait_ms,"duration_ms":start.elapsed().as_millis() as u64,"queued":batch[0].2.queued.load(Ordering::Relaxed)});
            if let Err(error) = &result {
                fields.as_object_mut().unwrap().extend(
                    crate::diagnostics::error_fields(error.as_ref())
                        .as_object()
                        .unwrap()
                        .clone(),
                );
                diagnostics.record("persist.failed", fields);
            } else if checkpoint {
                diagnostics.record("journal.progress", fields);
            }
            for (_, _, c) in batch {
                if result.is_ok() {
                    c.persisted.fetch_add(1, Ordering::Relaxed);
                } else {
                    c.failures.fetch_add(1, Ordering::Relaxed);
                    c.dropped.fetch_add(1, Ordering::Relaxed);
                    c.pending_gap.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
        if let Some(reply) = barrier {
            let _ = reply.send(());
        }
    }
}
