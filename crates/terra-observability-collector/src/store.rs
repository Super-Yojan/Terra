use crate::Scope;
use rusqlite::{Connection, params};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{path::PathBuf, time::Duration};
use terra_observability::{EventEnvelope, EventId, EventInput, JournalError, wall_time_ns};
#[derive(Clone)]
pub struct StoreConfig {
    pub path: PathBuf,
    pub max_bytes: u64,
    pub retention_seconds: u64,
    pub attachment_max_bytes: u64,
    pub export_max_bytes: u64,
}
impl Default for StoreConfig {
    fn default() -> Self {
        Self {
            path: "collector.db".into(),
            max_bytes: 10 * 1024 * 1024 * 1024,
            retention_seconds: 30 * 86400,
            attachment_max_bytes: 5 * 1024 * 1024 * 1024,
            export_max_bytes: 2 * 1024 * 1024 * 1024,
        }
    }
}
#[derive(Clone)]
pub struct Store {
    pub config: StoreConfig,
    pub(crate) attachment_lock: std::sync::Arc<std::sync::Mutex<()>>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StoredEvent {
    pub cursor: i64,
    pub received_ns: u64,
    pub event: EventEnvelope,
}
impl Store {
    pub fn open(config: StoreConfig) -> Result<Self, JournalError> {
        if config.max_bytes < 1024
            || config.retention_seconds == 0
            || config.attachment_max_bytes == 0
            || config.export_max_bytes == 0
        {
            return Err("invalid store limits".into());
        }
        if let Some(p) = config.path.parent() {
            std::fs::create_dir_all(p)?;
        }
        let store = Self {
            config,
            attachment_lock: Default::default(),
        };
        let db = store.db()?;
        db.execute_batch("PRAGMA journal_mode=WAL; PRAGMA synchronous=FULL; CREATE TABLE IF NOT EXISTS observations(cursor INTEGER PRIMARY KEY AUTOINCREMENT,producer TEXT,boot TEXT,seq INTEGER,rover TEXT,session TEXT,received_ns INTEGER,payload TEXT,hash TEXT,bytes INTEGER, UNIQUE(producer,boot,seq)); CREATE INDEX IF NOT EXISTS observations_scope ON observations(rover,session,cursor); CREATE TABLE IF NOT EXISTS retention_gaps(producer TEXT,rover TEXT,at_ns INTEGER,count INTEGER,attachments INTEGER NOT NULL DEFAULT 0,PRIMARY KEY(producer,rover));")?;
        Ok(store)
    }
    pub fn db(&self) -> Result<Connection, JournalError> {
        let db = Connection::open(&self.config.path)?;
        db.busy_timeout(Duration::from_secs(5))?;
        Ok(db)
    }
    pub fn ingest(
        &self,
        scope: &Scope,
        events: &[EventEnvelope],
    ) -> Result<Vec<EventId>, JournalError> {
        if scope.read || events.is_empty() || events.len() > 256 {
            return Err("invalid batch".into());
        }
        for e in events {
            if e.schema_version != 1
                || e.id.producer_id != scope.producer_id
                || e.id.boot_id.is_empty()
                || e.id.boot_id.len() > 256
                || e.id.sequence > i64::MAX as u64
                || (!scope.rovers.contains(&e.input.rover_id) && e.input.rover_id != "_producer")
                || !e.input.validate()
                || contains_secret(&e.input.attributes)
            {
                return Err("invalid or unauthorized event".into());
            }
        }
        let db = self.db()?;
        let tx = db.unchecked_transaction()?;
        let mut ack = Vec::new();
        for e in events {
            let payload = serde_json::to_string(e)?;
            if payload.len() > 128 * 1024 {
                return Err("event too large".into());
            }
            let hash = format!("{:x}", Sha256::digest(payload.as_bytes()));
            let existing = tx.query_row(
                "SELECT hash FROM observations WHERE producer=? AND boot=? AND seq=?",
                params![e.id.producer_id, e.id.boot_id, e.id.sequence],
                |r| r.get::<_, String>(0),
            );
            match existing {
                Ok(old) => {
                    if old != hash {
                        return Err("conflicting event identity".into());
                    }
                }
                Err(rusqlite::Error::QueryReturnedNoRows) => {
                    tx.execute("INSERT INTO observations(producer,boot,seq,rover,session,received_ns,payload,hash,bytes) VALUES(?,?,?,?,?,?,?,?,?)",params![e.id.producer_id,e.id.boot_id,e.id.sequence,e.input.rover_id,e.input.session_id,wall_time_ns(),payload,hash,payload.len()])?;
                }
                Err(e) => return Err(e.into()),
            }
            ack.push(e.id.clone());
        }
        tx.commit()?;
        self.prune(
            wall_time_ns()
                .saturating_sub(self.config.retention_seconds.saturating_mul(1_000_000_000)),
        )?;
        Ok(ack)
    }
    pub fn events(
        &self,
        scope: &Scope,
        session: Option<&str>,
        cursor: i64,
        limit: usize,
    ) -> Result<Vec<StoredEvent>, JournalError> {
        let db = self.db()?;
        let mut q=db.prepare("SELECT cursor,received_ns,payload,rover,producer FROM observations WHERE cursor>? AND (? IS NULL OR session=?) ORDER BY cursor")?;
        let rows = q.query_map(params![cursor, session, session], |r| {
            Ok((
                r.get::<_, i64>(0)?,
                r.get::<_, u64>(1)?,
                r.get::<_, String>(2)?,
                r.get::<_, String>(3)?,
                r.get::<_, String>(4)?,
            ))
        })?;
        let mut result = Vec::new();
        for row in rows {
            let (c, t, p, r, producer) = row?;
            if scope.rovers.contains(&r) || (r == "_producer" && scope.producer_id == producer) {
                result.push(StoredEvent {
                    cursor: c,
                    received_ns: t,
                    event: serde_json::from_str(&p)?,
                });
                if result.len() >= limit.clamp(1, 1000) {
                    break;
                }
            }
        }
        if cursor == 0 {
            let mut gaps =
                db.prepare("SELECT producer,rover,at_ns,count,attachments FROM retention_gaps")?;
            for gap in gaps.query_map([], |r| {
                Ok((
                    r.get::<_, String>(0)?,
                    r.get::<_, String>(1)?,
                    r.get::<_, u64>(2)?,
                    r.get::<_, u64>(3)?,
                    r.get::<_, u64>(4)?,
                ))
            })? {
                let (producer, rover, at, count, attachments) = gap?;
                if scope.rovers.contains(&rover)
                    || (rover == "_producer" && scope.producer_id == producer)
                {
                    result.push(StoredEvent {
                        cursor: 0,
                        received_ns: at,
                        event: EventEnvelope {
                            schema_version: 1,
                            id: EventId {
                                producer_id: producer,
                                boot_id: format!("collector-retention-{count}-{attachments}"),
                                sequence: at,
                            },
                            input: {let mut input=EventInput::new(&rover,"collector","observability.gap",serde_json::json!({"evicted_events":count,"evicted_attachments":attachments,"at_ns":at}));input.wall_time_ns=at;input},
                        },
                    });
                }
            }
        }
        Ok(result)
    }
    pub fn sessions(&self, scope: &Scope) -> Result<serde_json::Value, JournalError> {
        let db = self.db()?;
        let mut q=db.prepare("SELECT rover,session,COUNT(*),MIN(received_ns),MAX(received_ns) FROM observations GROUP BY rover,session")?;
        let mut result = Vec::new();
        for row in q.query_map([], |r| {
            Ok((
                r.get::<_, String>(0)?,
                r.get::<_, Option<String>>(1)?,
                r.get::<_, u64>(2)?,
                r.get::<_, u64>(3)?,
                r.get::<_, u64>(4)?,
            ))
        })? {
            let (rover, session, count, first, last) = row?;
            if scope.rovers.contains(&rover) {
                result.push(serde_json::json!({"rover_id":rover,"session_id":session,"count":count,"first_received_ns":first,"last_received_ns":last}));
            }
        }
        Ok(result.into())
    }
    pub fn prune(&self, cutoff: u64) -> Result<(), JournalError> {
        let db = self.db()?;
        let tx = db.unchecked_transaction()?;
        let mut total: u64 =
            tx.query_row("SELECT COALESCE(SUM(bytes),0) FROM observations", [], |r| {
                r.get(0)
            })?;
        loop {
            let oldest=tx.query_row("SELECT cursor,producer,rover,received_ns,bytes FROM observations ORDER BY cursor LIMIT 1",[],|r|Ok((r.get::<_,i64>(0)?,r.get::<_,String>(1)?,r.get::<_,String>(2)?,r.get::<_,u64>(3)?,r.get::<_,u64>(4)?)));
            let Ok((id, producer, rover, at, bytes)) = oldest else {
                break;
            };
            if at >= cutoff && total <= self.config.max_bytes {
                break;
            }
            tx.execute("DELETE FROM observations WHERE cursor=?", [id])?;
            tx.execute(
                "INSERT INTO retention_gaps(producer,rover,at_ns,count) VALUES(?,?,?,1) ON CONFLICT(producer,rover) DO UPDATE SET count=count+1,at_ns=MAX(at_ns,excluded.at_ns)",
                params![producer, rover, at],
            )?;
            total = total.saturating_sub(bytes);
        }
        tx.execute("DELETE FROM retention_gaps WHERE rowid NOT IN (SELECT MAX(rowid) FROM retention_gaps GROUP BY producer,rover)",[])?;
        tx.commit()?;
        Ok(())
    }
}
fn contains_secret(value: &serde_json::Value) -> bool {
    match value {
        serde_json::Value::Object(m) => m.iter().any(|(key, v)| {
            let k = key.to_lowercase();
            [
                "password",
                "authorization",
                "credential",
                "private_key",
                "secret",
                "access_token",
                "refresh_token",
            ]
            .iter()
            .any(|s| k.contains(s))
                || contains_secret(v)
        }),
        serde_json::Value::Array(a) => a.iter().any(contains_secret),
        _ => false,
    }
}
