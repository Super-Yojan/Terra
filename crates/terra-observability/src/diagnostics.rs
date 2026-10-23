//! Independent bounded diagnostics for the logging workers themselves.
//! Call only on administrative/background workers, never on the control tick.
use serde_json::{Value, json};
use std::{
    fs::OpenOptions,
    io::Write,
    path::{Path, PathBuf},
    sync::Mutex,
};

static WRITE_LOCK: Mutex<()> = Mutex::new(());
const MAX_BYTES: u64 = 1024 * 1024;

pub(crate) struct Diagnostics {
    path: PathBuf,
}

/// A separate, bounded diagnostic queue. Never waits for SQLite, disk or network.
/// Its worker writes at most two 1 MiB JSONL segments beside the supplied database.
pub struct DiagnosticRecorder {
    sender: std::sync::mpsc::SyncSender<(String, Value)>,
    dropped: std::sync::Arc<std::sync::atomic::AtomicU64>,
}
impl DiagnosticRecorder {
    /// Administrative setup; `component` becomes a file suffix, not a user-entered path.
    pub fn open(database: &Path, component: &str) -> Result<Self, crate::JournalError> {
        if !matches!(component, "router" | "phone") {
            return Err("invalid diagnostic component".into());
        }
        let mut log = Diagnostics::new(database, component);
        let (sender, receiver) = std::sync::mpsc::sync_channel::<(String, Value)>(32);
        let dropped = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(0));
        let losses = dropped.clone();
        std::thread::Builder::new()
            .name(format!("{component}-diagnostics"))
            .spawn(move || {
                for (kind, mut attrs) in receiver {
                    attrs["diagnostic_records_dropped"] =
                        losses.load(std::sync::atomic::Ordering::Relaxed).into();
                    log.record(&kind, attrs);
                }
            })?;
        Ok(Self { sender, dropped })
    }
    pub fn try_record(&self, kind: &str, mut attributes: Value) {
        if kind.len() > 100 || !attributes.is_object() {
            return;
        }
        if serde_json::to_vec(&attributes).map_or(true, |bytes| bytes.len() > 3500) {
            return;
        }
        attributes["observed_wall_time_ns"] = crate::wall_time_ns().into();
        if self.sender.try_send((kind.into(), attributes)).is_err() {
            self.dropped
                .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        }
    }
}
impl Diagnostics {
    pub(crate) fn new(database: &Path, component: &str) -> Self {
        Self {
            path: database.with_extension(format!("{component}-diagnostics.jsonl")),
        }
    }
    pub(crate) fn record(&mut self, kind: &str, mut attributes: Value) {
        crate::redact(&mut attributes);
        let row = json!({"wall_time_ns":crate::wall_time_ns(), "pid":std::process::id(), "kind":kind, "attributes":attributes});
        let Ok(mut bytes) = serde_json::to_vec(&row) else {
            return;
        };
        // Never copy event payloads, images, endpoints or credentials into this file.
        if bytes.len() > 4095 {
            return;
        }
        bytes.push(b'\n');
        let Ok(_lock) = WRITE_LOCK.lock() else { return };
        if std::fs::metadata(&self.path).is_ok_and(|m| m.len() + bytes.len() as u64 > MAX_BYTES) {
            // One previous segment only. Diagnostic failure never affects control or ingestion.
            if std::fs::rename(&self.path, self.path.with_extension("jsonl.1")).is_err() {
                return;
            }
        }
        let mut options = OpenOptions::new();
        options.create(true).append(true);
        #[cfg(unix)]
        {
            use std::os::unix::fs::OpenOptionsExt;
            options.mode(0o600);
        }
        if let Ok(mut file) = options.open(&self.path) {
            let _ = file.write_all(&bytes);
        }
    }
}

/// Structured error categories avoid putting URLs, credentials or arbitrary server replies in diagnostics.
pub(crate) fn error_fields(error: &(dyn std::error::Error + 'static)) -> Value {
    if let Some(error) = error.downcast_ref::<HttpStatusError>() {
        return json!({"error_class":"http_status","http_status":error.0});
    }
    if let Some(error) = error.downcast_ref::<StageError>() {
        let mut fields = error_fields(error.error.as_ref());
        fields["stage"] = error.stage.into();
        return fields;
    }
    if let Some(error) = error.downcast_ref::<rusqlite::Error>() {
        return match error {
            rusqlite::Error::SqliteFailure(code, _) => json!({"error_class":match code.code {
                rusqlite::ErrorCode::DatabaseBusy => "sqlite_busy",
                rusqlite::ErrorCode::DatabaseLocked => "sqlite_locked",
                _ => "sqlite_error"
            },"sqlite_extended_code":code.extended_code}),
            _ => json!({"error_class":"sqlite_error"}),
        };
    }
    if let Some(error) = error.downcast_ref::<reqwest::Error>() {
        return json!({"error_class":if error.is_timeout(){"http_timeout"}else if error.is_connect(){"http_connect"}else{"http_error"},"http_status":error.status().map(|s|s.as_u16())});
    }
    json!({"error_class":"operation_failed"})
}

#[derive(Debug)]
pub(crate) struct StageError {
    stage: &'static str,
    error: crate::JournalError,
}
impl StageError {
    pub(crate) fn wrap(stage: &'static str, error: crate::JournalError) -> crate::JournalError {
        Box::new(Self { stage, error })
    }
}
impl std::fmt::Display for StageError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}: {}", self.stage, self.error)
    }
}
impl std::error::Error for StageError {}

#[derive(Debug)]
pub(crate) struct HttpStatusError(pub(crate) u16);
impl std::fmt::Display for HttpStatusError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "collector status {}", self.0)
    }
}
impl std::error::Error for HttpStatusError {}
