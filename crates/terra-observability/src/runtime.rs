use crate::{EventInput, Journal, JournalConfig, JournalError, PRODUCER_ROVER};
use serde_json::Value;
use std::{
    path::Path,
    sync::OnceLock,
    time::{Duration, Instant},
};
pub struct RuntimeObserver {
    journal: std::sync::Arc<Journal>,
    start: Instant,
    session: String,
    _uploader: Option<crate::UploadHandle>,
    settings: crate::ObservationSettings,
    path: std::path::PathBuf,
}
impl RuntimeObserver {
    pub fn open(path: &Path, producer: &str) -> Result<Self, JournalError> {
        Self::open_with_settings(path, producer, crate::ObservationSettings::default())
    }
    pub fn open_with_settings(
        path: &Path,
        producer: &str,
        settings: crate::ObservationSettings,
    ) -> Result<Self, JournalError> {
        if !settings.valid() {
            return Err("invalid observability settings".into());
        }
        Ok(Self {
            journal: std::sync::Arc::new(Journal::open(JournalConfig {
                path: path.into(),
                producer_id: producer.into(),
                max_bytes: settings.journal_max_bytes,
                retention_seconds: settings.retention_seconds,
                ..Default::default()
            })?),
            start: Instant::now(),
            session: uuid::Uuid::new_v4().to_string(),
            _uploader: None,
            settings,
            path: path.into(),
        })
    }
    pub fn record(&self, rover: &str, component: &str, kind: &str, attributes: Value) {
        let mut e = EventInput::new(rover, component, kind, attributes);
        e.session_id = Some(self.session.clone());
        e.monotonic_ns = self.start.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        self.journal.try_record(e);
    }
    pub fn flush(&self) -> Result<(), JournalError> {
        self.journal.flush(Duration::from_secs(5))
    }
}
static OBSERVER: OnceLock<Option<RuntimeObserver>> = OnceLock::new();
/// Call on application startup, before entering control/message callbacks.
pub fn initialize_runtime(role: &str) {
    OBSERVER.get_or_init(|| {
        if std::env::var("TERRA_OBSERVABILITY_DISABLED").as_deref() == Ok("1") {
            return None;
        }
        let path = std::env::var("TERRA_OBSERVABILITY_DB")
            .map(std::path::PathBuf::from)
            .unwrap_or_else(|_| default_state_path(role));
        let identity_path = path.with_extension("producer");
        let producer = std::env::var("TERRA_OBSERVABILITY_PRODUCER")
            .ok()
            .or_else(|| std::fs::read_to_string(&identity_path).ok())
            .filter(|s| !s.is_empty())
            .unwrap_or_else(|| {
                let id = format!("{role}-{}", uuid::Uuid::new_v4());
                if let Some(p) = path.parent() {
                    let _ = std::fs::create_dir_all(p);
                }
                let _ = std::fs::write(&identity_path, &id);
                id
            });
        // Bad settings must not silently turn observation off; run on the
        // defaults and record why.
        let (settings, rejected) = match crate::ObservationSettings::from_environment() {
            Ok(settings) => (settings, None),
            Err(error) => (Default::default(), Some(error.to_string())),
        };
        let mut observer = RuntimeObserver::open_with_settings(&path, &producer, settings).ok()?;
        if let Some(reason) = rejected {
            observer.record(
                PRODUCER_ROVER,
                role,
                "observability.settings_rejected",
                serde_json::json!({"reason": reason}),
            );
        }
        let upload = match (
            std::env::var("TERRA_OBSERVABILITY_ENDPOINT"),
            std::env::var("TERRA_OBSERVABILITY_UPLOAD_TOKEN_FILE"),
        ) {
            (Ok(endpoint), Ok(token_file)) => Some(SavedUploadConfig {
                endpoint,
                token_file: token_file.into(),
            }),
            (Err(std::env::VarError::NotPresent), Err(std::env::VarError::NotPresent)) => {
                load_upload_config(&path.with_file_name("upload.json"))
                    .ok()
                    .flatten()
            }
            _ => None,
        };
        if let Some(config) = upload
            && let Ok(token) = std::fs::read_to_string(config.token_file)
            && let Ok(reader) = crate::JournalReader::open(&path)
        {
            observer._uploader = crate::UploadHandle::start(
                reader,
                crate::UploadConfig {
                    endpoint: config.endpoint,
                    token: token.trim().into(),
                    evidence_directory: Some(crate::evidence_directory(&path)),
                    clock_journal: Some(observer.journal.clone()),
                },
            )
            .ok();
        }
        Some(observer)
    });
    record_runtime(
        PRODUCER_ROVER,
        role,
        "lifecycle.start",
        serde_json::json!({"version":env!("CARGO_PKG_VERSION")}),
    );
}
/// Never initializes files on a control tick; no-op if startup initialization failed.
pub fn record_runtime(rover: &str, component: &str, kind: &str, attributes: Value) {
    if let Some(Some(o)) = OBSERVER.get() {
        o.record(rover, component, kind, attributes);
    }
}
/// Preserve source session and clock when projecting an existing structured event.
pub fn record_runtime_input(input: EventInput) {
    if let Some(Some(o)) = OBSERVER.get() {
        o.journal.try_record(input);
    }
}

pub fn runtime_journal() -> Option<std::sync::Arc<Journal>> {
    OBSERVER
        .get()
        .and_then(|o| o.as_ref().map(|o| o.journal.clone()))
}

pub fn runtime_settings() -> crate::ObservationSettings {
    OBSERVER
        .get()
        .and_then(|o| o.as_ref().map(|o| o.settings))
        .unwrap_or_default()
}

fn default_state_path(role: &str) -> std::path::PathBuf {
    let home = std::env::var_os("HOME")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(std::env::temp_dir);
    #[cfg(target_os = "macos")]
    let base = home.join("Library/Application Support/Terra/Observability");
    #[cfg(not(target_os = "macos"))]
    let base = std::env::var_os("XDG_STATE_HOME")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| home.join(".local/state"))
        .join("terra/observability");
    base.join(role).join("events.db")
}
pub fn runtime_evidence_directory() -> Option<std::path::PathBuf> {
    OBSERVER
        .get()
        .and_then(|o| o.as_ref().map(|o| crate::evidence_directory(&o.path)))
}

/// Private per-application upload settings, used when explicit environment
/// settings are absent. The token itself stays in a separate protected file.
#[derive(serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SavedUploadConfig {
    pub endpoint: String,
    pub token_file: std::path::PathBuf,
}
pub fn load_upload_config(path: &Path) -> Result<Option<SavedUploadConfig>, JournalError> {
    let metadata = match std::fs::metadata(path) {
        Ok(metadata) => metadata,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(error) => return Err(error.into()),
    };
    if metadata.len() > 65536 {
        return Err("oversized upload settings".into());
    }
    let mut config: SavedUploadConfig = serde_json::from_slice(&std::fs::read(path)?)?;
    if config.token_file.is_relative() {
        config.token_file = path
            .parent()
            .unwrap_or_else(|| Path::new("."))
            .join(config.token_file);
    }
    Ok(Some(config))
}
