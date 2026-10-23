use std::path::PathBuf;
#[derive(Clone, Debug)]
pub struct JournalConfig {
    pub path: PathBuf,
    pub producer_id: String,
    pub queue_capacity: usize,
    /// Logical event bytes, excluding SQLite page/index overhead.
    pub max_bytes: u64,
    pub retention_seconds: u64,
}
impl Default for JournalConfig {
    fn default() -> Self {
        Self {
            path: "observability.db".into(),
            producer_id: "unknown".into(),
            queue_capacity: 4096,
            max_bytes: ObservationSettings::default().journal_max_bytes,
            retention_seconds: ObservationSettings::default().retention_seconds,
        }
    }
}

/// Source recording policy. These affect diagnostics only, never motion or sensor safety.
#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ObservationSettings {
    pub journal_max_bytes: u64,
    pub attachment_max_bytes: u64,
    pub retention_seconds: u64,
    pub control_sample_seconds: f64,
    pub map_sample_seconds: f64,
    pub evidence_sample_seconds: f64,
}
impl Default for ObservationSettings {
    fn default() -> Self {
        Self {
            journal_max_bytes: 256 * 1024 * 1024,
            attachment_max_bytes: 512 * 1024 * 1024,
            retention_seconds: 7 * 86400,
            control_sample_seconds: 0.1,
            map_sample_seconds: 0.5,
            evidence_sample_seconds: 1.,
        }
    }
}
impl ObservationSettings {
    pub fn valid(&self) -> bool {
        self.journal_max_bytes >= 1024
            && self.attachment_max_bytes > 0
            && self.retention_seconds > 0
            && [
                self.control_sample_seconds,
                self.map_sample_seconds,
                self.evidence_sample_seconds,
            ]
            .iter()
            .all(|s| s.is_finite() && *s >= 0.01 && *s <= 3600.)
    }
    pub fn from_environment() -> Result<Self, crate::JournalError> {
        let settings = if let Ok(path) = std::env::var("TERRA_OBSERVABILITY_SETTINGS") {
            if std::fs::metadata(&path)?.len() > 65536 {
                return Err("oversized observability settings".into());
            }
            serde_json::from_slice(&std::fs::read(path)?)?
        } else {
            Self::default()
        };
        if !settings.valid() {
            return Err("invalid observability settings".into());
        }
        Ok(settings)
    }
}
