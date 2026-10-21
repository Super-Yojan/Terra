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
            max_bytes: 256 * 1024 * 1024,
            retention_seconds: 7 * 86400,
        }
    }
}
