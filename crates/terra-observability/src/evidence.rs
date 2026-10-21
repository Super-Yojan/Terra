use crate::{EventInput, Journal, JournalError};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::{
    path::PathBuf,
    sync::{
        Arc,
        mpsc::{self, SyncSender},
    },
    time::{Duration, Instant},
};
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AttachmentInput {
    pub name: String,
    pub data: Vec<u8>,
    pub encoding: String,
    pub dimensions: Vec<u32>,
    pub units: String,
}
#[derive(Clone, Debug)]
pub struct EvidenceBundle {
    pub event: EventInput,
    pub manifest: Value,
    pub attachments: Vec<AttachmentInput>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Attachment {
    pub name: String,
    pub sha256: String,
    pub encoding: String,
    pub dimensions: Vec<u32>,
    pub units: String,
    pub byte_length: usize,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CaptureOutcome {
    Queued,
    Dropped,
    Invalid,
}
enum Message {
    Capture(Box<EvidenceBundle>),
    Flush(mpsc::Sender<()>),
}
pub struct EvidenceWriter {
    sender: SyncSender<Message>,
    journal: Arc<Journal>,
}
impl EvidenceWriter {
    pub fn open(
        directory: PathBuf,
        journal: Arc<Journal>,
        max_bytes: u64,
    ) -> Result<Self, JournalError> {
        std::fs::create_dir_all(&directory)?;
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            std::fs::set_permissions(&directory, std::fs::Permissions::from_mode(0o700))?;
        }
        let (tx, rx) = mpsc::sync_channel::<Message>(2);
        let log = journal.clone();
        std::thread::Builder::new()
            .name("mapping-evidence".into())
            .spawn(move || {
                while let Ok(msg) = rx.recv() {
                    match msg {
                        Message::Flush(done) => {
                            let _ = done.send(());
                        }
                        Message::Capture(bundle) => {
                            let mut bundle = *bundle;
                            if bundle.attachments.iter().any(|a| a.name == "rgb")
                                && bundle.manifest["rgb_exposure_ns"]
                                    != bundle.manifest["exposure_ns"]
                            {
                                bundle.attachments.retain(|a| a.name != "rgb");
                                bundle.manifest["rgb_missing_reason"] =
                                    Value::String("exposure_mismatch".into());
                            }
                            if !bundle.attachments.iter().any(|a| a.name == "rgb")
                                && bundle.manifest.get("rgb_missing_reason").is_none()
                                && bundle.manifest["evidence_type"] != "occupancy"
                            {
                                bundle.manifest["rgb_missing_reason"] =
                                    Value::String("unavailable".into());
                            }
                            let mut references = Vec::new();
                            let mut failed = false;
                            for a in bundle.attachments {
                                let hash = format!("{:x}", Sha256::digest(&a.data));
                                let path = directory.join(&hash);
                                if !path.exists() {
                                    if a.data.len() as u64 > max_bytes {
                                        failed = true;
                                        continue;
                                    }
                                    let mut files = std::fs::read_dir(&directory)
                                        .ok()
                                        .into_iter()
                                        .flatten()
                                        .filter_map(Result::ok)
                                        .filter_map(|f| {
                                            let m = f.metadata().ok()?;
                                            Some((m.modified().ok()?, m.len(), f.path()))
                                        })
                                        .collect::<Vec<_>>();
                                    files.sort_by_key(|f| f.0);
                                    let mut bytes: u64 = files.iter().map(|f| f.1).sum();
                                    let mut removed = 0;
                                    for (_, size, old) in files {
                                        if bytes + a.data.len() as u64 <= max_bytes {
                                            break;
                                        }
                                        if std::fs::remove_file(&old).is_ok() {
                                            bytes = bytes.saturating_sub(size);
                                            removed += 1;
                                        }
                                    }
                                    if removed > 0 {
                                        gap(&log, &bundle.event, "attachment_quota", removed);
                                    }
                                    let temp = directory.join(format!("{hash}.tmp"));
                                    if durable_attachment(&temp, &path, &a.data).is_err() {
                                        failed = true;
                                        let _ = std::fs::remove_file(temp);
                                        continue;
                                    }
                                }
                                references.push(Attachment {
                                    name: a.name,
                                    sha256: hash,
                                    encoding: a.encoding,
                                    dimensions: a.dimensions,
                                    units: a.units,
                                    byte_length: a.data.len(),
                                });
                            }
                            bundle.manifest["id"] = Value::String(uuid::Uuid::new_v4().to_string());
                            bundle.manifest["attachments"] =
                                serde_json::to_value(references).unwrap_or(Value::Null);
                            bundle.manifest["attachments_incomplete"] = Value::Bool(failed);
                            if failed {
                                gap(&log, &bundle.event, "attachment_storage", 1);
                            }
                            bundle.event.attributes = bundle.manifest;
                            log.try_record(bundle.event);
                        }
                    }
                }
            })?;
        Ok(Self {
            sender: tx,
            journal,
        })
    }
    pub fn try_capture(&self, bundle: EvidenceBundle) -> CaptureOutcome {
        if !bundle.manifest.is_object()
            || (bundle.manifest["exposure_ns"].as_u64().is_none()
                && !(bundle.manifest["time_basis"] == "receipt_only"
                    && bundle.manifest["receipt_ns"].as_u64().is_some()))
            || bundle.manifest["frame_id"]
                .as_str()
                .is_none_or(|s| s.is_empty() || s.len() > 256)
            || bundle.attachments.len() > 8
            || bundle.attachments.iter().any(|a| {
                a.data.len() > 32 * 1024 * 1024 || a.dimensions.len() > 3 || a.name.len() > 64
            })
            || bundle
                .attachments
                .iter()
                .map(|a| a.data.len())
                .sum::<usize>()
                > 48 * 1024 * 1024
        {
            return CaptureOutcome::Invalid;
        }
        match self.sender.try_send(Message::Capture(Box::new(bundle))) {
            Ok(()) => CaptureOutcome::Queued,
            Err(_) => {
                self.journal.try_record(EventInput::new(
                    "_producer",
                    "observability",
                    "observability.gap",
                    serde_json::json!({"reason":"capture_queue_full","dropped_bundles":1}),
                ));
                CaptureOutcome::Dropped
            }
        }
    }
    pub fn flush(&self, timeout: Duration) -> Result<(), JournalError> {
        let start = Instant::now();
        let (tx, rx) = mpsc::channel();
        let mut msg = Message::Flush(tx);
        loop {
            match self.sender.try_send(msg) {
                Ok(()) => break,
                Err(mpsc::TrySendError::Full(m)) => {
                    msg = m;
                    if start.elapsed() >= timeout {
                        return Err("evidence flush queue timeout".into());
                    }
                    std::thread::sleep(Duration::from_millis(1));
                }
                Err(_) => return Err("evidence worker stopped".into()),
            }
        }
        rx.recv_timeout(timeout.saturating_sub(start.elapsed()))?;
        Ok(())
    }
}
fn gap(journal: &Journal, event: &EventInput, reason: &str, count: u64) {
    let mut e = event.clone();
    e.kind = "observability.gap".into();
    e.component = "observability".into();
    e.attributes = serde_json::json!({"reason":reason,"attachments":count});
    journal.try_record(e);
}

fn durable_attachment(
    temp: &std::path::Path,
    path: &std::path::Path,
    bytes: &[u8],
) -> std::io::Result<()> {
    use std::io::Write;
    let mut file = std::fs::File::create(temp)?;
    file.write_all(bytes)?;
    file.sync_all()?;
    std::fs::rename(temp, path)?;
    #[cfg(unix)]
    if let Some(parent) = path.parent() {
        std::fs::File::open(parent)?.sync_all()?;
    }
    Ok(())
}
