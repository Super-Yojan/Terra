use crate::{Attachment, EventEnvelope, EventId, JournalError, JournalReader};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::io::Read;
use std::{
    path::{Path, PathBuf},
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicU64, Ordering},
    },
    time::Duration,
};
#[derive(Debug)]
struct AttachmentRejected;
impl std::fmt::Display for AttachmentRejected {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("attachment rejected by collector")
    }
}
impl std::error::Error for AttachmentRejected {}

pub trait UploadTransport {
    fn clock_sample(&mut self) -> Result<Option<u64>, JournalError> {
        Ok(None)
    }
    fn send(&mut self, events: &[EventEnvelope]) -> Result<Vec<EventId>, JournalError>;
    fn put_attachment(&mut self, rover: &str, hash: &str, bytes: &[u8])
    -> Result<(), JournalError>;
}
pub struct HttpTransport {
    client: reqwest::blocking::Client,
    endpoint: String,
    token: String,
    last_batch_size: usize,
}
impl HttpTransport {
    pub fn new(endpoint: &str, token: &str) -> Result<Self, JournalError> {
        validate_endpoint(endpoint, token)?;
        let client = reqwest::blocking::Client::builder()
            .timeout(Duration::from_secs(15))
            .connect_timeout(Duration::from_secs(5))
            .redirect(reqwest::redirect::Policy::none())
            .build()?;
        Ok(Self {
            client,
            endpoint: endpoint.trim_end_matches('/').into(),
            token: token.into(),
            last_batch_size: 0,
        })
    }
}
impl UploadTransport for HttpTransport {
    fn clock_sample(&mut self) -> Result<Option<u64>, JournalError> {
        let response = self
            .client
            .get(format!("{}/v1/clock", self.endpoint))
            .bearer_auth(&self.token)
            .send()?;
        if !response.status().is_success() {
            return Err("clock probe failed".into());
        }
        let mut bytes = Vec::new();
        std::io::Read::take(response, 4096).read_to_end(&mut bytes)?;
        Ok(serde_json::from_slice::<serde_json::Value>(&bytes)?["server_wall_ns"].as_u64())
    }

    fn send(&mut self, events: &[EventEnvelope]) -> Result<Vec<EventId>, JournalError> {
        #[derive(Deserialize)]
        struct Ack {
            acknowledged: Vec<EventId>,
        }
        self.last_batch_size = events.len();
        let response = self
            .client
            .post(format!("{}/v1/events", self.endpoint))
            .bearer_auth(&self.token)
            .json(&serde_json::json!({"events":events}))
            .send()?;
        if !response.status().is_success() {
            return Err(format!("collector status {}", response.status().as_u16()).into());
        }
        let mut bytes = Vec::new();
        std::io::Read::take(response, 1024 * 1024).read_to_end(&mut bytes)?;
        Ok(serde_json::from_slice::<Ack>(&bytes)?.acknowledged)
    }
    fn put_attachment(
        &mut self,
        rover: &str,
        hash: &str,
        bytes: &[u8],
    ) -> Result<(), JournalError> {
        let status = self
            .client
            .put(format!("{}/v1/attachments/{hash}", self.endpoint))
            .bearer_auth(&self.token)
            .header("x-rover-id", rover)
            .body(bytes.to_vec())
            .send()?
            .status();
        if status.is_success() {
            Ok(())
        } else if status.is_client_error() && ![401, 403, 408, 429].contains(&status.as_u16()) {
            Err(Box::new(AttachmentRejected))
        } else {
            Err(format!("attachment status {}", status.as_u16()).into())
        }
    }
}
/// A failed response leaves exactly the original batch pending. Acknowledgments cannot name other events.
pub fn upload_once(
    reader: &JournalReader,
    transport: &mut dyn UploadTransport,
    evidence: Option<&Path>,
) -> Result<usize, JournalError> {
    let mut events = reader.batch(64)?;
    if events.is_empty() {
        return Ok(0);
    }
    while serde_json::to_vec(&serde_json::json!({"events":events}))?.len() > 900 * 1024 {
        events.pop();
        if events.is_empty() {
            return Err("event exceeds ingestion limit".into());
        }
    }
    if let Some(directory) = evidence {
        for e in &events {
            if let Some(items) = e.input.attributes["attachments"].as_array() {
                for item in items {
                    let attachment: Attachment = serde_json::from_value(item.clone())?;
                    if !valid_hash(&attachment.sha256) {
                        return Err("invalid attachment hash".into());
                    }
                    let path = directory.join(&attachment.sha256);
                    match std::fs::metadata(&path) {
                        Ok(m) if m.len() <= 32 * 1024 * 1024 => {
                            let bytes = std::fs::read(path)?;
                            if bytes.len() != attachment.byte_length
                                || format!("{:x}", Sha256::digest(&bytes)) != attachment.sha256
                            {
                                reader.attachment_gap(
                                    e,
                                    &attachment.sha256,
                                    "attachment_integrity_failure",
                                )?;
                                continue;
                            }
                            if let Err(error) = transport.put_attachment(
                                &e.input.rover_id,
                                &attachment.sha256,
                                &bytes,
                            ) {
                                if error.downcast_ref::<AttachmentRejected>().is_some() {
                                    reader.attachment_gap(
                                        e,
                                        &attachment.sha256,
                                        "attachment_rejected",
                                    )?;
                                } else {
                                    return Err(error);
                                }
                            }
                        }
                        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {
                            reader.attachment_gap(
                                e,
                                &attachment.sha256,
                                "attachment_not_available",
                            )?;
                        }
                        _ => {
                            reader.attachment_gap(
                                e,
                                &attachment.sha256,
                                "attachment_unavailable_or_oversized",
                            )?;
                        }
                    }
                }
            }
        }
    }
    let acknowledged = transport.send(&events)?;
    if acknowledged
        .iter()
        .any(|id| !events.iter().any(|e| &e.id == id))
    {
        return Err("invalid acknowledgment".into());
    }
    reader.ack(&acknowledged)?;
    Ok(acknowledged.len())
}
pub fn valid_hash(hash: &str) -> bool {
    hash.len() == 64
        && hash
            .bytes()
            .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
}
#[derive(Serialize, Default)]
pub struct UploadHealth {
    pub failures: u64,
    pub acknowledged: u64,
    pub pending_events: Option<u64>,
    pub last_error: Option<String>,
}
pub struct UploadConfig {
    pub endpoint: String,
    pub token: String,
    pub evidence_directory: Option<PathBuf>,
    pub clock_journal: Option<Arc<crate::Journal>>,
}
#[derive(Default)]
struct UploadState {
    stop: AtomicBool,
    failures: AtomicU64,
    acknowledged: AtomicU64,
    pending: AtomicU64,
    last_error: std::sync::Mutex<Option<String>>,
}
pub struct UploadHandle {
    state: Arc<UploadState>,
}
impl UploadHandle {
    pub fn start(reader: JournalReader, config: UploadConfig) -> Result<Self, JournalError> {
        validate_endpoint(&config.endpoint, &config.token)?;
        let state = Arc::new(UploadState::default());
        state.pending.store(u64::MAX, Ordering::Relaxed);
        let worker_state = state.clone();
        std::thread::Builder::new()
            .name("observation-upload".into())
            .spawn(
                move || match HttpTransport::new(&config.endpoint, &config.token) {
                    Ok(transport) => upload_worker(reader, transport, config, &worker_state),
                    Err(_) => {
                        worker_state.failures.fetch_add(1, Ordering::Relaxed);
                        if let Ok(mut error) = worker_state.last_error.lock() {
                            *error = Some("HTTP client initialization failed".into());
                        }
                    }
                },
            )?;
        Ok(Self { state })
    }
    pub fn health(&self) -> UploadHealth {
        let pending = self.state.pending.load(Ordering::Relaxed);
        UploadHealth {
            failures: self.state.failures.load(Ordering::Relaxed),
            acknowledged: self.state.acknowledged.load(Ordering::Relaxed),
            pending_events: (pending != u64::MAX).then_some(pending),
            last_error: self
                .state
                .last_error
                .try_lock()
                .ok()
                .and_then(|e| e.clone()),
        }
    }
}
impl Drop for UploadHandle {
    fn drop(&mut self) {
        self.state.stop.store(true, Ordering::Release);
    }
}
fn validate_endpoint(endpoint: &str, token: &str) -> Result<(), JournalError> {
    let url = reqwest::Url::parse(endpoint)?;
    let local = matches!(url.host_str(), Some("127.0.0.1" | "localhost" | "[::1]"));
    if endpoint.len() > 2048
        || token.is_empty()
        || token.len() > 4096
        || !((url.scheme() == "https") || (url.scheme() == "http" && local))
        || !url.username().is_empty()
        || url.password().is_some()
        || url.query().is_some()
        || url.fragment().is_some()
    {
        return Err("invalid upload endpoint or credential".into());
    }
    Ok(())
}
fn worker_event(config: &UploadConfig, kind: &str, attributes: serde_json::Value) {
    if let Some(journal) = &config.clock_journal {
        let mut attrs = attributes;
        attrs["time_basis"] = "wall_only".into();
        journal.try_record(crate::EventInput::new(
            "_producer",
            "observability",
            kind,
            attrs,
        ));
    }
}
fn sample_clock(transport: &mut dyn UploadTransport, config: &UploadConfig) {
    let before = crate::wall_time_ns();
    let elapsed = std::time::Instant::now();
    if let Ok(Some(server)) = transport.clock_sample() {
        let after = crate::wall_time_ns();
        if after >= before {
            let midpoint = before + (after - before) / 2;
            let offset = (server as i128 - midpoint as i128)
                .clamp(i64::MIN as i128, i64::MAX as i128) as i64;
            let uncertainty = (elapsed.elapsed().as_nanos() / 2)
                .max(((after - before) / 2) as u128)
                .min(u64::MAX as u128) as u64;
            worker_event(
                config,
                "clock.alignment",
                serde_json::json!({"offset_ns":offset,"uncertainty_ns":uncertainty,"measurement_wall_ns":midpoint,"scope":"RTT uncertainty; clock drift unmeasured"}),
            );
        }
    }
}
fn upload_worker(
    reader: JournalReader,
    mut transport: HttpTransport,
    config: UploadConfig,
    state: &UploadState,
) {
    let mut delay = 1u64;
    let mut next_clock = std::time::Instant::now();
    while !state.stop.load(Ordering::Acquire) {
        if std::time::Instant::now() >= next_clock {
            sample_clock(&mut transport, &config);
            worker_event(
                &config,
                "observability.health",
                serde_json::json!({"failures":state.failures.load(Ordering::Relaxed),"acknowledged":state.acknowledged.load(Ordering::Relaxed),"pending_events":reader.pending_count().ok(),"journal":config.clock_journal.as_ref().map(|j|j.health())}),
            );
            next_clock = std::time::Instant::now() + Duration::from_secs(60);
        }
        let started = std::time::Instant::now();
        let mut wait = Duration::from_secs(1);
        match upload_once(
            &reader,
            &mut transport,
            config.evidence_directory.as_deref(),
        ) {
            Ok(n) => {
                state.acknowledged.fetch_add(n as u64, Ordering::Relaxed);
                delay = 1;
                if n > 0 {
                    // Stay below the collector’s 200 events/second producer budget.
                    wait = Duration::from_secs_f64(transport.last_batch_size as f64 / 180.0)
                        .saturating_sub(started.elapsed());
                }
                if n > 0
                    && let Ok(mut error) = state.last_error.lock()
                    && error.take().is_some()
                {
                    worker_event(
                        &config,
                        "upload.recovered",
                        serde_json::json!({"acknowledged":n}),
                    );
                }
            }
            Err(error) => {
                state.failures.fetch_add(1, Ordering::Relaxed);
                delay = (delay * 2).min(60);
                wait = Duration::from_millis(delay * 1000 + crate::wall_time_ns() % 500);
                let message = error.to_string().chars().take(256).collect::<String>();
                if let Ok(mut previous) = state.last_error.lock()
                    && previous.as_ref() != Some(&message)
                {
                    *previous = Some(message.clone());
                    worker_event(
                        &config,
                        "upload.failed",
                        serde_json::json!({"error":message}),
                    );
                }
            }
        }
        state.pending.store(
            reader.pending_count().unwrap_or(u64::MAX),
            Ordering::Relaxed,
        );
        let deadline = std::time::Instant::now() + wait;
        while !state.stop.load(Ordering::Acquire) {
            let remaining = deadline.saturating_duration_since(std::time::Instant::now());
            if remaining.is_zero() {
                break;
            }
            std::thread::sleep(remaining.min(Duration::from_millis(100)));
        }
    }
}
