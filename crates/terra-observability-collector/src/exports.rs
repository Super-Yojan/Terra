use crate::{App, Scope};
use axum::{
    Json,
    body::Body,
    extract::{Path, State},
    http::{HeaderMap, StatusCode},
    response::Response,
};
use serde::{Deserialize, Serialize};
use std::{
    collections::{HashMap, HashSet},
    io::Write,
    path::PathBuf,
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};
use terra_observability::{JournalError, valid_hash};
#[derive(Clone, Serialize)]
pub struct ExportJob {
    pub id: String,
    pub status: String,
    pub rover_id: String,
    pub session_id: String,
    pub artifacts: Vec<String>,
    pub error: Option<String>,
}
#[derive(Clone)]
pub struct ExportService {
    pub jobs: Arc<Mutex<HashMap<String, ExportJob>>>,
    pub slot: Arc<tokio::sync::Semaphore>,
    pub python: PathBuf,
}
impl Default for ExportService {
    fn default() -> Self {
        Self {
            jobs: Default::default(),
            slot: Arc::new(tokio::sync::Semaphore::new(1)),
            python: std::env::var("TERRA_OBSERVABILITY_EXPORTER_PYTHON")
                .map(PathBuf::from)
                .unwrap_or_else(|_| "python3".into()),
        }
    }
}
#[derive(Deserialize)]
pub struct ExportRequest {
    rover_id: String,
    session_id: String,
    start_ns: Option<u64>,
    end_ns: Option<u64>,
}
pub async fn create(
    State(app): State<App>,
    headers: HeaderMap,
    Json(request): Json<ExportRequest>,
) -> Result<Json<ExportJob>, StatusCode> {
    let scope = crate::api::scope(&app, &headers, true)?;
    if !scope.rovers.contains(&request.rover_id)
        || request.session_id.len() > 256
        || request.session_id.is_empty()
    {
        return Err(StatusCode::FORBIDDEN);
    }
    if request
        .start_ns
        .zip(request.end_ns)
        .is_some_and(|(s, e)| s >= e)
    {
        return Err(StatusCode::BAD_REQUEST);
    }
    let permit = app
        .exports
        .slot
        .clone()
        .try_acquire_owned()
        .map_err(|_| StatusCode::TOO_MANY_REQUESTS)?;
    let id = format!("{:032x}", terra_observability::wall_time_ns());
    let job = ExportJob {
        id: id.clone(),
        status: "running".into(),
        rover_id: request.rover_id.clone(),
        session_id: request.session_id.clone(),
        artifacts: Vec::new(),
        error: None,
    };
    {
        let mut jobs = app
            .exports
            .jobs
            .lock()
            .map_err(|_| StatusCode::INTERNAL_SERVER_ERROR)?;
        if jobs.len() >= 64
            && let Some(old) = jobs.keys().min().cloned()
        {
            jobs.remove(&old);
        }
        jobs.insert(id.clone(), job.clone());
    }
    tokio::task::spawn_blocking(move || {
        let _permit = permit;
        let result = run_export(&app, &scope, &request, &id);
        if let Ok(mut jobs) = app.exports.jobs.lock()
            && let Some(j) = jobs.get_mut(&id)
        {
            match result {
                Ok(()) => {
                    j.status = "complete".into();
                    j.artifacts = vec![
                        "recording.rrd".into(),
                        "recording.manifest.json".into(),
                        "recording.index.json".into(),
                        "recording.events.jsonl".into(),
                    ];
                }
                Err(_) => {
                    j.status = "failed".into();
                    j.error=Some("Export failed; check exporter installation, source limits and collector storage".into());
                }
            }
        }
    });
    Ok(Json(job))
}
fn run_export(
    app: &App,
    scope: &Scope,
    request: &ExportRequest,
    id: &str,
) -> Result<(), JournalError> {
    let root = app.store.config.path.with_extension("exports");
    std::fs::create_dir_all(&root)?;
    // Bounded export history. Active conversion owns the single slot.
    let mut dirs = std::fs::read_dir(&root)?
        .filter_map(Result::ok)
        .filter(|e| e.path().is_dir())
        .collect::<Vec<_>>();
    dirs.sort_by_key(|d| d.file_name());
    let remove = dirs.len().saturating_sub(15);
    for d in dirs.into_iter().take(remove) {
        std::fs::remove_dir_all(d.path())?;
    }
    let directory = root.join(id);
    let source = directory.join("source");
    let attachments = source.join("attachments");
    std::fs::create_dir_all(&attachments)?;
    let mut file = std::fs::File::create(source.join("events.jsonl"))?;
    let mut count = 0usize;
    let mut bytes = 0usize;
    let mut hashes = HashSet::new();
    let mut sources = HashSet::new();
    let mut first_wall = u64::MAX;
    let mut last_wall = 0u64;
    let budget = Instant::now();
    {
        let mut append =
            |e: &crate::StoredEvent, association: Option<&str>| -> Result<(), JournalError> {
                let mut value = serde_json::to_value(e)?;
                if let Some(association) = association {
                    value["association"] = association.into();
                }
                let line = serde_json::to_vec(&value)?;
                count += 1;
                bytes += line.len();
                if count > 100_000 || bytes > 256 * 1024 * 1024 {
                    return Err("export too large; narrow time range".into());
                }
                file.write_all(&line)?;
                file.write_all(b"\n")?;
                if let Some(items) = e.event.input.attributes["attachments"].as_array() {
                    for item in items {
                        if let Some(hash) = item["sha256"].as_str().filter(|h| valid_hash(h)) {
                            hashes.insert(hash.to_string());
                        }
                    }
                }
                Ok(())
            };
        let mut cursor = 0;
        loop {
            if budget.elapsed() > Duration::from_secs(120) {
                return Err("export source scan timed out".into());
            }
            let events = app
                .store
                .events(scope, Some(&request.session_id), cursor, 1000)?;
            let next = events.iter().map(|e| e.cursor).max().unwrap_or(cursor);
            if next <= cursor {
                break;
            }
            for e in events {
                let input = &e.event.input;
                if input.rover_id != request.rover_id
                    || input.session_id.as_deref() != Some(&request.session_id)
                {
                    continue;
                }
                if request.start_ns.is_some_and(|t| input.wall_time_ns < t)
                    || request.end_ns.is_some_and(|t| input.wall_time_ns >= t)
                {
                    continue;
                }
                first_wall = first_wall.min(input.wall_time_ns);
                last_wall = last_wall.max(input.wall_time_ns);
                sources.insert((e.event.id.producer_id.clone(), e.event.id.boot_id.clone()));
                append(&e, None)?;
            }
            cursor = next;
        }
        if first_wall == u64::MAX {
            return Err("no session events".into());
        }
        let db = app.store.db()?;
        let decode = |r: &rusqlite::Row<'_>| -> rusqlite::Result<(i64, u64, String)> {
            Ok((r.get(0)?, r.get(1)?, r.get(2)?))
        };
        // Pi service events need not carry a phone mission ID. Preserve that fact while
        // including only nearby same-rover context, with explicit association metadata.
        let mut contexts=db.prepare("SELECT cursor,received_ns,payload FROM observations WHERE rover=? AND session IS NULL AND json_extract(payload,'$.input.wall_time_ns') BETWEEN ? AND ? ORDER BY cursor LIMIT 100001")?;
        for row in contexts.query_map(
            rusqlite::params![
                request.rover_id,
                first_wall.saturating_sub(10_000_000_000),
                last_wall.saturating_add(10_000_000_000)
            ],
            decode,
        )? {
            let (cursor, received_ns, payload) = row?;
            let e = crate::StoredEvent {
                cursor,
                received_ns,
                event: serde_json::from_str(&payload)?,
            };
            sources.insert((e.event.id.producer_id.clone(), e.event.id.boot_id.clone()));
            append(&e, Some("nearby_unassociated_context"))?;
        }
        // Include alignment context only for producer boots actually present in this export.
        for (producer, boot) in sources {
            let mut clocks=db.prepare("SELECT cursor,received_ns,payload FROM observations WHERE producer=? AND boot=? AND rover='_producer' AND json_extract(payload,'$.input.kind')='clock.alignment' AND json_extract(payload,'$.input.wall_time_ns') BETWEEN ? AND ? ORDER BY cursor LIMIT 100001")?;
            for row in clocks.query_map(
                rusqlite::params![
                    producer,
                    boot,
                    first_wall.saturating_sub(60_000_000_000),
                    last_wall.saturating_add(60_000_000_000)
                ],
                decode,
            )? {
                let (cursor, received_ns, payload) = row?;
                append(
                    &crate::StoredEvent {
                        cursor,
                        received_ns,
                        event: serde_json::from_str(&payload)?,
                    },
                    Some("clock_context"),
                )?;
            }
        }
    }
    file.sync_all()?;
    for hash in hashes {
        if let Ok(data) = app.store.attachment(scope, &hash) {
            std::fs::write(attachments.join(hash), data)?;
        }
    }
    if count == 0 {
        return Err("no session events".into());
    }
    let output = directory.join("recording.rrd");
    let stderr = std::fs::File::create(directory.join("conversion.log"))?;
    let mut child = std::process::Command::new(&app.exports.python)
        .args(["-m", "terra_replay.export", "export"])
        .arg(&source)
        .arg("--output")
        .arg(&output)
        .stdout(std::process::Stdio::null())
        .stderr(stderr)
        .spawn()?;
    let start = Instant::now();
    loop {
        if let Some(status) = child.try_wait()? {
            if status.success() {
                return Ok(());
            }
            return Err("export conversion failed".into());
        }
        if start.elapsed() > Duration::from_secs(180) {
            child.kill()?;
            let _ = child.wait();
            return Err("export timed out".into());
        }
        std::thread::sleep(Duration::from_millis(100));
    }
}
pub async fn status(
    State(app): State<App>,
    headers: HeaderMap,
    Path(id): Path<String>,
) -> Result<Json<ExportJob>, StatusCode> {
    let scope = crate::api::scope(&app, &headers, true)?;
    let job = app
        .exports
        .jobs
        .lock()
        .map_err(|_| StatusCode::INTERNAL_SERVER_ERROR)?
        .get(&id)
        .cloned()
        .ok_or(StatusCode::NOT_FOUND)?;
    if !scope.rovers.contains(&job.rover_id) {
        return Err(StatusCode::NOT_FOUND);
    }
    Ok(Json(job))
}
pub async fn artifact(
    State(app): State<App>,
    headers: HeaderMap,
    Path((id, name)): Path<(String, String)>,
) -> Result<Response, StatusCode> {
    let scope = crate::api::scope(&app, &headers, true)?;
    let job = app
        .exports
        .jobs
        .lock()
        .map_err(|_| StatusCode::INTERNAL_SERVER_ERROR)?
        .get(&id)
        .cloned()
        .ok_or(StatusCode::NOT_FOUND)?;
    if job.status != "complete"
        || !scope.rovers.contains(&job.rover_id)
        || !job.artifacts.contains(&name)
    {
        return Err(StatusCode::NOT_FOUND);
    }
    let file = tokio::fs::File::open(
        app.store
            .config
            .path
            .with_extension("exports")
            .join(id)
            .join(&name),
    )
    .await
    .map_err(|_| StatusCode::NOT_FOUND)?;
    let stream = tokio_util::io::ReaderStream::new(file);
    Response::builder()
        .header("content-type", "application/octet-stream")
        .header(
            "content-disposition",
            format!("attachment; filename=\"{name}\""),
        )
        .body(Body::from_stream(stream))
        .map_err(|_| StatusCode::INTERNAL_SERVER_ERROR)
}
