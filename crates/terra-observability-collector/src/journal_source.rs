//! Allowlisted system journal projection. Source cursor determines immutable event identity.
use crate::{Scope, Store};
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::{
    io::{BufRead, BufReader},
    path::PathBuf,
    process::{Command, Stdio},
};
use terra_observability::{EventEnvelope, EventId, EventInput};
pub fn project_journal(v: &Value, producer: &str) -> Option<EventEnvelope> {
    let unit = v["_SYSTEMD_UNIT"].as_str()?;
    if ![
        "terra-rover.service",
        "bluetooth.service",
        "argos-router.service",
        "argos-router-tls.service",
    ]
    .contains(&unit)
    {
        return None;
    }
    let cursor = v["__CURSOR"].as_str()?;
    let id = format!("{:x}", Sha256::digest(cursor.as_bytes()));
    let message = v["MESSAGE"].as_str().unwrap_or("");
    let lower = message.to_lowercase();
    let text = if ["password", "bearer ", "private key", "credential"]
        .iter()
        .any(|p| lower.contains(p))
    {
        "[sensitive journal message omitted]"
    } else {
        message
    };
    let mut input = EventInput::new(
        "_producer",
        unit,
        "service.journal",
        serde_json::json!({"message":text,"cursor_hash":id}),
    );
    input.wall_time_ns = v["__REALTIME_TIMESTAMP"]
        .as_str()
        .and_then(|t| t.parse::<u64>().ok())?
        .saturating_mul(1000);
    input.monotonic_ns = v["__MONOTONIC_TIMESTAMP"]
        .as_str()
        .and_then(|t| t.parse::<u64>().ok())
        .unwrap_or(0)
        .saturating_mul(1000);
    Some(EventEnvelope {
        schema_version: 1,
        id: EventId {
            producer_id: producer.into(),
            boot_id: id,
            sequence: 0,
        },
        input,
    })
}
/// Separate service thread; permissions are supplied by deployment, never by elevation here.
pub fn follow_router_journal(
    store: Store,
    cursor_file: PathBuf,
) -> std::io::Result<std::thread::JoinHandle<()>> {
    follow_router_journal_program(
        store,
        cursor_file,
        "journalctl".into(),
        std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false)),
        std::time::Duration::from_secs(3),
    )
}
/// Configurable journal executable and cancellation for supervised deployments.
pub fn follow_router_journal_program(
    store: Store,
    cursor_file: PathBuf,
    program: PathBuf,
    stop: std::sync::Arc<std::sync::atomic::AtomicBool>,
    retry: std::time::Duration,
) -> std::io::Result<std::thread::JoinHandle<()>> {
    std::thread::Builder::new()
        .name("router-journal".into())
        .spawn(move || {
            use std::sync::atomic::Ordering;
            let scope = Scope {
                producer_id: "gcp-router".into(),
                rovers: vec!["_producer".into()],
                read: false,
            };
            let mut delay = retry;
            while !stop.load(Ordering::Acquire) {
                let mut cmd = Command::new(&program);
                cmd.args([
                    "--follow",
                    "--output=json",
                    "--no-pager",
                    "--unit=argos-router.service",
                    "--unit=argos-router-tls.service",
                ]);
                if let Ok(cursor) = std::fs::read_to_string(&cursor_file) {
                    cmd.arg(format!("--after-cursor={}", cursor.trim()));
                } else {
                    cmd.args(["--since=-24h", "--lines=5000"]);
                }
                let mut reason = "journal_unavailable";
                if let Ok(mut child) = cmd.stdout(Stdio::piped()).stderr(Stdio::null()).spawn() {
                    if let Some(out) = child.stdout.take() {
                        let mut reader = BufReader::new(out);
                        reason = "journal_stream_ended";
                        while !stop.load(Ordering::Acquire) {
                            let mut line = Vec::new();
                            use std::io::Read;
                            match reader
                                .by_ref()
                                .take(1024 * 1024)
                                .read_until(b'\n', &mut line)
                            {
                                Ok(0) => break,
                                Err(_) => {
                                    reason = "journal_read_failed";
                                    break;
                                }
                                _ => {}
                            }
                            if line.last() != Some(&b'\n') {
                                // Consume the rest in bounded chunks, and report explicitly
                                // before a later cursor can advance beyond this record.
                                if record_fault(
                                    &store,
                                    &scope,
                                    "observability.gap",
                                    "oversized_journal_record",
                                )
                                .is_err()
                                {
                                    reason = "journal_ingest_failed";
                                    break;
                                }
                                loop {
                                    line.clear();
                                    let n = reader
                                        .by_ref()
                                        .take(1024 * 1024)
                                        .read_until(b'\n', &mut line)
                                        .unwrap_or(0);
                                    if n == 0 || line.last() == Some(&b'\n') {
                                        break;
                                    }
                                }
                                continue;
                            }
                            let Some(value) = serde_json::from_slice::<Value>(&line).ok() else {
                                if record_fault(
                                    &store,
                                    &scope,
                                    "observability.gap",
                                    "malformed_journal_record",
                                )
                                .is_err()
                                {
                                    reason = "journal_ingest_failed";
                                    break;
                                }
                                continue;
                            };
                            let Some(event) = project_journal(&value, "gcp-router") else {
                                continue;
                            };
                            // Never consume B after failure to persist A. Restart from the
                            // last committed cursor; immutable IDs make retry harmless.
                            if store.ingest(&scope, &[event]).is_err() {
                                reason = "journal_ingest_failed";
                                break;
                            }
                            if let Some(cursor) = value["__CURSOR"].as_str() {
                                let temp = cursor_file.with_extension("tmp");
                                if std::fs::write(&temp, cursor)
                                    .and_then(|_| std::fs::rename(&temp, &cursor_file))
                                    .is_err()
                                {
                                    reason = "journal_checkpoint_failed";
                                    break;
                                }
                            }
                            delay = retry;
                        }
                    }
                    let _ = child.kill();
                    let _ = child.wait();
                }
                if !stop.load(Ordering::Acquire) {
                    let _ = record_fault(&store, &scope, "service.journal_reconnect", reason);
                    let until = std::time::Instant::now() + delay;
                    while !stop.load(Ordering::Acquire) && std::time::Instant::now() < until {
                        std::thread::sleep(
                            until
                                .saturating_duration_since(std::time::Instant::now())
                                .min(std::time::Duration::from_millis(100)),
                        );
                    }
                    delay = (delay * 2).min(std::time::Duration::from_secs(60));
                }
            }
        })
}
fn record_fault(
    store: &Store,
    scope: &Scope,
    kind: &str,
    reason: &str,
) -> Result<(), terra_observability::JournalError> {
    let event = EventEnvelope {
        schema_version: 1,
        id: EventId {
            producer_id: scope.producer_id.clone(),
            boot_id: uuid_identity(),
            sequence: 0,
        },
        input: EventInput::new(
            "_producer",
            "journal",
            kind,
            serde_json::json!({"reason":reason}),
        ),
    };
    store.ingest(scope, &[event])?;
    Ok(())
}
fn uuid_identity() -> String {
    // New fault identity; ordinary journal records retain their cursor-derived IDs.
    format!("journal-fault-{}", terra_observability::wall_time_ns())
}
