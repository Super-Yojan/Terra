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
    std::thread::Builder::new()
        .name("router-journal".into())
        .spawn(move || {
            let mut cmd = Command::new("journalctl");
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
            let Ok(mut child) = cmd.stdout(Stdio::piped()).stderr(Stdio::null()).spawn() else {
                return;
            };
            let Some(out) = child.stdout.take() else {
                return;
            };
            let scope = Scope {
                producer_id: "gcp-router".into(),
                rovers: vec!["_producer".into()],
                read: false,
            };
            // Limit line bytes before decoding; journalctl produces one JSON object per line.
            let mut reader = BufReader::new(out);
            loop {
                let mut line = Vec::new();
                use std::io::Read;
                let result = reader
                    .by_ref()
                    .take(1024 * 1024)
                    .read_until(b'\n', &mut line);
                if result.unwrap_or(0) == 0 {
                    break;
                }
                if line.last() != Some(&b'\n') {
                    break;
                }
                if let Ok(value) = serde_json::from_slice::<Value>(&line)
                    && let Some(event) = project_journal(&value, "gcp-router")
                    && store.ingest(&scope, &[event]).is_ok()
                    && let Some(cursor) = value["__CURSOR"].as_str()
                {
                    let temp = cursor_file.with_extension("tmp");
                    if std::fs::write(&temp, cursor).is_ok() {
                        let _ = std::fs::rename(temp, &cursor_file);
                    }
                }
            }
            let _ = child.kill();
            let _ = child.wait();
        })
}
