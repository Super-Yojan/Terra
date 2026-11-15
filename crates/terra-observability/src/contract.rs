//! Wire contract shared by every producer, the uploader and the collector.
//!
//! The rover-pi journal (`apps/rover-pi/terra_rover/observability.py`) mirrors
//! these values; `tests/fixtures/observability/contract.json` pins both sides.
use serde_json::Value;
use std::path::{Path, PathBuf};

/// Event envelope schema version.
pub const SCHEMA_VERSION: u32 = 1;
/// Largest serialized event a producer may queue or the collector will store.
pub const MAX_EVENT_BYTES: usize = 128 * 1024;
/// Most events the collector accepts in one ingestion request.
pub const MAX_INGEST_EVENTS: usize = 256;
/// Collector request body limit for event ingestion.
pub const MAX_INGEST_BODY_BYTES: usize = 1024 * 1024;
/// Uploaders trim a batch below this so it stays under [`MAX_INGEST_BODY_BYTES`].
pub const MAX_UPLOAD_BATCH_BYTES: usize = 900 * 1024;
/// Events an uploader reads from its journal per batch.
pub const UPLOAD_BATCH_EVENTS: usize = 64;
/// Largest evidence attachment either side transfers.
pub const MAX_ATTACHMENT_BYTES: usize = 32 * 1024 * 1024;
/// Largest `rover_id`, `component`, `kind` or `session_id` in an event.
pub const MAX_FIELD_BYTES: usize = 256;
/// Rover id for events about the producer itself rather than a rover.
pub const PRODUCER_ROVER: &str = "_producer";
/// Kind of the event that records lost or omitted observations.
pub const GAP_KIND: &str = "observability.gap";
/// Producer id of the collector host's own journal.
pub const ROUTER_PRODUCER: &str = "gcp-router";
/// Attribute keys containing any of these (case-insensitive) are credentials.
/// Producers strip them; the collector rejects a batch that still has one.
pub const SECRET_KEY_FRAGMENTS: [&str; 7] = [
    "password",
    "authorization",
    "credential",
    "private_key",
    "secret",
    "access_token",
    "refresh_token",
];

/// A journald message containing any of these (case-insensitive) is replaced
/// by [`SENSITIVE_JOURNAL_PLACEHOLDER`] before it becomes an event.
pub const SENSITIVE_JOURNAL_WORDS: [&str; 6] = [
    "password",
    "passphrase",
    "bearer ",
    "private key",
    "credential",
    "authorization:",
];
/// Message text stored in place of a sensitive journald line.
pub const SENSITIVE_JOURNAL_PLACEHOLDER: &str = "[sensitive journal message omitted]";

/// True when a journald message may carry a credential: one of
/// [`SENSITIVE_JOURNAL_WORDS`], or a Wi-Fi key assignment such as `psk=...`,
/// `wpa_psk: ...` or `psk value ...`.
pub fn sensitive_journal_message(text: &str) -> bool {
    let lower = text.to_lowercase();
    SENSITIVE_JOURNAL_WORDS.iter().any(|w| lower.contains(w)) || psk_assignment(&lower)
}

fn psk_assignment(lower: &str) -> bool {
    let word = |c: Option<char>| c.is_some_and(|c| c.is_alphanumeric() || c == '_');
    lower.match_indices("psk").any(|(at, _)| {
        let before = &lower[..at];
        let starts = !word(before.chars().next_back())
            || ["wpa_", "wpa-"].iter().any(|p| {
                before
                    .strip_suffix(p)
                    .is_some_and(|b| !word(b.chars().next_back()))
            });
        let after = &lower[at + 3..];
        if !starts || word(after.chars().next()) {
            return false;
        }
        let rest = after
            .strip_prefix(['\'', '"'])
            .unwrap_or(after)
            .trim_start();
        rest.starts_with(['=', ':'])
            || rest
                .strip_prefix("value")
                .is_some_and(|r| !word(r.chars().next()))
    })
}

/// Directory beside a journal database that holds its evidence attachments.
pub fn evidence_directory(journal: &Path) -> PathBuf {
    journal.with_extension("evidence")
}

/// True when an attribute key names a credential.
pub fn is_secret_key(key: &str) -> bool {
    let key = key.to_lowercase();
    SECRET_KEY_FRAGMENTS.iter().any(|s| key.contains(s))
}

/// True when any nested object key names a credential.
pub fn contains_secret(value: &Value) -> bool {
    match value {
        Value::Object(m) => m
            .iter()
            .any(|(key, v)| is_secret_key(key) || contains_secret(v)),
        Value::Array(a) => a.iter().any(contains_secret),
        _ => false,
    }
}

/// Sanitize structured credential fields before local persistence as well as upload.
pub fn redact(value: &mut Value) {
    match value {
        Value::Object(m) => {
            m.retain(|k, _| !is_secret_key(k));
            for v in m.values_mut() {
                redact(v);
            }
        }
        Value::Array(a) => {
            for v in a {
                redact(v)
            }
        }
        _ => {}
    }
}
