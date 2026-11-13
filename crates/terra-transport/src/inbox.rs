//! Bounded control-intent inbox. Stops are protected from eviction and drained last.
use std::time::Instant;

#[cfg(test)]
#[path = "../tests/internal/inbox.rs"]
mod tests;

/// Bounded intent inbox with protected distinct emergency stops.
/// Tuple of topic suffix, uninterpreted payload bytes, and local receipt instant.
/// Consumers validate payloads and calculate age from the instant before acting.
pub(crate) type ControlAction = (String, Vec<u8>, Instant);
fn stop_token(a: &ControlAction) -> Option<String> {
    if a.0 == "hardware" {
        let v: serde_json::Value = serde_json::from_slice(&a.1).ok()?;
        let token = v["token"].as_str()?;
        return (v["action"] == "disarm" && (1..=64).contains(&token.len()))
            .then(|| format!("hardware:{token}"));
    }
    if a.0 != "safety" {
        return None;
    }
    let v: serde_json::Value = serde_json::from_slice(&a.1).ok()?;
    let o = v.as_object()?;
    let token = o.get("token")?.as_str()?;
    (o.len() == 2
        && v["action"] == "stop"
        && (1..=64).contains(&token.len())
        && token
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b"._:-".contains(&b)))
    .then(|| token.into())
}
fn is_stop(a: &ControlAction) -> bool {
    stop_token(a).is_some()
}
/// Append intent to a queue bounded to 256 entries, preserving distinct stops.
/// `kind` is a topic suffix; payload validation remains the consumer’s job.
/// Recognized safety stops and hardware disarms deduplicate by stop token.
/// When full, evicts the first non-stop; if all queued entries are stops, drops
/// the new action. `received` is the local monotonic receipt instant.
pub(crate) fn enqueue_control_action(
    q: &mut std::collections::VecDeque<ControlAction>,
    kind: String,
    payload: Vec<u8>,
    received: Instant,
) {
    let action = (kind, payload, received);
    if let Some(token) = stop_token(&action) {
        q.retain(|a| stop_token(a).as_deref() != Some(&token));
    }
    if q.len() >= 256 {
        if let Some(i) = q.iter().position(|a| !is_stop(a)) {
            q.remove(i);
        } else {
            return;
        }
    }
    q.push_back(action);
}
/// A stop dominates every other request received in the same control tick, including reset.
/// Empties the queue and stably partitions non-stops before recognized stops.
/// Receipt timestamps and relative ordering within each partition are preserved.
pub(crate) fn drain_control_actions(
    q: &mut std::collections::VecDeque<ControlAction>,
) -> Vec<ControlAction> {
    let mut actions = std::mem::take(q).into_iter().collect::<Vec<_>>();
    actions.sort_by_key(is_stop);
    actions
}

/// Operator intents only. Sensor evidence is supplied by the onboard adapter.
/// Match the explicit autonomy, safety, goal, teleop, hardware, navigation-settings,
/// and search intent suffixes. Returns false for sensor, telemetry, and WebRTC
/// signaling topics. Signaling has its own queue and size cap. This allowlist
/// alone does not validate a payload or grant authority.
pub(crate) fn is_control_topic(kind: &str) -> bool {
    matches!(
        kind,
        "autonomy"
            | "safety"
            | "goal"
            | "goal/decision"
            | "navigation/source"
            | "navigation/resume"
            | "teleop"
            | "hardware"
            | "navigation/settings"
            | "search"
            | "search/action"
            | "search/report/ack"
            | "task"
            | "task/action"
            | "dock"
    )
}
