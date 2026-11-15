//! Bounded control-intent inbox. Stops are protected from eviction and drained last.
use std::time::Instant;
use terra_types::requests::{MAX_QUEUED_REQUESTS, MAX_REQUEST_BYTES, RequestKind};

#[cfg(test)]
#[path = "../tests/internal/inbox.rs"]
mod tests;

/// Bounded intent inbox with protected distinct emergency stops.
/// Tuple of topic suffix, uninterpreted payload bytes, and local receipt instant.
/// Consumers validate payloads and calculate age from the instant before acting.
pub(crate) type ControlAction = (String, Vec<u8>, Instant);
/// The token of a well-formed emergency stop: exactly `action` and a valid
/// `token`, so a padded payload cannot claim a stop's eviction protection.
fn stop_token(a: &ControlAction) -> Option<String> {
    let kind = RequestKind::find(&a.0)?;
    let v: serde_json::Value = serde_json::from_slice(&a.1).ok()?;
    let o = v.as_object()?;
    let token = o.get("token")?.as_str()?;
    (kind.stop(&v)
        && o.len() == 2
        && (1..=64).contains(&token.len())
        && token
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b"._:-".contains(&b)))
    .then(|| token.into())
}
fn is_stop(a: &ControlAction) -> bool {
    stop_token(a).is_some()
}
/// Append intent to a queue bounded to [`MAX_QUEUED_REQUESTS`], preserving
/// distinct stops. Payload validation remains the consumer's job.
/// Recognized safety stops deduplicate by stop token. When full, evicts the
/// first non-stop; if all queued entries are stops, drops the new action.
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
    if q.len() >= MAX_QUEUED_REQUESTS {
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

/// Whether an inbound sample is an operator request worth queueing: a topic
/// from [`terra_types::requests`] within the size cap. Sensor, telemetry and
/// WebRTC topics are not. This does not validate the payload or grant authority.
pub(crate) fn admits(kind: &str, payload: &[u8]) -> bool {
    RequestKind::find(kind).is_some() && payload.len() <= MAX_REQUEST_BYTES
}
