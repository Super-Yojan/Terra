//! Versioned target-search requests, observations, reports, and acknowledgements.

use crate::valid_token;
use serde::{Deserialize, Serialize};
/// Axis-aligned search rectangle in world metres; valid sides are positive and at most 100 m.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SearchBounds {
    /// Inclusive minimum world x coordinate, in metres.
    pub min_x: f64,
    /// Inclusive minimum world y coordinate, in metres.
    pub min_y: f64,
    /// Inclusive maximum world x coordinate, in metres.
    pub max_x: f64,
    /// Inclusive maximum world y coordinate, in metres.
    pub max_y: f64,
}
impl SearchBounds {
    /// Return whether all coordinates are finite and each side is positive and at most 100 m.
    pub fn valid(self) -> bool {
        [self.min_x, self.min_y, self.max_x, self.max_y]
            .iter()
            .all(|v| v.is_finite())
            && self.max_x > self.min_x
            && self.max_y > self.min_y
            && self.max_x - self.min_x <= 100.
            && self.max_y - self.min_y <= 100.
    }
    /// Return whether finite world-metre coordinates lie inside the rectangle, including its edges.
    pub fn contains(self, x: f64, y: f64) -> bool {
        x.is_finite()
            && y.is_finite()
            && x >= self.min_x
            && x <= self.max_x
            && y >= self.min_y
            && y <= self.max_y
    }
}
/// Version-one target-search session with a bounded region and time budget.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SearchRequest {
    /// Wire schema version; incoming requests require `1`.
    pub version: u32,
    /// Run identifier used to correlate commands and telemetry.
    pub run_id: String,
    /// Search-session identifier scoped to a run.
    pub search_id: String,
    /// Request correlation token; command tokens use 1–64 ASCII letters, digits, or `._:-`.
    pub token: String,
    /// Detector class label, using 1–64 ASCII letters, digits, underscores, or hyphens.
    pub target_class: String,
    /// World-metre rectangle constraining the search.
    pub bounds: SearchBounds,
    /// Finite search budget in seconds, greater than zero and at most 3600.
    pub time_budget_s: f64,
}
impl SearchRequest {
    /// Validate version 1, correlation tokens, target class, bounds, and a budget in `(0, 3600]` seconds.
    pub fn valid(&self) -> bool {
        self.version == 1
            && [&self.run_id, &self.search_id, &self.token]
                .iter()
                .all(|s| valid_token(s))
            && valid_class(&self.target_class)
            && self.bounds.valid()
            && self.time_budget_s.is_finite()
            && self.time_budget_s > 0.
            && self.time_budget_s <= 3600.
    }
}
/// Return whether `s` is 1–64 ASCII letters, digits, underscores, or hyphens.
pub fn valid_class(s: &str) -> bool {
    (1..=64).contains(&s.len())
        && s.bytes()
            .all(|b| b.is_ascii_alphanumeric() || b"_-".contains(&b))
}
/// Operator lifecycle action for an existing search session.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SearchAction {
    /// Pause navigation and clear candidate confirmation evidence.
    Pause,
    /// Resume navigation and replan after a pause or attention hold.
    Resume,
    /// End the search permanently.
    Cancel,
}
/// Versioned, token-correlated lifecycle request for a particular run and search.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SearchActionRequest {
    /// Wire schema version; incoming requests require `1`.
    pub version: u32,
    /// Run identifier used to correlate commands and telemetry.
    pub run_id: String,
    /// Search-session identifier scoped to a run.
    pub search_id: String,
    /// Request correlation token; command tokens use 1–64 ASCII letters, digits, or `._:-`.
    pub token: String,
    /// Requested lifecycle or safety action.
    pub action: SearchAction,
}
impl SearchActionRequest {
    /// Validate version 1 and all run, search, and request tokens.
    pub fn valid(&self) -> bool {
        self.version == 1
            && [&self.run_id, &self.search_id, &self.token]
                .iter()
                .all(|s| valid_token(s))
    }
}
/// Lifecycle state of a sensor-confirmed target search.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SearchPhase {
    /// Selecting or following a reachable frontier.
    Searching,
    /// Holding motion while accumulating consistent detector evidence.
    Confirming,
    /// Target confirmed and a report produced.
    Completed,
    /// Operator-paused; the time budget continues to elapse.
    Paused,
    /// Held until an explicit resume resolves the reported problem.
    NeedsAttention,
    /// Terminal operator or authority cancellation.
    Cancelled,
    /// Terminal expiry of the search time budget.
    TimedOut,
    /// Terminal absence of reachable frontiers; no target confirmation implied.
    Exhausted,
}
impl SearchPhase {
    /// Return whether this phase is completed, cancelled, timed out, or exhausted.
    pub fn terminal(self) -> bool {
        matches!(
            self,
            Self::Completed | Self::Cancelled | Self::TimedOut | Self::Exhausted
        )
    }
}
/// Confirmed target location and the distinct sensor frames supporting it.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SearchReport {
    /// Wire schema version; incoming requests require `1`.
    pub version: u32,
    /// Report correlation identifier, up to 128 ASCII token characters.
    pub report_id: String,
    /// Run identifier used to correlate commands and telemetry.
    pub run_id: String,
    /// Search-session identifier scoped to a run.
    pub search_id: String,
    /// Detector class label, using 1–64 ASCII letters, digits, underscores, or hyphens.
    pub target_class: String,
    /// World-frame target x coordinate, in metres.
    pub world_x: f64,
    /// World-frame target y coordinate, in metres.
    pub world_y: f64,
    /// Confirmation time in seconds on the search control clock.
    pub confirmed_at: f64,
    /// Distinct detector frame identifiers supporting confirmation.
    pub frame_ids: Vec<u64>,
    /// Evidence references corresponding to the supporting frames.
    pub evidence_ids: Vec<String>,
}
/// Search lifecycle, navigation goal, detector health, and latest action receipt.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SearchStatus {
    /// Detector class label, using 1–64 ASCII letters, digits, underscores, or hyphens.
    pub target_class: String,
    /// Wire schema version; incoming requests require `1`.
    pub version: u32,
    /// Run identifier used to correlate commands and telemetry.
    pub run_id: String,
    /// Search-session identifier scoped to a run.
    pub search_id: String,
    /// Monotonically advancing state or authority revision.
    pub revision: u64,
    /// Current lifecycle phase.
    pub phase: SearchPhase,
    /// Machine-readable explanation of the current decision or transition.
    pub reason: String,
    /// Elapsed search time in seconds, including pauses.
    pub elapsed_s: f64,
    /// Current navigation destination as world-metre `(x, y)`, when moving.
    pub active_goal: Option<(f64, f64)>,
    /// Confirmed target report, retained after completion.
    pub report: Option<SearchReport>,
    /// Whether the target class has a fresh advertised detector capability.
    pub detector_available: bool,
    /// Request correlation token; command tokens use 1–64 ASCII letters, digits, or `._:-`.
    pub token: Option<String>,
    /// Latest request outcome, normally `accepted` or `rejected`.
    pub result: Option<String>,
}
/// Versioned delivery acknowledgement correlated to run, search, and report.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SearchReportAck {
    /// Wire schema version; incoming requests require `1`.
    pub version: u32,
    /// Run identifier used to correlate commands and telemetry.
    pub run_id: String,
    /// Search-session identifier scoped to a run.
    pub search_id: String,
    /// Report correlation identifier, up to 128 ASCII token characters.
    pub report_id: String,
    /// Request correlation token; command tokens use 1–64 ASCII letters, digits, or `._:-`.
    pub token: String,
}
fn decode<T: serde::de::DeserializeOwned>(b: &[u8]) -> Option<T> {
    if b.is_empty() || b.len() > 2048 {
        None
    } else {
        serde_json::from_slice(b).ok()
    }
}
/// Decode strict JSON up to 2048 bytes, returning `None` unless the complete search request validates.
pub fn decode_search_request(b: &[u8]) -> Option<SearchRequest> {
    decode::<SearchRequest>(b).filter(SearchRequest::valid)
}
/// Decode strict JSON up to 2048 bytes, returning `None` unless the version and correlation tokens validate.
pub fn decode_search_action(b: &[u8]) -> Option<SearchActionRequest> {
    decode::<SearchActionRequest>(b).filter(SearchActionRequest::valid)
}
/// Decode strict JSON up to 2048 bytes; reject unsupported versions or invalid correlation and report IDs.
pub fn decode_search_report_ack(b: &[u8]) -> Option<SearchReportAck> {
    decode::<SearchReportAck>(b).filter(|r| {
        r.version == 1
            && [&r.run_id, &r.search_id, &r.token]
                .iter()
                .all(|s| valid_token(s))
            && valid_report_id(&r.report_id)
    })
}

/// Return whether `s` is 1–128 ASCII letters, digits, or characters from `._:-`.
pub fn valid_report_id(s: &str) -> bool {
    (1..=128).contains(&s.len())
        && s.bytes()
            .all(|b| b.is_ascii_alphanumeric() || b"._:-".contains(&b))
}
