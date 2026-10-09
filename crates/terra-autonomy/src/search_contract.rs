use crate::valid_token;
use serde::{Deserialize, Serialize};
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SearchBounds {
    pub min_x: f64,
    pub min_y: f64,
    pub max_x: f64,
    pub max_y: f64,
}
impl SearchBounds {
    pub fn valid(self) -> bool {
        [self.min_x, self.min_y, self.max_x, self.max_y]
            .iter()
            .all(|v| v.is_finite())
            && self.max_x > self.min_x
            && self.max_y > self.min_y
            && self.max_x - self.min_x <= 100.
            && self.max_y - self.min_y <= 100.
    }
    pub fn contains(self, x: f64, y: f64) -> bool {
        x.is_finite()
            && y.is_finite()
            && x >= self.min_x
            && x <= self.max_x
            && y >= self.min_y
            && y <= self.max_y
    }
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SearchRequest {
    pub version: u32,
    pub run_id: String,
    pub search_id: String,
    pub token: String,
    pub target_class: String,
    pub bounds: SearchBounds,
    pub time_budget_s: f64,
}
impl SearchRequest {
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
pub fn valid_class(s: &str) -> bool {
    (1..=64).contains(&s.len())
        && s.bytes()
            .all(|b| b.is_ascii_alphanumeric() || b"_-".contains(&b))
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SearchAction {
    Pause,
    Resume,
    Cancel,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SearchActionRequest {
    pub version: u32,
    pub run_id: String,
    pub search_id: String,
    pub token: String,
    pub action: SearchAction,
}
impl SearchActionRequest {
    pub fn valid(&self) -> bool {
        self.version == 1
            && [&self.run_id, &self.search_id, &self.token]
                .iter()
                .all(|s| valid_token(s))
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SearchPhase {
    Searching,
    Confirming,
    Completed,
    Paused,
    NeedsAttention,
    Cancelled,
    TimedOut,
    Exhausted,
}
impl SearchPhase {
    pub fn terminal(self) -> bool {
        matches!(
            self,
            Self::Completed | Self::Cancelled | Self::TimedOut | Self::Exhausted
        )
    }
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SearchReport {
    pub version: u32,
    pub report_id: String,
    pub run_id: String,
    pub search_id: String,
    pub target_class: String,
    pub world_x: f64,
    pub world_y: f64,
    pub confirmed_at: f64,
    pub frame_ids: Vec<u64>,
    pub evidence_ids: Vec<String>,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SearchStatus {
    pub target_class: String,
    pub version: u32,
    pub run_id: String,
    pub search_id: String,
    pub revision: u64,
    pub phase: SearchPhase,
    pub reason: String,
    pub elapsed_s: f64,
    pub active_goal: Option<(f64, f64)>,
    pub report: Option<SearchReport>,
    pub detector_available: bool,
    pub token: Option<String>,
    pub result: Option<String>,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SearchReportAck {
    pub version: u32,
    pub run_id: String,
    pub search_id: String,
    pub report_id: String,
    pub token: String,
}
fn decode<T: serde::de::DeserializeOwned>(b: &[u8]) -> Option<T> {
    if b.is_empty() || b.len() > 2048 {
        None
    } else {
        serde_json::from_slice(b).ok()
    }
}
pub fn decode_search_request(b: &[u8]) -> Option<SearchRequest> {
    decode::<SearchRequest>(b).filter(SearchRequest::valid)
}
pub fn decode_search_action(b: &[u8]) -> Option<SearchActionRequest> {
    decode::<SearchActionRequest>(b).filter(SearchActionRequest::valid)
}
pub fn decode_search_report_ack(b: &[u8]) -> Option<SearchReportAck> {
    decode::<SearchReportAck>(b).filter(|r| {
        r.version == 1
            && [&r.run_id, &r.search_id, &r.token]
                .iter()
                .all(|s| valid_token(s))
            && valid_report_id(&r.report_id)
    })
}

pub fn valid_report_id(s: &str) -> bool {
    (1..=128).contains(&s.len())
        && s.bytes()
            .all(|b| b.is_ascii_alphanumeric() || b"._:-".contains(&b))
}
