use serde::{Deserialize, Serialize};
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Level {
    #[default]
    Teleop,
    AssistedTeleop,
    Waypoint,
    Supervised,
    TargetSearch,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LevelRequest {
    pub level: Level,
    pub token: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TeleopRequest {
    pub linear: f64,
    pub angular: f64,
    pub run_id: Option<String>,
    pub authority_revision: Option<u64>,
    pub operator_session_id: Option<String>,
    pub sequence: Option<u64>,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum SafetyAction {
    Stop,
    Reset,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SafetyRequest {
    pub action: SafetyAction,
    pub token: String,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Decision {
    Approve,
    Reject,
    Resume,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ProposalDecision {
    pub run_id: String,
    pub proposal_id: Option<u64>,
    pub decision: Decision,
    pub token: String,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct GoalProposal {
    pub run_id: String,
    pub proposal_id: u64,
    pub x: f64,
    pub y: f64,
    pub map_revision: u64,
    pub expires_at: f64,
    pub reason: String,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct AutonomyStatus {
    pub run_id: String,
    pub assigned_level: Option<Level>,
    pub requested_level: Level,
    pub effective_level: Option<Level>,
    pub active_source: String,
    pub safety: String,
    pub reason: String,
    pub revision: u64,
    pub token: Option<String>,
    pub result: Option<String>,
    pub request_reason: Option<String>,
    pub supported_levels: Vec<Level>,
    pub paused: bool,
}
pub fn valid_token(s: &str) -> bool {
    (1..=64).contains(&s.len())
        && s.bytes()
            .all(|b| b.is_ascii_alphanumeric() || b"._:-".contains(&b))
}
fn decode<T: serde::de::DeserializeOwned>(b: &[u8]) -> Option<T> {
    if b.is_empty() || b.len() > 2048 {
        return None;
    }
    serde_json::from_slice(b).ok()
}
pub fn decode_level(b: &[u8]) -> Option<LevelRequest> {
    decode::<LevelRequest>(b).filter(|r| valid_token(&r.token))
}
pub fn decode_safety(b: &[u8]) -> Option<SafetyRequest> {
    decode::<SafetyRequest>(b).filter(|r| valid_token(&r.token))
}
pub fn decode_teleop(b: &[u8]) -> Option<TeleopRequest> {
    decode::<TeleopRequest>(b).filter(|r| {
        r.linear.is_finite()
            && r.angular.is_finite()
            && r.linear.abs() <= 5.
            && r.angular.abs() <= 5.
            && r.run_id.as_ref().is_none_or(|s| valid_token(s))
            && r.operator_session_id
                .as_ref()
                .is_none_or(|s| valid_token(s))
    })
}
pub fn decode_decision(b: &[u8]) -> Option<ProposalDecision> {
    decode::<ProposalDecision>(b).filter(|r| {
        valid_token(&r.token)
            && valid_token(&r.run_id)
            && match r.decision {
                Decision::Resume => r.proposal_id.is_none(),
                _ => r.proposal_id.is_some(),
            }
    })
}
