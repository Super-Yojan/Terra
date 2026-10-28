//! Autonomy levels and operator request/status message contracts.

use serde::{Deserialize, Serialize};
/// Motion authority selected by the operator; serialized using snake-case names.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Level {
    /// Operator velocity with a short-lived input lease.
    #[default]
    Teleop,
    /// Operator intent filtered through collision-aware local planning.
    AssistedTeleop,
    /// Waypoint following with global routing and local collision checks.
    Waypoint,
    /// Direct waypoint following without occupancy-map planning.
    WaypointDirect,
    /// Frontier exploration requiring explicit proposal approval.
    Supervised,
    /// Bounded navigation with sensor-based target confirmation.
    TargetSearch,
    /// Time-bounded frontier exploration. Goals are not held for approval.
    Explore,
    /// L5: an on-device language model turns an operator task into waypoints,
    /// which run on the L3 route. Requires a model advertised by the host.
    Task,
}
/// Token-correlated authority selection with an optional exploration budget.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LevelRequest {
    /// Requested or event-time motion authority.
    pub level: Level,
    /// Request correlation token; command tokens use 1–64 ASCII letters, digits, or `._:-`.
    pub token: String,
    /// Seconds to explore. Omit both budget fields to reuse [`crate::AutonomyArbiter::set_exploration_budget`].
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub budget_seconds: Option<f64>,
    /// Minutes to explore. Mutually exclusive with `budget_seconds`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub budget_minutes: Option<f64>,
}
/// Study condition on the `autonomy` topic: `{"assigned_level": <level> | null, "token"}`.
///
/// While a level is assigned, operator requests for any other level are
/// rejected (`level_not_assigned`). L0 takeover stays allowed, and an assigned
/// `explore` also allows `target_search` (L4 is explore plus search).
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct AssignmentRequest {
    /// Condition level, or `None` to clear the assignment.
    pub assigned_level: Option<Level>,
    /// Request correlation token; command tokens use 1–64 ASCII letters, digits, or `._:-`.
    pub token: String,
}
impl LevelRequest {
    /// Create an authority request with no explicit budget; token validation is deferred to decoding or arbitration.
    pub fn new(level: Level, token: impl Into<String>) -> Self {
        Self {
            level,
            token: token.into(),
            budget_seconds: None,
            budget_minutes: None,
        }
    }
    /// Set a seconds budget and clear the minutes field. Validation is deferred to decoding or arbitration.
    pub fn with_budget_seconds(mut self, seconds: f64) -> Self {
        self.budget_seconds = Some(seconds);
        self.budget_minutes = None;
        self
    }
    /// Resolve exactly one positive finite budget of at most 86400 seconds; return `None` for absent or invalid budgets.
    pub fn resolved_budget_seconds(&self) -> Option<f64> {
        match (self.budget_seconds, self.budget_minutes) {
            (Some(seconds), None) if seconds.is_finite() && seconds > 0. && seconds <= 86_400. => {
                Some(seconds)
            }
            (None, Some(minutes))
                if minutes.is_finite() && minutes > 0. && minutes * 60. <= 86_400. =>
            {
                Some(minutes * 60.)
            }
            _ => None,
        }
    }
}
/// Return whether the request has no budget or exactly one valid budget in `(0, 86400]` seconds.
pub fn budget_fields_valid(request: &LevelRequest) -> bool {
    match (request.budget_seconds, request.budget_minutes) {
        (None, None) => true,
        (Some(_), None) | (None, Some(_)) => request.resolved_budget_seconds().is_some(),
        (Some(_), Some(_)) => false,
    }
}
/// Operator velocity request, optionally bound to a run, authority revision, and packet order.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TeleopRequest {
    /// Requested forward velocity, in metres per second; accepted magnitude is at most 5.
    pub linear: f64,
    /// Requested yaw rate, in radians per second; accepted magnitude is at most 5.
    pub angular: f64,
    /// Run identifier used to correlate commands and telemetry.
    pub run_id: Option<String>,
    /// Optional authority revision required to match before accepting motion.
    pub authority_revision: Option<u64>,
    /// Optional operator-session identifier used with sequence numbers to reject reordered packets.
    pub operator_session_id: Option<String>,
    /// Optional strictly increasing packet sequence within the operator session.
    pub sequence: Option<u64>,
}
/// Action on the latched emergency-stop state.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum SafetyAction {
    /// Latch a stop and clear active motion intent.
    Stop,
    /// Clear the stop only when platform health permits.
    Reset,
}
/// Token-correlated request to latch or reset the stop.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SafetyRequest {
    /// Requested lifecycle or safety action.
    pub action: SafetyAction,
    /// Request correlation token; command tokens use 1–64 ASCII letters, digits, or `._:-`.
    pub token: String,
}
/// Operator response to supervised exploration.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Decision {
    /// Approve the identified unexpired proposal.
    Approve,
    /// Reject and temporarily exclude the identified frontier.
    Reject,
    /// Resume supervised proposal generation without identifying a proposal.
    Resume,
}
/// Run-correlated approval, rejection, or supervision-resume request.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ProposalDecision {
    /// Run identifier used to correlate commands and telemetry.
    pub run_id: String,
    /// Identifier of the proposal being decided; absent for resume.
    pub proposal_id: Option<u64>,
    /// Operator decision to apply.
    pub decision: Decision,
    /// Request correlation token; command tokens use 1–64 ASCII letters, digits, or `._:-`.
    pub token: String,
}
/// Expiring frontier destination awaiting operator approval.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct GoalProposal {
    /// Run identifier used to correlate commands and telemetry.
    pub run_id: String,
    /// Monotonically assigned proposal identifier within this arbiter.
    pub proposal_id: u64,
    /// World-frame x coordinate, in metres.
    pub x: f64,
    /// World-frame y coordinate, in metres.
    pub y: f64,
    /// Occupancy-map revision associated with this evidence or tick.
    pub map_revision: u64,
    /// Proposal expiry in seconds on the control clock; approval must arrive before it.
    pub expires_at: f64,
    /// Machine-readable explanation of the current decision or transition.
    pub reason: String,
}
/// Snapshot of requested authority, effective motion source, holds, and request receipt.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct AutonomyStatus {
    /// World-metre A* route for L3. Empty when no safe global route is available.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub global_route: Vec<[f64; 2]>,
    /// Latest local-planner trajectory diagnostics when available.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub planner_trace: Option<terra_navigation::PlannerTrace>,
    /// Run identifier used to correlate commands and telemetry.
    pub run_id: String,
    /// Study condition: operator requests outside it are rejected (see
    /// [`crate::AutonomyArbiter::level_permitted`]). Set with an [`AssignmentRequest`].
    pub assigned_level: Option<Level>,
    /// Operator-selected authority, even while safety holds motion.
    pub requested_level: Level,
    /// Authority currently allowed to act, or none during a hold.
    pub effective_level: Option<Level>,
    /// Machine-readable source of the selected motion command.
    pub active_source: String,
    /// Machine-readable safety state.
    pub safety: String,
    /// Machine-readable explanation of the current decision or transition.
    pub reason: String,
    /// Monotonically advancing state or authority revision.
    pub revision: u64,
    /// Request correlation token; command tokens use 1–64 ASCII letters, digits, or `._:-`.
    pub token: Option<String>,
    /// Latest request outcome, normally `accepted` or `rejected`.
    pub result: Option<String>,
    /// Explanation of the latest request receipt.
    pub request_reason: Option<String>,
    /// Authority levels advertised by this arbiter.
    pub supported_levels: Vec<Level>,
    /// Whether supervised operation is paused.
    pub paused: bool,
    /// Present once an exploration run has started, including after it ends.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub exploration: Option<terra_exploration::ExplorationStatus>,
    /// Present once an L5 task has started, including after it ends.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub task: Option<crate::TaskStatus>,
    /// The latest `sensor_unhealthy` hold, set by the runtime and kept for a
    /// few seconds so a hold of one tick can still be read on a dashboard.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sensor_hold: Option<SensorHold>,
    /// Present once a bucket dock has started, including after it ends.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dock: Option<crate::DockStatus>,
}
/// What tripped a `sensor_unhealthy` hold.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct SensorHold {
    /// Machine-readable cause, such as `stale_vio`, `pose_stale` or `recording_failed`.
    pub cause: String,
    /// Pose age at the hold in seconds, when a pose existed.
    pub pose_age_s: Option<f64>,
    /// IMU age at the hold in seconds, when an IMU sample existed.
    pub imu_age_s: Option<f64>,
    /// Seconds since the hold, on the control clock.
    pub age_s: f64,
    /// Holds seen in this run.
    pub count: u64,
}
/// Return whether `s` is 1–64 ASCII letters, digits, or characters from `._:-`.
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
/// Decode an [`AssignmentRequest`]; `assigned_level` must be present (null clears).
pub fn decode_assignment(b: &[u8]) -> Option<AssignmentRequest> {
    let value: serde_json::Value = decode(b)?;
    value.get("assigned_level")?;
    serde_json::from_value::<AssignmentRequest>(value)
        .ok()
        .filter(|r| valid_token(&r.token))
}
/// Decode at most 2048 bytes of strict JSON; return `None` for invalid tokens, budgets, fields, or syntax.
pub fn decode_level(b: &[u8]) -> Option<LevelRequest> {
    decode::<LevelRequest>(b).filter(|r| valid_token(&r.token) && budget_fields_valid(r))
}
/// Decode a level request that may carry an inline `task` object, so one message
/// selects L5 and gives the task. A `task` is accepted only with `"level":"task"`.
pub fn decode_level_with_task(b: &[u8]) -> Option<(LevelRequest, Option<crate::TaskSpec>)> {
    if b.is_empty() || b.len() > 2048 {
        return None;
    }
    let mut value: serde_json::Value = serde_json::from_slice(b).ok()?;
    let task = match value.as_object_mut()?.remove("task") {
        None => None,
        Some(task) => Some(serde_json::from_value::<crate::TaskSpec>(task).ok()?),
    };
    let request: LevelRequest = serde_json::from_value(value).ok()?;
    if !valid_token(&request.token) || !budget_fields_valid(&request) {
        return None;
    }
    match &task {
        Some(t) if request.level != Level::Task || !t.valid() => None,
        _ => Some((request, task)),
    }
}
/// Decode at most 2048 bytes of strict JSON; return `None` for invalid fields, tokens, or syntax.
pub fn decode_safety(b: &[u8]) -> Option<SafetyRequest> {
    decode::<SafetyRequest>(b).filter(|r| valid_token(&r.token))
}
/// Decode strict JSON up to 2048 bytes; reject nonfinite velocities, magnitudes above 5, or invalid optional IDs.
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
/// Decode strict JSON up to 2048 bytes; require valid IDs and a proposal ID exactly when approving or rejecting.
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
