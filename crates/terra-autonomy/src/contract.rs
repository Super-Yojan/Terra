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
impl Level {
    /// Every level, in rung order (L0 to L5).
    pub const ALL: [Level; 8] = [
        Level::Teleop,
        Level::AssistedTeleop,
        Level::WaypointDirect,
        Level::Waypoint,
        Level::Supervised,
        Level::Explore,
        Level::TargetSearch,
        Level::Task,
    ];
    /// Levels every arbiter offers, in the order the phone pickers list them. `target_search` is added while a detector is
    /// fresh and `task` while a language model or MCP agent is attached.
    pub const BASE: [Level; 6] = [
        Level::Teleop,
        Level::AssistedTeleop,
        Level::Waypoint,
        Level::WaypointDirect,
        Level::Explore,
        Level::Supervised,
    ];
    /// Wire name, identical to the serde form.
    pub fn as_str(self) -> &'static str {
        match self {
            Level::Teleop => "teleop",
            Level::AssistedTeleop => "assisted_teleop",
            Level::Waypoint => "waypoint",
            Level::WaypointDirect => "waypoint_direct",
            Level::Supervised => "supervised",
            Level::TargetSearch => "target_search",
            Level::Explore => "explore",
            Level::Task => "task",
        }
    }
    /// The level with this wire name.
    pub fn parse(name: &str) -> Option<Level> {
        Level::ALL.into_iter().find(|level| level.as_str() == name)
    }
    /// L0: direct operator effort without sensor-based navigation.
    pub fn is_manual(self) -> bool {
        self == Level::Teleop
    }
    /// The operator drives (L0, L1); every other level drives on its own targets.
    pub fn operator_driven(self) -> bool {
        matches!(self, Level::Teleop | Level::AssistedTeleop)
    }
    /// Holds without a fresh occupancy map. L0 and L2 direct do not read it.
    pub fn needs_map(self) -> bool {
        !matches!(self, Level::Teleop | Level::WaypointDirect)
    }
    /// Goals are driven along a Hybrid A* route. L2 direct deliberately follows
    /// its goal without map planning.
    pub fn routed(self) -> bool {
        matches!(
            self,
            Level::Waypoint
                | Level::Task
                | Level::Supervised
                | Level::Explore
                | Level::TargetSearch
        )
    }
    /// Takes an operator waypoint goal directly.
    pub fn accepts_goal(self) -> bool {
        matches!(
            self,
            Level::Waypoint | Level::WaypointDirect | Level::Supervised
        )
    }
    /// May hold a GPS destination: L2 and L3 follow it directly, L4 explore and
    /// L5 task plan toward it on the local map. Target search keeps its own
    /// operator-drawn bounds.
    pub fn accepts_outdoor_goal(self) -> bool {
        matches!(
            self,
            Level::WaypointDirect | Level::Waypoint | Level::Explore | Level::Task
        )
    }
    /// Plans toward the GPS destination itself (L4 explore, L5 task) instead of
    /// being driven to it as a waypoint.
    pub fn reads_outdoor_destination(self) -> bool {
        matches!(self, Level::Explore | Level::Task)
    }
    /// Motion is estimated from phone VIO (the L3 route, and L5 on that route).
    pub fn uses_vio(self) -> bool {
        matches!(self, Level::Waypoint | Level::Task)
    }
    /// The name operators and study participants see, identical in ARGOS
    /// (`OPERATOR_LEVELS`), both phone pickers and the thesis tables
    /// (Super-Yojan/ARGOS#112). `target_search` is part of L4, so it shares
    /// the L4 label. `supervised` is not a level in the design.
    pub fn label(self) -> &'static str {
        match self {
            Level::Teleop => "L0 Manual",
            Level::AssistedTeleop => "L1 Assisted",
            Level::WaypointDirect => "L2 Direct",
            Level::Waypoint => "L3 Planned",
            Level::Explore | Level::TargetSearch => "L4 Explore + search",
            Level::Task => "L5 Task",
            Level::Supervised => "Supervised",
        }
    }
}
/// Who supplied this tick's motion. Serialized as `active_source`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Source {
    /// Nothing is driving, including every hold.
    #[default]
    None,
    /// L0 operator velocity.
    Operator,
    /// L1 operator intent through the local planner.
    AssistedOperator,
    /// L2/L3 waypoint follower.
    Waypoint,
    /// Approved supervised frontier.
    Frontier,
    /// L4 exploration frontier or look-around.
    Explore,
    /// Target-search route.
    TargetSearch,
    /// L5 task waypoint.
    Task,
    /// L5 MCP agent drive grant.
    TaskAgentDrive,
    /// Getting unstuck on a routed level.
    Recovery,
    /// Bucket dock.
    Dock,
}
impl Source {
    /// Wire name, identical to the serde form.
    pub fn as_str(self) -> &'static str {
        match self {
            Source::None => "none",
            Source::Operator => "operator",
            Source::AssistedOperator => "assisted_operator",
            Source::Waypoint => "waypoint",
            Source::Frontier => "frontier",
            Source::Explore => "explore",
            Source::TargetSearch => "target_search",
            Source::Task => "task",
            Source::TaskAgentDrive => "task_agent_drive",
            Source::Recovery => "recovery",
            Source::Dock => "dock",
        }
    }
}
/// Kind of a [`crate::DecisionEvent`]: the arbiter's own decisions are
/// variants; kinds forwarded from the exploration, dock and task state
/// machines are carried in [`EventKind::Other`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EventKind {
    AuthorityGenerationChanged,
    CandidateObserved,
    CandidateRejected,
    DockAligned,
    DockBackoff,
    DockCancelled,
    DockDone,
    DockFailed,
    DockGateClosed,
    DockGateOpened,
    DockStarted,
    GoalAccepted,
    GoalCancel,
    GoalDropped,
    GoalRejected,
    LocalFrameReset,
    MissionPaused,
    OperatorReceived,
    ProposalCreated,
    RemoteRequestRejected,
    ReportAcknowledged,
    ReportQueued,
    RequestAccepted,
    RequestRejected,
    RouteFailed,
    RouteSelected,
    SearchAccepted,
    SearchPaused,
    SearchResumed,
    SearchCancelled,
    Takeover,
    TargetConfirmed,
    TaskAction,
    TaskDriveRequested,
    TaskEnded,
    TaskPlan,
    TaskQuestion,
    TaskStarted,
    TaskWaypointAbandoned,
    TiltStop,
    /// A kind named by a forwarded state machine.
    Other(&'static str),
}
impl EventKind {
    /// Wire name, as published in `DecisionEvent::kind`.
    pub fn as_str(self) -> &'static str {
        match self {
            EventKind::AuthorityGenerationChanged => "authority_generation_changed",
            EventKind::CandidateObserved => "candidate_observed",
            EventKind::CandidateRejected => "candidate_rejected",
            EventKind::DockAligned => "dock_aligned",
            EventKind::DockBackoff => "dock_backoff",
            EventKind::DockCancelled => "dock_cancelled",
            EventKind::DockDone => "dock_done",
            EventKind::DockFailed => "dock_failed",
            EventKind::DockGateClosed => "dock_gate_closed",
            EventKind::DockGateOpened => "dock_gate_opened",
            EventKind::DockStarted => "dock_started",
            EventKind::GoalAccepted => "goal_accepted",
            EventKind::GoalCancel => "goal_cancel",
            EventKind::GoalDropped => "goal_dropped",
            EventKind::GoalRejected => "goal_rejected",
            EventKind::LocalFrameReset => "local_frame_reset",
            EventKind::MissionPaused => "mission_paused",
            EventKind::OperatorReceived => "operator_received",
            EventKind::ProposalCreated => "proposal_created",
            EventKind::RemoteRequestRejected => "remote_request_rejected",
            EventKind::ReportAcknowledged => "report_acknowledged",
            EventKind::ReportQueued => "report_queued",
            EventKind::RequestAccepted => "request_accepted",
            EventKind::RequestRejected => "request_rejected",
            EventKind::RouteFailed => "route_failed",
            EventKind::RouteSelected => "route_selected",
            EventKind::SearchAccepted => "search_accepted",
            EventKind::SearchPaused => "search_paused",
            EventKind::SearchResumed => "search_resumed",
            EventKind::SearchCancelled => "search_cancelled",
            EventKind::Takeover => "takeover",
            EventKind::TargetConfirmed => "target_confirmed",
            EventKind::TaskAction => "task_action",
            EventKind::TaskDriveRequested => "task_drive_requested",
            EventKind::TaskEnded => "task_ended",
            EventKind::TaskPlan => "task_plan",
            EventKind::TaskQuestion => "task_question",
            EventKind::TaskStarted => "task_started",
            EventKind::TaskWaypointAbandoned => "task_waypoint_abandoned",
            EventKind::TiltStop => "tilt_stop",
            EventKind::Other(name) => name,
        }
    }
}
/// Dashboard safety state. Serialized as `safety`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Safety {
    /// Motion is allowed.
    #[default]
    Clear,
    /// An ordinary hold: zero motion until the condition clears.
    Hold,
    /// A latched operator, keyboard or tilt stop; only a safety reset clears it.
    EmergencyStop,
}
impl Safety {
    /// Wire name, identical to the serde form.
    pub fn as_str(self) -> &'static str {
        match self {
            Safety::Clear => "clear",
            Safety::Hold => "hold",
            Safety::EmergencyStop => "emergency_stop",
        }
    }
}
/// Why a tick moves or holds, published as `AutonomyStatus::reason`.
///
/// The reasons the arbiter and its hosts branch on are variants. Every other
/// reason (dock, task, search, pause and route-failure details) is carried as
/// its wire string in [`Reason::Other`]; build reasons with `Reason::from(&str)`
/// so a known string always becomes its variant.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Reason {
    #[default]
    Idle,
    /// Moving as intended (see [`terra_navigation::PlanReason::Active`]).
    Active,
    /// Moving, but the local planner changed the intent.
    Assisted,
    /// The local planner found no admissible motion.
    ObstacleBlocked,
    /// The local planner was given invalid input; not evidence of an obstacle.
    PlannerInvalidInput,
    InvalidTime,
    OperatorStop,
    TiltStop,
    SensorUnhealthy,
    MapStale,
    RoutePlanning,
    RouteNoPath,
    RouteGoalBlocked,
    HeadingRequiresSpinOrReplan,
    InvalidTurnTiming,
    /// Recovery could not free the rover; a route failure retried after a wait.
    BoxedIn,
    ExplorationScan,
    SearchHold,
    /// Dock creeping forward at zero twist between pulses, which is not a hold.
    DockCreep,
    /// Any other reason, by wire name.
    Other(&'static str),
}
impl Reason {
    const KNOWN: [Reason; 19] = [
        Reason::Idle,
        Reason::Active,
        Reason::Assisted,
        Reason::ObstacleBlocked,
        Reason::PlannerInvalidInput,
        Reason::InvalidTime,
        Reason::OperatorStop,
        Reason::TiltStop,
        Reason::SensorUnhealthy,
        Reason::MapStale,
        Reason::RoutePlanning,
        Reason::RouteNoPath,
        Reason::RouteGoalBlocked,
        Reason::HeadingRequiresSpinOrReplan,
        Reason::InvalidTurnTiming,
        Reason::BoxedIn,
        Reason::ExplorationScan,
        Reason::SearchHold,
        Reason::DockCreep,
    ];
    /// Wire name.
    pub fn as_str(self) -> &'static str {
        match self {
            Reason::Idle => "idle",
            Reason::Active => "active",
            Reason::Assisted => "assisted",
            Reason::ObstacleBlocked => "obstacle_blocked",
            Reason::PlannerInvalidInput => "planner_invalid_input",
            Reason::InvalidTime => "invalid_time",
            Reason::OperatorStop => "operator_stop",
            Reason::TiltStop => "tilt_stop",
            Reason::SensorUnhealthy => "sensor_unhealthy",
            Reason::MapStale => "map_stale",
            Reason::RoutePlanning => "route_planning",
            Reason::RouteNoPath => "route_no_path",
            Reason::RouteGoalBlocked => "route_goal_blocked",
            Reason::HeadingRequiresSpinOrReplan => "heading_requires_spin_or_replan",
            Reason::InvalidTurnTiming => "invalid_turn_timing",
            Reason::BoxedIn => "boxed_in",
            Reason::ExplorationScan => "exploration_scan",
            Reason::SearchHold => "search_hold",
            Reason::DockCreep => "dock_creep",
            Reason::Other(name) => name,
        }
    }
    /// The variant with this wire name, or `None` for a reason carried in [`Reason::Other`].
    pub fn known(name: &str) -> Option<Reason> {
        Reason::KNOWN.into_iter().find(|r| r.as_str() == name)
    }
    /// The route to the current goal cannot be driven right now: the planner
    /// is blocked, no route exists, the goal is blocked, or the heading needs a
    /// turn the vehicle may not make. L5 abandons a waypoint that stays like
    /// this; the outdoor mission reports it as `route_blocked`.
    pub fn is_route_blocked(self) -> bool {
        matches!(
            self,
            Reason::ObstacleBlocked
                | Reason::RouteNoPath
                | Reason::RouteGoalBlocked
                | Reason::HeadingRequiresSpinOrReplan
        )
    }
}
impl From<&'static str> for Reason {
    fn from(name: &'static str) -> Self {
        Reason::known(name).unwrap_or(Reason::Other(name))
    }
}
impl From<terra_navigation::RouteError> for Reason {
    fn from(error: terra_navigation::RouteError) -> Self {
        Reason::from(error.as_str())
    }
}
impl From<terra_navigation::PlanReason> for Reason {
    fn from(reason: terra_navigation::PlanReason) -> Self {
        match reason {
            terra_navigation::PlanReason::Active => Reason::Active,
            terra_navigation::PlanReason::Assisted => Reason::Assisted,
            terra_navigation::PlanReason::ObstacleBlocked => Reason::ObstacleBlocked,
            terra_navigation::PlanReason::InvalidInput => Reason::PlannerInvalidInput,
        }
    }
}
impl Serialize for Reason {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.serialize_str(self.as_str())
    }
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
            (Some(seconds), None) if terra_exploration::valid_budget(seconds) => Some(seconds),
            (None, Some(minutes)) if terra_exploration::valid_budget(minutes * 60.) => {
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
    /// Source of the selected motion command.
    pub active_source: Source,
    /// Dashboard safety state.
    pub safety: Safety,
    /// Machine-readable explanation of the current decision, normally
    /// [`Reason::as_str`]; hosts may replace it with an outdoor-mission hold reason.
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
    /// L5 answerers this phone has: `builtin` (a built-in model) and/or `mcp` (an MCP agent).
    /// A task's `agent` must name one of them.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub task_agents: Vec<crate::TaskAgent>,
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
    /// Seconds the current hold has lasted without a break; 0 once it clears.
    #[serde(default)]
    pub held_s: f64,
    /// Data is fresh again after a long hold, and the rover waits a short
    /// settle period before it drives.
    #[serde(default)]
    pub settling: bool,
    /// The hold outlasted the goal timeout and the L2/L3 goal was dropped.
    #[serde(default)]
    pub goal_dropped: bool,
}
pub use terra_waypoint::valid_token;
use terra_types::requests::MAX_REQUEST_BYTES;
/// Decode one strict JSON request of 1 to [`MAX_REQUEST_BYTES`] bytes.
pub(crate) fn decode<T: serde::de::DeserializeOwned>(b: &[u8]) -> Option<T> {
    if b.is_empty() || b.len() > MAX_REQUEST_BYTES {
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
    let mut value: serde_json::Value = decode(b)?;
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
