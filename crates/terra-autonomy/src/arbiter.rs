//! Decide whether the rover may move, and which command it may execute.
//!
//! Start with `AutonomyArbiter::step`: prepare -> check safety -> choose motion ->
//! limit turning -> apply holds -> report. A hold always produces zero motion.
//! `intent` explains what was requested; only `twist` is safe to send to control.
//!
//! Supporting files: `requests.rs` handles incoming commands, `motion.rs` handles
//! planning, `exploration.rs` handles frontier exploration, `search_requests.rs`
//! handles search messages, and `status.rs` builds
//! telemetry. Public APIs and telemetry strings are unchanged.

use crate::*;
use serde::Serialize;
use std::collections::VecDeque;
use terra_control::{TurningConfig, TurningLimiter};
use terra_exploration::{EndReason, ExplorationStatus, ExploreAction, ExploreTick, Explorer};
use terra_mapping::MapSnapshot;
use terra_navigation::{LocalPlanner, Pose, Twist, frontier};
use terra_types::timing;
/// Backward-compatible name for the shared map freshness default. Edit
/// `terra_types::timing::MAP_FRESHNESS_SECONDS`, or override it per arbiter using
/// [`AutonomyArbiter::set_map_freshness_timeout`]. Pose/operator/detector timeouts
/// are independent. Observations exactly this old are stale.
pub use terra_types::timing::MAP_FRESHNESS_SECONDS as DEFAULT_MAP_FRESHNESS_TIMEOUT_SECONDS;
use terra_waypoint::{GoalCommand, GoalState, WaypointConfig, WaypointController, WaypointStatus};

/// Trusted command provenance, assigned at ingress rather than decoded from a packet.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum CommandOrigin {
    LocalManual,
    FleetTeleop,
    Autonomous,
}
/// Bounded decision-log entry emitted by the authority arbiter.
#[derive(Clone, Debug, Serialize)]
pub struct DecisionEvent {
    /// Machine-readable event category.
    pub kind: String,
    /// Request correlation token; command tokens use 1–64 ASCII letters, digits, or `._:-`.
    pub token: Option<String>,
    /// Machine-readable explanation of the current decision or transition.
    pub reason: String,
    /// Requested or event-time motion authority.
    pub level: Level,
}
/// One control tick of pose, map, measured motion, timestamps, and health evidence.
pub struct ArbiterInput<'a> {
    /// Current control-clock time, in seconds.
    pub now: f64,
    /// World-frame rover pose; position is metres and yaw is radians.
    pub pose: Pose,
    /// Pose receipt time, in seconds on the same clock as `now`.
    pub pose_time: f64,
    /// Latest observed occupancy snapshot, when available.
    pub map: Option<&'a MapSnapshot>,
    /// Map receipt time in seconds on the control clock, when available.
    pub map_time: Option<f64>,
    /// Occupancy-map revision associated with this evidence or tick.
    pub map_revision: u64,
    /// Measured forward velocity (m/s) and yaw rate (rad/s).
    pub measured: Twist,
    /// Whether the platform sensor and control health checks pass.
    pub healthy: bool,
}
/// Selected intent, safety-filtered command, telemetry, and events from one control tick.
pub struct ArbiterOutput {
    /// Origin of the selected authority.
    pub command_origin: CommandOrigin,
    /// Desired motion before authority and safety filtering.
    pub intent: Twist,
    /// Allowed motion command; integrations must send this command to control.
    pub twist: Twist,
    /// Published authority and safety snapshot.
    pub status: AutonomyStatus,
    /// Current waypoint follower status.
    pub goal: WaypointStatus,
    /// Frontier awaiting supervised approval, when present.
    pub proposal: Option<GoalProposal>,
    /// Exploration status, retained after an exploration run ends.
    pub exploration: Option<ExplorationStatus>,
    /// Decision events drained during this tick.
    pub events: Vec<DecisionEvent>,
    /// Whether the downstream controller should clear its accumulated state.
    pub reset_controller: bool,
    /// Target-search lifecycle snapshot when a session exists.
    pub search: Option<SearchStatus>,
    /// Confirmed reports awaiting delivery acknowledgement.
    pub pending_reports: Vec<SearchReport>,
    /// True when `twist` is an exploration look-around: a slow turn in place that
    /// the vehicle may run even where in-place turns are otherwise disabled.
    pub in_place_turn: bool,
    /// L5 task status once a task has started under the `task` level.
    pub task: Option<crate::TaskStatus>,
    /// Open the dump gate this tick. Only an aligned, stopped dock with
    /// approval sets it; every hold clears it. Hosts map it to the gate servo.
    pub gate_open: bool,
}
/// Stateful authority arbiter, waypoint follower, exploration runner, and target-search coordinator.
pub struct AutonomyArbiter {
    /// Local collision-aware planner and its current configuration.
    pub planner: LocalPlanner,
    /// Study condition: operator requests outside it are rejected (see
    /// [`AutonomyArbiter::level_permitted`]). Set with an [`AssignmentRequest`].
    pub assigned_level: Option<Level>,
    map_freshness_timeout_seconds: f64,
    level: Level,
    follower: WaypointController,
    operator: Option<(TeleopRequest, f64)>,
    operator_origin: CommandOrigin,
    turning: TurningLimiter,
    turning_config: TurningConfig,
    turning_identity: Option<(String, u64, CommandOrigin)>,
    maneuver: VecDeque<terra_navigation::ManeuverSegment>,
    maneuver_goal: Option<u64>,
    operator_order: std::collections::BTreeMap<String, u64>,
    latest_operator_time: Option<f64>,
    /// Run identifier used to correlate commands and telemetry.
    pub run_id: String,
    stopped: bool,
    paused: bool,
    pause_reason: &'static str,
    last_time: Option<f64>,
    revision: u64,
    receipt: Option<(String, bool)>,
    request_reason: Option<String>,
    cache: VecDeque<(String, String, bool)>,
    proposal: Option<GoalProposal>,
    next_proposal: u64,
    rejected: Vec<(f64, f64, f64)>,
    events: Vec<DecisionEvent>,
    reset: bool,
    last_plan: Option<(f64, Twist, &'static str)>,
    /// Last motion allowed, zero after a hold. Centres the local planner's
    /// velocity window: a measured speed that sits in the motor deadband
    /// would otherwise pin every command near zero.
    commanded: Twist,
    planner_trace: Option<(f64, terra_navigation::PlannerTrace)>,
    global_route: Vec<[f64; 2]>,
    route_goal: Option<(u64, f64, f64)>,
    route_map_revision: u64,
    route_checked_at: f64,
    route_reaches_goal: bool,
    last_frontier: f64,
    search: Option<SearchController>,
    detector_classes: Vec<String>,
    detector_version: String,
    detector_time: Option<f64>,
    pending_reports: VecDeque<SearchReport>,
    search_sessions: VecDeque<String>,
    reported_sessions: std::collections::BTreeSet<(String, String)>,
    acknowledged_reports: std::collections::BTreeSet<(String, String, String)>,
    /// Frontier-exploration state machine and configuration.
    pub explorer: Explorer,
    exploration_budget: Option<f64>,
    explore_blocked: bool,
    explore_stale_since: Option<f64>,
    explore_unhealthy_since: Option<f64>,
    /// Yaw rate requested by the explorer's look-around on this tick.
    explore_scan: Option<f64>,
    /// Turning limits for the look-around, with in-place turns allowed.
    scan_turning: Option<TurningLimiter>,
    /// L5 task run under the `task` level, kept after it ends for status.
    task: Option<task::TaskRun>,
    /// Name of the host's language model; `task` is offered only while set.
    task_model: Option<String>,
    /// Name of an MCP agent attached to the rover's tools; also offers `task`.
    task_agent: Option<String>,
    task_landmarks: Vec<crate::task::TaskLandmark>,
    next_task_query: u64,
    /// GPS destination in the local frame, refreshed by the host each tick.
    /// Explore biases its frontiers toward it; the L5 prompt lists it as G1.
    outdoor_destination: Option<(f64, f64)>,
    /// Destination the explorer's scorer currently pulls toward.
    explore_bias: Option<(f64, f64)>,
    /// Bucket dock, kept after it ends for status.
    dock: Option<Dock>,
    dock_config: DockConfig,
    next_dock: u64,
    gate_open: bool,
    dock_in_place: bool,
}
impl Default for AutonomyArbiter {
    fn default() -> Self {
        Self {
            planner: LocalPlanner::default(),
            assigned_level: None,
            map_freshness_timeout_seconds: DEFAULT_MAP_FRESHNESS_TIMEOUT_SECONDS,
            level: Level::Teleop,
            follower: WaypointController::new(WaypointConfig::default()).unwrap(),
            operator: None,
            operator_origin: CommandOrigin::LocalManual,
            turning: TurningLimiter::new(TurningConfig::default()).unwrap(),
            turning_config: TurningConfig::default(),
            turning_identity: None,
            maneuver: VecDeque::new(),
            maneuver_goal: None,
            operator_order: Default::default(),
            latest_operator_time: None,
            run_id: "local".into(),
            stopped: false,
            paused: false,
            pause_reason: "supervision_paused",
            last_time: None,
            revision: 0,
            receipt: None,
            request_reason: None,
            cache: VecDeque::new(),
            proposal: None,
            next_proposal: 0,
            rejected: Vec::new(),
            events: Vec::new(),
            reset: true,
            last_plan: None,
            commanded: Twist::default(),
            planner_trace: None,
            global_route: Vec::new(),
            route_goal: None,
            route_map_revision: 0,
            route_checked_at: f64::NEG_INFINITY,
            route_reaches_goal: false,
            last_frontier: -1.,
            search: None,
            detector_classes: vec![],
            detector_version: String::new(),
            detector_time: None,
            pending_reports: VecDeque::new(),
            search_sessions: VecDeque::new(),
            reported_sessions: Default::default(),
            acknowledged_reports: Default::default(),
            explorer: Explorer::default(),
            exploration_budget: None,
            explore_blocked: false,
            explore_stale_since: None,
            explore_unhealthy_since: None,
            explore_scan: None,
            scan_turning: None,
            task: None,
            task_model: None,
            task_agent: None,
            task_landmarks: Vec::new(),
            next_task_query: 0,
            outdoor_destination: None,
            explore_bias: None,
            dock: None,
            dock_config: DockConfig::default(),
            next_dock: 0,
            gate_open: false,
            dock_in_place: false,
        }
    }
}

mod agent;
mod dock;
mod exploration;
mod motion;
mod requests;
mod search_requests;
mod status;
pub use agent::AgentOutcome;
mod task;

/// Working result for one tick. Kept private so external code uses ArbiterOutput.
struct TickDecision {
    goal: WaypointStatus,
    /// Why this tick is moving or holding (for example, "map_stale").
    reason: &'static str,
    /// Who supplied the motion: operator, waypoint, exploration, or none.
    source: &'static str,
    /// Dashboard state: "clear", "hold", or "emergency_stop".
    safety: &'static str,
    /// Requested motion before collision and turning checks.
    intent_command: Twist,
    /// Motion actually allowed after those checks.
    twist: Twist,
    /// A hold forces the final command to zero, even if intent is nonzero.
    held: bool,
    route_failure: Option<&'static str>,
}
impl TickDecision {
    fn new(goal: WaypointStatus) -> Self {
        Self {
            goal,
            reason: "idle",
            source: "none",
            safety: "clear",
            intent_command: Twist::default(),
            twist: Twist::default(),
            held: false,
            route_failure: None,
        }
    }
}

impl AutonomyArbiter {
    /// Set the maximum occupancy observation age in seconds.
    /// Returns false for non-finite or non-positive values, retaining the old value.
    /// This does not change pose, operator command, or detector freshness limits.
    pub fn set_map_freshness_timeout(&mut self, seconds: f64) -> bool {
        if !seconds.is_finite() || seconds <= 0. {
            return false;
        }
        self.map_freshness_timeout_seconds = seconds;
        true
    }

    /// Current maximum occupancy observation age in seconds (exclusive).
    pub fn map_freshness_timeout(&self) -> f64 {
        self.map_freshness_timeout_seconds
    }

    fn map_is_fresh(&self, input: &ArbiterInput<'_>) -> bool {
        fresh(
            input.now,
            input.map_time,
            self.map_freshness_timeout_seconds,
        )
    }

    /// Backing up never needs the occupancy map. The phone camera faces
    /// forward, so the map holds no fresh evidence behind the rover anyway.
    /// True when this tick's request is reverse motion: L1 operator input,
    /// an L5 agent drive, or a bucket dock backing off. Stop, sensor health,
    /// the turning limiter and the relay lease still apply.
    fn reverse_requested(&self, now: f64) -> bool {
        let operator_reverse = self.level == Level::AssistedTeleop
            && self.operator.as_ref().is_some_and(|(r, t)| {
                r.linear < 0. && fresh(now, Some(*t), timing::OPERATOR_COMMAND_FRESHNESS_SECONDS)
            });
        let agent_reverse = self.task_agent_twist(now).is_some_and(|t| t.linear < 0.);
        let dock_backoff = self
            .dock
            .as_ref()
            .is_some_and(|d| d.phase == crate::dock::DockPhase::Backoff);
        operator_reverse || agent_reverse || dock_backoff
    }

    /// Process one set of observations and return the allowed motion command.
    /// Callers must execute `output.twist`, not the unfiltered `output.intent`.
    pub fn step(&mut self, input: ArbiterInput<'_>) -> ArbiterOutput {
        // 1. Identify who is driving and check that the control clock advanced.
        let command_origin = if matches!(self.level, Level::Teleop | Level::AssistedTeleop) {
            self.operator_origin
        } else {
            CommandOrigin::Autonomous
        };
        let valid_time = self.begin_tick(&input, command_origin);
        let goal = self
            .follower
            .step(input.pose.x, input.pose.y, input.pose.yaw)
            .status;
        let mut decision = TickDecision::new(goal);
        let bounded_map = self
            .search_bounded_map(&input)
            .or_else(|| self.explore_planning_map(&input));
        let planning_map = bounded_map.as_ref().or(input.map);

        // 2. Safety has priority over operator intent, goals and planning.
        self.check_initial_safety(&input, valid_time, &mut decision);
        // A bucket dock takes over motion at any level until it ends; it does
        // its own corridor check and keeps the gate shut under every hold.
        let docking = self.drive_dock(&input, &mut decision);
        if !docking && !decision.held && self.level == Level::Task {
            // L5: apply the model's latest answer, or hold while it thinks.
            self.drive_task(&input, &mut decision);
        }
        if !docking && !decision.held {
            // 3. Choose intent and check the planned motion against occupancy.
            self.select_motion(&input, planning_map, command_origin, &mut decision);
        }
        if !docking && self.level == Level::Task {
            self.watch_task_route(&input, &decision);
        }

        // 4. Apply the vehicle's turning constraints to the selected command.
        // Dock commands already respect the turn radius and spin policy.
        if !docking {
            self.limit_turning(&input, command_origin, &mut decision);
        }

        // 5. Update search state and turn every remaining hold into zero motion.
        let search = self.finish_holds(&input, valid_time, &mut decision);
        self.commanded = if decision.held {
            Twist::default()
        } else {
            decision.twist
        };

        // 6. Publish both the requested intent and the motion actually allowed.
        let status = self.status_for_tick(&input, valid_time, &decision);
        if self.level == Level::TargetSearch {
            decision.goal = self
                .follower
                .step(input.pose.x, input.pose.y, input.pose.yaw)
                .status;
        }
        let exploration = status.exploration.clone();
        self.drain_explore_events();
        let in_place_turn = !decision.held
            && (decision.reason == "exploration_scan"
                || (decision.source == "dock" && self.dock_in_place));
        let gate_open = self.gate_open && !decision.held;
        if !gate_open {
            self.gate_open = false;
        }
        ArbiterOutput {
            task: status.task.clone(),
            in_place_turn,
            command_origin,
            intent: decision.intent_command,
            twist: decision.twist,
            status,
            goal: decision.goal,
            proposal: self.proposal.clone(),
            exploration,
            events: std::mem::take(&mut self.events),
            reset_controller: std::mem::take(&mut self.reset),
            search,
            pending_reports: self.pending_reports.iter().cloned().collect(),
            gate_open,
        }
    }
    fn begin_tick(&mut self, input: &ArbiterInput<'_>, command_origin: CommandOrigin) -> bool {
        let identity = (self.run_id.clone(), self.revision, command_origin);
        if self.turning_identity.as_ref() != Some(&identity) {
            self.turning.reset();
            self.last_plan = None;
            self.commanded = Twist::default();
            self.maneuver.clear();
            self.turning_identity = Some(identity);
        }
        let valid_time = input.now.is_finite()
            && input.now >= 0.
            && self.last_time.is_none_or(|t| input.now > t);
        if valid_time {
            self.last_time = Some(input.now);
        }
        if valid_time && let Some(search) = self.search.as_mut() {
            let _ = search.update_time(input.now);
            if search.request.run_id != self.run_id {
                search.cancel("run_changed");
            }
        }

        valid_time
    }

    // First matching condition wins: invalid time, stop, sensors, map, pause.
    fn check_initial_safety(
        &mut self,
        input: &ArbiterInput<'_>,
        valid_time: bool,
        decision: &mut TickDecision,
    ) {
        if !valid_time {
            decision.reason = "invalid_time";
            decision.held = true;
        } else if self.stopped {
            decision.safety = "emergency_stop";
            decision.reason = "operator_stop";
            decision.held = true;
            self.halt_explore(EndReason::OperatorStop);
        } else if !input.healthy
            || (!self.is_manual()
                && (!input.pose.finite()
                    || !fresh(
                        input.now,
                        Some(input.pose_time),
                        timing::POSE_FRESHNESS_SECONDS,
                    )
                    || !input.measured.linear.is_finite()
                    || !input.measured.angular.is_finite()))
        {
            decision.reason = "sensor_unhealthy";
            decision.held = true;
            self.note_safety_hold(input.now);
        } else if !matches!(self.level, Level::Teleop | Level::WaypointDirect)
            && !self.reverse_requested(input.now)
            && (input.map.is_none() || !self.map_is_fresh(input))
        {
            decision.reason = "map_stale";
            decision.held = true;
            self.note_map_stale(input.now);
        } else if self.paused {
            decision.reason = self.pause_reason;
            decision.held = true;
        }
    }

    // Preserve the distinction between an emergency stop and an ordinary hold.
    fn finish_holds(
        &mut self,
        input: &ArbiterInput<'_>,
        valid_time: bool,
        decision: &mut TickDecision,
    ) -> Option<SearchStatus> {
        if decision.held {
            self.turning.reset();
            self.scan_turning = None;
            self.maneuver.clear();
        }
        if (!input.healthy
            || !fresh(
                input.now,
                Some(input.pose_time),
                timing::POSE_FRESHNESS_SECONDS,
            )
            || !valid_time)
            && decision.goal.state == GoalState::Arrived
        {
            decision.goal.state = GoalState::Active;
        }
        if decision.held
            && self.level == Level::TargetSearch
            && decision.reason == "obstacle_blocked"
        {
            if let Some(search) = self.search.as_mut() {
                search.navigator.invalidate_goal(input.now);
            }
            self.last_plan = None;
            // Retire cached motion, but keep fresh diagnostics explaining the route failure.
            self.event("route_failed", None, "obstacle_blocked");
        }
        if decision.held
            && self.level == Level::TargetSearch
            && decision.reason != "search_hold"
            && decision.reason != "obstacle_blocked"
        {
            if let Some(search) = self.search.as_mut() {
                search.hold(decision.reason);
            }
            self.follower.cancel();
        }
        self.collect_search_report(input.now);
        let search_status = self.search_status(input.now);
        if let Some(status) = search_status.as_ref()
            && decision.source != "dock"
            && self.level == Level::TargetSearch
            && status.phase != SearchPhase::Searching
        {
            decision.held = true;
            decision.reason = "search_hold";
        }
        if decision.held {
            if decision.reason != "obstacle_blocked" {
                self.last_plan = None;
                self.planner_trace = None;
            }
            decision.twist = Twist::default();
            decision.source = "none";
            self.reset = true;
            if decision.safety == "clear" {
                decision.safety = "hold";
            }
        }

        search_status
    }
}
fn fresh(now: f64, time: Option<f64>, limit: f64) -> bool {
    time.is_some_and(|t| t.is_finite() && t >= 0. && now >= t && now - t < limit)
}

fn wrap_heading(x: f64) -> f64 {
    (x + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}
