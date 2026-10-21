//! Select motion authority from operator requests, onboard evidence, and safety state.
//! The tick combines waypoint/search intent with planning and emits reasons for holds.

use crate::*;
use serde::Serialize;
use std::collections::VecDeque;
use terra_control::{TurningConfig, TurningLimiter};
use terra_exploration::{EndReason, ExplorationStatus, ExploreAction, ExploreTick, Explorer};
use terra_mapping::MapSnapshot;
use terra_navigation::{LocalPlanner, Pose, Twist, frontier};
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
}
/// Stateful authority arbiter, waypoint follower, exploration runner, and target-search coordinator.
pub struct AutonomyArbiter {
    /// Local collision-aware planner and its current configuration.
    pub planner: LocalPlanner,
    /// Optional externally assigned authority for display.
    pub assigned_level: Option<Level>,
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
}
impl Default for AutonomyArbiter {
    fn default() -> Self {
        Self {
            planner: LocalPlanner::default(),
            assigned_level: None,
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
        }
    }
}
impl AutonomyArbiter {
    /// Current map-age limit, exposed for diagnostics without changing admission.
    pub fn map_freshness_timeout(&self) -> f64 { 0.5 }
    /// Outdoor guidance is admitted only by the L3 authority path.
    pub fn outdoor_authority(&self) -> bool {
        self.level == Level::Waypoint && !self.stopped && !self.paused
    }
    pub fn has_active_goal(&self, pose: Pose) -> bool {
        self.follower.step(pose.x, pose.y, pose.yaw).status.state == GoalState::Active
    }
    pub fn clear_guidance(&mut self) {
        self.follower.cancel();
        self.maneuver.clear();
    }
    pub fn update_guidance(&mut self, x: f64, y: f64) {
        if self.follower.step(x, y, 0.).status.state == GoalState::Idle {
            let _ = self.follower.accept(
                &GoalCommand::Local {
                    x,
                    y,
                    yaw: None,
                    token: None,
                },
                20_000.,
            );
        } else {
            self.follower.update_guidance(x, y);
        }
        self.maneuver.clear();
    }

    /// Whether this tick uses direct operator effort rather than sensor-based navigation.
    pub fn is_manual(&self) -> bool { self.level == Level::Teleop }

    /// Check live authority/goal state, including commands accepted earlier in this tick.
    pub fn navigation_configuration_allowed(
        &self,
        run_id: &str,
        revision: u64,
        pose: Pose,
    ) -> bool {
        self.run_id == run_id
            && self.revision == revision
            && !self.stopped
            && self.follower.step(pose.x, pose.y, pose.yaw).status.state != GoalState::Active
    }
    /// Invalidate cached trajectories and routes after a validated configuration change.
    pub fn configure_planner(&mut self, config: terra_navigation::PlannerConfig) {
        self.planner.config = config;
        self.last_plan = None;
        self.planner_trace = None;
        self.global_route.clear();
        self.route_goal = None;
        self.route_map_revision = 0;
        self.route_checked_at = 0.;
        self.operator = None;
        self.reset = true;
        self.revision = self.revision.saturating_add(1);
    }

    /// Held local stop always relatches after reset and never replays an old token.
    pub fn hold_emergency_stop(&mut self) {
        if !self.stopped {
            self.set_safety(
                SafetyRequest {
                    action: SafetyAction::Stop,
                    token: format!("keyboard-{}-{}", self.run_id, self.revision),
                },
                true,
            );
        }
    }
    /// Set the waypoint WGS84 origin in degrees; invalid coordinates are ignored by the follower.
    pub fn set_origin(&mut self, lat: f64, lon: f64) {
        let _ = self.follower.set_origin(lat, lon);
    }
    fn event(&mut self, kind: &str, token: Option<String>, reason: &str) {
        if self.events.len() < 256 {
            self.events.push(DecisionEvent {
                kind: kind.into(),
                token,
                reason: reason.into(),
                level: self.level,
            });
        }
    }
    fn duplicate(&mut self, token: &str, fingerprint: &str) -> bool {
        if let Some((_, old, ok)) = self.cache.iter().find(|(t, _, _)| t == token) {
            self.receipt = Some((token.into(), old == fingerprint && *ok));
            true
        } else {
            false
        }
    }
    fn acknowledge(&mut self, token: String, fingerprint: String, ok: bool, reason: &str) {
        self.request_reason = Some(reason.into());
        self.revision = self.revision.saturating_add(1);
        self.receipt = Some((token.clone(), ok));
        self.cache.push_back((token.clone(), fingerprint, ok));
        if self.cache.len() > 128 {
            self.cache.pop_front();
        }
        self.event(
            if ok {
                "request_accepted"
            } else {
                "request_rejected"
            },
            Some(token),
            reason,
        );
    }
    /// Validated managed turning settings; changing them clears motion history.
    pub fn configure_turning(
        &mut self,
        config: TurningConfig,
    ) -> Result<(), terra_types::InputError> {
        self.turning = TurningLimiter::new(config)?;
        self.turning_config = config;
        self.last_plan = None;
        self.reset = true;
        Ok(())
    }
    fn clear(&mut self) {
        self.turning.reset();
        self.turning_identity = None;
        self.maneuver.clear();
        self.maneuver_goal = None;
        if let Some(search) = self.search.as_mut() {
            search.cancel("authority_cleared");
        }
        self.operator = None;
        self.follower.cancel();
        self.proposal = None;
        self.reset = true;
        self.last_plan = None;
        self.planner_trace = None;
    }
    /// Store a budget used when an `explore` level request omits one.
    ///
    /// Return `true` for finite `seconds` in `(0, 86400]`; invalid values return
    /// `false` and leave the previous budget unchanged.
    pub fn set_exploration_budget(&mut self, seconds: f64) -> bool {
        if seconds.is_finite() && seconds > 0. && seconds <= 86_400. {
            self.exploration_budget = Some(seconds);
            true
        } else {
            false
        }
    }
    /// Phase 2 hook. Ignored unless an exploration run is already in progress.
    pub fn notify_detection(&mut self, detection: terra_exploration::Detection) {
        self.explorer.notify_detection(detection);
    }
    /// Apply a token-correlated authority request, clearing old intent on acceptance.
    ///
    /// Invalid tokens are ignored. Budget and detector rejections are exposed through status receipts and decision events.
    pub fn set_level(&mut self, r: LevelRequest) {
        let fp = format!(
            "level:{:?}:{:?}:{:?}",
            r.level, r.budget_seconds, r.budget_minutes
        );
        if !valid_token(&r.token) || self.duplicate(&r.token, &fp) {
            return;
        }
        if r.level == Level::TargetSearch && self.detector_classes.is_empty() {
            self.acknowledge(r.token, fp, false, "detector_unavailable");
            return;
        }
        if !budget_fields_valid(&r) {
            self.acknowledge(r.token, fp, false, "invalid_budget");
            return;
        }
        let budget = r
            .resolved_budget_seconds()
            .or(if r.level == Level::Explore {
                self.exploration_budget
            } else {
                None
            });
        if r.level == Level::Explore && budget.is_none() {
            self.acknowledge(r.token, fp, false, "budget_required");
            return;
        }
        if r.level != Level::Explore && self.explorer.is_running() {
            self.explorer.stop(EndReason::OperatorTakeover);
        }
        let changed = self.level != r.level;
        self.clear();
        self.level = r.level;
        let config = if r.level == Level::TargetSearch {
            WaypointConfig {
                arrive_radius: 0.1,
                slow_radius: 0.5,
                ..WaypointConfig::default()
            }
        } else {
            WaypointConfig::default()
        };
        self.follower = WaypointController::new(config).unwrap();
        self.paused = false;
        self.explore_blocked = false;
        self.explore_stale_since = None;
        self.explore_unhealthy_since = None;
        if r.level == Level::Teleop {
            self.event("takeover", Some(r.token.clone()), "operator_takeover");
        }
        if r.level == Level::Explore
            && let Some(seconds) = budget
        {
            self.exploration_budget = Some(seconds);
            self.explorer.start(seconds, &self.run_id);
        }
        self.acknowledge(
            r.token,
            fp,
            true,
            if changed {
                "operator_selection"
            } else {
                "operator_reselection"
            },
        );
    }
    /// Accept operator velocities at `now` seconds when authority, lease ordering, and optional run/revision bindings match.
    ///
    /// Invalid, stale, reordered, or unauthorized requests are ignored without renewing the motion lease.
    pub fn accept_operator(&mut self, r: TeleopRequest, now: f64) {
        self.accept_operator_from(r, now, CommandOrigin::LocalManual);
    }
    /// Provenance is assigned by a trusted ingress; invalid requests do not change it.
    pub fn accept_operator_from(
        &mut self,
        r: TeleopRequest,
        now: f64,
        origin: CommandOrigin,
    ) -> bool {
        if !now.is_finite()
            || now < 0.
            || self.last_time.is_some_and(|t| now < t)
            || self.latest_operator_time.is_some_and(|t| now < t)
            || !r.linear.is_finite()
            || !r.angular.is_finite()
            || r.linear.abs() > 5.
            || r.angular.abs() > 5.
        {
            return false;
        }
        if r.run_id.as_ref().is_some_and(|run| run != &self.run_id)
            || r.authority_revision
                .is_some_and(|revision| revision != self.revision)
        {
            return false;
        }
        if self.stopped || !matches!(self.level, Level::Teleop | Level::AssistedTeleop) {
            return false;
        }
        if let (Some(session), Some(sequence)) = (&r.operator_session_id, r.sequence) {
            if self
                .operator_order
                .get(session)
                .is_some_and(|last| sequence <= *last)
            {
                return false;
            }
            if self.operator_order.len() >= 128 && !self.operator_order.contains_key(session) {
                return false;
            }
            self.operator_order.insert(session.clone(), sequence);
        }
        self.latest_operator_time = Some(now);
        self.event(
            "operator_received",
            r.operator_session_id.clone(),
            &format!("sequence:{:?}", r.sequence),
        );
        self.operator = Some((r, now));
        self.operator_origin = origin;
        true
    }
    /// Accept or cancel a waypoint and return whether it was accepted.
    ///
    /// Local coordinates and `extent` use metres; WGS84 coordinates use degrees and yaw uses radians.
    /// Invalid, out-of-bounds, unauthorized, or conflicting-token goals return `false`; cancellation clears active intent.
    pub fn reject_goal(&mut self, g: &GoalCommand, reason: &str) {
        let token = match g {
            GoalCommand::Local { token, .. } | GoalCommand::Wgs84 { token, .. } => token.clone(),
            GoalCommand::Cancel => None,
        };
        if let Some(t) = token.clone() {
            self.acknowledge(t, format!("goal:{g:?}"), false, reason);
        }
        self.event("goal_rejected", token, reason);
    }
    pub fn accept_goal(&mut self, g: &GoalCommand, extent: f64) -> bool {
        if self.level == Level::TargetSearch {
            self.event("goal_rejected", None, "search_active");
            return false;
        }
        if matches!(g, GoalCommand::Cancel) {
            if self.level == Level::Explore && self.explorer.is_running() {
                self.explorer.stop(EndReason::OperatorStop);
            }
            self.clear();
            if matches!(self.level, Level::Waypoint | Level::WaypointDirect) {
                self.level = Level::Teleop;
            } else if self.level == Level::Supervised {
                self.paused = true;
            }
            self.revision += 1;
            self.event("goal_cancel", None, "operator_cancel");
            return true;
        }
        let token = match g {
            GoalCommand::Local { token, .. } | GoalCommand::Wgs84 { token, .. } => token.clone(),
            _ => None,
        };
        let valid = match g {
            GoalCommand::Local { x, y, yaw, token } => {
                x.is_finite()
                    && y.is_finite()
                    && yaw.is_none_or(|v| v.is_finite() && v.abs() <= std::f64::consts::TAU)
                    && token.as_ref().is_none_or(|t| valid_token(t))
            }
            GoalCommand::Wgs84 {
                latitude,
                longitude,
                yaw,
                token,
            } => {
                terra_waypoint::GeoOrigin::new(*latitude, *longitude).is_ok()
                    && yaw.is_none_or(|v| v.is_finite() && v.abs() <= std::f64::consts::TAU)
                    && token.as_ref().is_none_or(|t| valid_token(t))
            }
            GoalCommand::Cancel => true,
        };
        if !valid {
            self.event("goal_rejected", token, "invalid_goal");
            return false;
        }
        let fingerprint = format!("goal:{g:?}");
        if let Some(t) = token.as_ref()
            && self.duplicate(t, &fingerprint)
        {
            return self.receipt.as_ref().is_some_and(|r| r.1);
        }
        if self.stopped
            || !matches!(
                self.level,
                Level::Waypoint | Level::WaypointDirect | Level::Supervised
            )
        {
            if let Some(t) = token.clone() {
                self.acknowledge(t, fingerprint, false, "authority_required");
            }
            self.event("goal_rejected", token, "authority_required");
            return false;
        }
        let ok = self.follower.accept(g, extent).unwrap_or(false);
        if ok {
            self.last_plan = None;
            self.planner_trace = None;
            self.proposal = None;
            self.paused = false;
        }
        if let Some(t) = token.clone() {
            self.acknowledge(
                t,
                fingerprint,
                ok,
                if ok {
                    "operator_goal"
                } else {
                    "goal_out_of_bounds"
                },
            );
        }
        self.event(
            if ok { "goal_accepted" } else { "goal_rejected" },
            token,
            "operator_goal",
        );
        ok
    }
    /// Apply a valid token-correlated stop/reset; reset is rejected when `healthy` is false.
    ///
    /// Stops clear motion intent immediately. Receipts and events expose acceptance or rejection.
    pub fn set_safety(&mut self, r: SafetyRequest, healthy: bool) {
        let fp = format!("safety:{:?}", r.action);
        if !valid_token(&r.token) || self.duplicate(&r.token, &fp) {
            return;
        }
        let ok = r.action == SafetyAction::Stop || healthy;
        if ok {
            if r.action == SafetyAction::Stop && self.explorer.is_running() {
                self.explorer.stop(EndReason::OperatorStop);
            }
            self.clear();
            self.stopped = r.action == SafetyAction::Stop;
        }
        self.acknowledge(
            r.token,
            fp,
            ok,
            if ok {
                "operator_safety"
            } else {
                "unhealthy_reset"
            },
        );
    }
    /// Apply a run-correlated supervised decision; approvals recheck freshness, health, expiry, and reachability.
    ///
    /// Rejected or stale decisions produce a rejected receipt; resume requires no proposal identifier.
    pub fn decide_proposal(&mut self, r: ProposalDecision, input: &ArbiterInput<'_>) {
        let fp = format!("decision:{}:{:?}:{:?}", r.run_id, r.decision, r.proposal_id);
        if !valid_token(&r.token) || self.duplicate(&r.token, &fp) {
            return;
        }
        let mut ok = false;
        if self.level == Level::Supervised && !self.stopped && r.run_id == self.run_id {
            if r.decision == Decision::Resume && r.proposal_id.is_none() {
                self.paused = false;
                ok = true;
            } else if let Some(p) = self.proposal.clone()
                && r.proposal_id == Some(p.proposal_id)
                && input.now.is_finite()
                && input.now < p.expires_at
            {
                if r.decision == Decision::Reject {
                    self.rejected.push((p.x, p.y, input.now + 30.));
                    self.proposal = None;
                    ok = true;
                } else if r.decision == Decision::Approve
                    && fresh(input.now, input.map_time, 0.5)
                    && fresh(input.now, Some(input.pose_time), 0.5)
                    && input.healthy
                {
                    // Revalidate reachability on the latest occupancy snapshot, independently of the proposal revision.
                    if let Some(m) = input.map
                        && terra_navigation::reachable(
                            m,
                            input.pose,
                            self.planner.config.radius,
                            p.x,
                            p.y,
                        )
                    {
                        ok = self
                            .follower
                            .accept(
                                &GoalCommand::Local {
                                    x: p.x,
                                    y: p.y,
                                    yaw: None,
                                    token: Some(r.token.clone()),
                                },
                                20_000.,
                            )
                            .unwrap_or(false);
                        if ok {
                            self.proposal = None;
                            self.paused = false;
                        }
                    }
                }
            }
        }
        self.acknowledge(
            r.token,
            fp,
            ok,
            if ok {
                match r.decision {
                    Decision::Approve => "proposal_approve",
                    Decision::Reject => "proposal_reject",
                    Decision::Resume => "supervision_resume",
                }
            } else {
                "stale_or_invalid_proposal"
            },
        );
    }
    /// Advertise up to 64 valid classes and a valid version token at nondecreasing `now` seconds.
    ///
    /// Returns `invalid_detector_capability` for invalid labels, version, or time, leaving the previous capability intact.
    pub fn set_detector_capability(
        &mut self,
        classes: Vec<String>,
        version: String,
        now: f64,
    ) -> Result<(), SearchError> {
        if classes.len() > 64
            || !classes.iter().all(|s| valid_class(s))
            || !valid_token(&version)
            || !now.is_finite()
            || now < 0.
            || self.detector_time.is_some_and(|t| now < t)
        {
            return Err("invalid_detector_capability");
        }
        self.detector_classes = classes;
        self.detector_version = version;
        self.detector_time = Some(now);
        Ok(())
    }
    /// Return whether `class` is advertised and the capability timestamp is at most 0.5 seconds old.
    pub fn detector_available(&self, class: &str, now: f64) -> bool {
        self.detector_classes.iter().any(|c| c == class) && fresh(now, self.detector_time, 0.5)
    }
    /// Return the current session snapshot and detector freshness, or `None` before a search is created.
    pub fn search_status(&self, now: f64) -> Option<SearchStatus> {
        self.search
            .as_ref()
            .map(|s| s.status(now, self.detector_available(&s.request.target_class, now)))
    }
    /// Start a validated search under target-search authority with fresh healthy pose, map, and detector evidence.
    ///
    /// Returns a machine-readable rejection for correlation, capacity, reused session, bounds, health, or authority failures.
    /// Matching accepted duplicate tokens are idempotent and do not restart the session.
    pub fn start_search(
        &mut self,
        r: SearchRequest,
        input: &ArbiterInput<'_>,
    ) -> Result<(), SearchError> {
        if !r.valid() {
            return Err("invalid_search");
        }
        let fp = format!("search:{}", serde_json::to_string(&r).unwrap());
        if self.duplicate(&r.token, &fp) {
            return if self.receipt.as_ref().is_some_and(|r| r.1) {
                Ok(())
            } else {
                Err("conflicting_or_rejected_token")
            };
        }
        let reason = if r.run_id != self.run_id {
            Some("wrong_run")
        } else if self.level != Level::TargetSearch || self.stopped {
            Some("authority_required")
        } else if self.search.as_ref().is_some_and(|s| !s.phase().terminal()) {
            Some("search_active")
        } else if self.search_sessions.contains(&r.search_id) {
            Some("search_id_reused")
        } else if self.search_sessions.len() >= 1024 {
            Some("search_session_capacity")
        } else if self.pending_reports.len() >= 128 {
            Some("report_capacity")
        } else if !self.detector_available(&r.target_class, input.now) {
            Some("detector_unavailable")
        } else if input.pose.x - self.planner.config.radius < r.bounds.min_x
            || input.pose.x + self.planner.config.radius > r.bounds.max_x
            || input.pose.y - self.planner.config.radius < r.bounds.min_y
            || input.pose.y + self.planner.config.radius > r.bounds.max_y
        {
            Some("outside_search_bounds")
        } else if !input.healthy
            || !input.pose.finite()
            || !fresh(input.now, Some(input.pose_time), 0.5)
            || input.map.is_none()
            || !fresh(input.now, input.map_time, 0.5)
            || self.last_time.is_some_and(|t| input.now < t)
        {
            Some("search_inputs_unhealthy")
        } else {
            None
        };
        if let Some(reason) = reason {
            self.acknowledge(r.token, fp, false, reason);
            return Err(reason);
        }
        self.collect_search_report(input.now);
        let search = SearchController::with_config(
            r.clone(),
            input.now,
            ConfirmationConfig::default(),
            self.planner.config.radius,
        )?;
        self.follower.cancel();
        self.last_plan = None;
        self.planner_trace = None;
        self.search_sessions.push_back(r.search_id.clone());
        self.search = Some(search);
        self.paused = false;
        self.acknowledge(r.token.clone(), fp, true, "search_started");
        self.event("search_accepted", Some(r.token), "search_started");
        Ok(())
    }
    /// Apply an action to the matching active run and search, with token idempotency.
    ///
    /// Resume additionally requires fresh healthy pose, map, and detector evidence.
    /// Returns validation, correlation, authority, health, clock, or terminal-state errors.
    pub fn apply_search_action(
        &mut self,
        r: SearchActionRequest,
        input: &ArbiterInput<'_>,
    ) -> Result<(), SearchError> {
        if !r.valid() {
            return Err("invalid_search_action");
        }
        let fp = format!("search_action:{}", serde_json::to_string(&r).unwrap());
        if self.duplicate(&r.token, &fp) {
            return if self.receipt.as_ref().is_some_and(|r| r.1) {
                Ok(())
            } else {
                Err("conflicting_or_rejected_token")
            };
        }
        let valid =
            self.search.as_ref().is_some_and(|s| {
                s.request.search_id == r.search_id && s.request.run_id == r.run_id
            }) && self.run_id == r.run_id
                && self.level == Level::TargetSearch
                && !self.stopped;
        if !valid {
            self.acknowledge(r.token, fp, false, "wrong_search_or_authority");
            return Err("wrong_search_or_authority");
        }
        if r.action == SearchAction::Resume
            && (!input.healthy
                || input.map.is_none()
                || !fresh(input.now, Some(input.pose_time), 0.5)
                || !fresh(input.now, input.map_time, 0.5)
                || !self.detector_available(
                    &self.search.as_ref().unwrap().request.target_class,
                    input.now,
                ))
        {
            self.acknowledge(r.token, fp, false, "search_inputs_unhealthy");
            return Err("search_inputs_unhealthy");
        }
        let result = self.search.as_mut().unwrap().action(r.action, input.now);
        self.acknowledge(
            r.token.clone(),
            fp,
            result.is_ok(),
            result.err().unwrap_or("search_action"),
        );
        self.search.as_mut().unwrap().receipt = Some((r.token.clone(), result.is_ok()));
        if result.is_ok() {
            self.follower.cancel();
            self.last_plan = None;
            self.planner_trace = None;
            self.reset = true;
            self.event(
                match r.action {
                    SearchAction::Pause => "search_paused",
                    SearchAction::Resume => "search_resumed",
                    SearchAction::Cancel => "search_cancelled",
                },
                Some(r.token),
                "operator_search_action",
            );
        }
        result
    }
    /// Forward an observation only under target-search authority with a fresh matching detector.
    ///
    /// Returns detector/authority, missing-search, or controller observation errors; a new report is queued on confirmation.
    pub fn accept_target_observation(
        &mut self,
        o: TargetObservation,
        now: f64,
    ) -> Result<(), SearchError> {
        if self.level != Level::TargetSearch
            || self.stopped
            || !self.detector_available(&o.target_class, now)
        {
            return Err("detector_or_authority_unavailable");
        }
        let result = self.search.as_mut().ok_or("no_search")?.observe(o, now);
        self.collect_search_report(now);
        self.event(
            if result.is_ok() {
                "candidate_observed"
            } else {
                "candidate_rejected"
            },
            None,
            result.err().unwrap_or("observed_target"),
        );
        result
    }
    fn collect_search_report(&mut self, now: f64) {
        let report = self.search_status(now).and_then(|s| s.report);
        if let Some(report) = report
            && self
                .reported_sessions
                .insert((report.run_id.clone(), report.search_id.clone()))
        {
            self.pending_reports.push_back(report);
            self.event("target_confirmed", None, "target_confirmed");
            self.event("report_queued", None, "report_pending");
        }
    }
    /// Remove a report matching the version, run, search, and report IDs.
    ///
    /// Returns `false` for malformed or unknown acknowledgements; repeats of a previously acknowledged report return `true`.
    pub fn ack_search_report(&mut self, r: SearchReportAck) -> bool {
        if r.version != 1
            || (![&r.run_id, &r.search_id, &r.token]
                .iter()
                .all(|s| valid_token(s))
                || !valid_report_id(&r.report_id))
        {
            return false;
        }
        if let Some(i) = self.pending_reports.iter().position(|p| {
            p.run_id == r.run_id && p.search_id == r.search_id && p.report_id == r.report_id
        }) {
            self.pending_reports.remove(i);
            self.acknowledged_reports.insert((
                r.run_id.clone(),
                r.search_id.clone(),
                r.report_id.clone(),
            ));
            self.event("report_acknowledged", Some(r.token), "report_delivered");
            true
        } else {
            self.acknowledged_reports
                .contains(&(r.run_id, r.search_id, r.report_id))
        }
    }
    /// Run one authority/safety tick and return intent, allowed motion, and status.
    /// Consumers must use the returned `twist`, not bypass holds with `intent`.
    pub fn step(&mut self, input: ArbiterInput<'_>) -> ArbiterOutput {
        let command_origin = if matches!(self.level, Level::Teleop | Level::AssistedTeleop) {
            self.operator_origin
        } else {
            CommandOrigin::Autonomous
        };
        let managed = command_origin != CommandOrigin::LocalManual;
        let identity = (self.run_id.clone(), self.revision, command_origin);
        if self.turning_identity.as_ref() != Some(&identity) {
            self.turning.reset();
            self.last_plan = None;
            self.maneuver.clear();
            self.turning_identity = Some(identity);
        }
        let valid_time = input.now.is_finite()
            && input.now >= 0.
            && self.last_time.is_none_or(|t| input.now > t);
        if valid_time {
            self.last_time = Some(input.now);
        }
        if valid_time {
            if let Some(search) = self.search.as_mut() {
                let _ = search.update_time(input.now);
                if search.request.run_id != self.run_id {
                    search.cancel("run_changed");
                }
            }
        }
        let mut goal = self
            .follower
            .step(input.pose.x, input.pose.y, input.pose.yaw)
            .status;
        let mut reason = "idle";
        let mut source = "none";
        let mut safety = "clear";
        let mut intent_command = Twist::default();
        let mut twist = Twist::default();
        let mut held = false;
        let mut route_failure = None;
        let bounded_map = if self.level == Level::TargetSearch {
            self.search.as_ref().and_then(|s| {
                input.map.map(|map| {
                    let mut m = map.clone();
                    let b = s.request.bounds;
                    for y in 0..m.height {
                        for x in 0..m.width {
                            let px = m.origin_x + x as f64 * m.resolution;
                            let py = m.origin_y + y as f64 * m.resolution;
                            if px < b.min_x
                                || py < b.min_y
                                || px + m.resolution > b.max_x
                                || py + m.resolution > b.max_y
                            {
                                m.occupancy[(y * m.width + x) as usize] = 100;
                            }
                        }
                    }
                    m
                })
            })
        } else {
            None
        };
        let planning_map = bounded_map.as_ref().or(input.map);
        if !valid_time {
            reason = "invalid_time";
            held = true;
        } else if self.stopped {
            safety = "emergency_stop";
            reason = "operator_stop";
            held = true;
            self.halt_explore(EndReason::OperatorStop);
        } else if !input.healthy
            || (!self.is_manual() && (!input.pose.finite()
                || !fresh(input.now, Some(input.pose_time), 0.5)
                || !input.measured.linear.is_finite()
                || !input.measured.angular.is_finite()))
        {
            reason = "sensor_unhealthy";
            held = true;
            self.note_safety_hold(input.now);
        } else if !matches!(self.level, Level::Teleop | Level::WaypointDirect)
            && (input.map.is_none() || !fresh(input.now, input.map_time, 0.5))
        {
            reason = "map_stale";
            held = true;
            self.note_map_stale(input.now);
        } else if self.paused {
            reason = "supervision_paused";
            held = true;
        } else {
            self.explore_stale_since = None;
            self.explore_unhealthy_since = None;
            let explore_hold = if self.level == Level::Explore {
                let why = self.drive_explore(&input);
                goal = self
                    .follower
                    .step(input.pose.x, input.pose.y, input.pose.yaw)
                    .status;
                why
            } else {
                None
            };
            if let Some(why) = explore_hold {
                reason = why;
                held = true;
                self.explore_blocked = false;
            } else {
                if self.level == Level::TargetSearch {
                    let detector = self.search.as_ref().is_some_and(|s| {
                        self.detector_available(&s.request.target_class, input.now)
                    });
                    let mut search_goal = None;
                    if let Some(search) = self.search.as_mut() {
                        if !detector {
                            search.hold("detector_unavailable");
                        } else {
                            search_goal = search
                                .tick(
                                    input.map.unwrap(),
                                    input.pose,
                                    input.map_revision,
                                    input.now,
                                )
                                .unwrap_or(None);
                        }
                    }
                    if let Some((x, y)) = search_goal {
                        if (goal.x - x).abs() > 1e-8
                            || (goal.y - y).abs() > 1e-8
                            || goal.state != GoalState::Active
                        {
                            let _ = self.follower.accept(
                                &GoalCommand::Local {
                                    x,
                                    y,
                                    yaw: None,
                                    token: None,
                                },
                                20_000.,
                            );
                            self.last_plan = None;
                            self.planner_trace = None;
                            self.event("route_selected", None, "search_route");
                        }
                        goal = self
                            .follower
                            .step(input.pose.x, input.pose.y, input.pose.yaw)
                            .status;
                    } else {
                        self.follower.cancel();
                        self.last_plan = None;
                        self.planner_trace = None;
                        held = true;
                        reason = "search_hold";
                    }
                }
                let mut guidance_heading = (goal.y - input.pose.y).atan2(goal.x - input.pose.x);
                let mut intent = match self.level {
                    Level::Teleop | Level::AssistedTeleop => self
                        .operator
                        .as_ref()
                        .filter(|(_, t)| fresh(input.now, Some(*t), 0.5))
                        .map(|(r, _)| {
                            source = if self.level == Level::Teleop {
                                "operator"
                            } else {
                                "assisted_operator"
                            };
                            Twist {
                                linear: r.linear.clamp(-2., 2.),
                                angular: r.angular.clamp(-2., 2.),
                            }
                        })
                        .unwrap_or_default(),
                    _ => {
                        let out = self
                            .follower
                            .step(input.pose.x, input.pose.y, input.pose.yaw);
                        if out.status.state == GoalState::Active {
                            source = if self.level == Level::Supervised {
                                "frontier"
                            } else if self.level == Level::Explore {
                                "explore"
                            } else if self.level == Level::TargetSearch {
                                "target_search"
                            } else {
                                "waypoint"
                            };
                        }
                        if self.level == Level::Waypoint && out.status.state == GoalState::Active {
                            let map = planning_map.unwrap();
                            let radius = self.planner.config.radius;
                            let destination = (out.status.goal_id, out.status.x, out.status.y);
                            let here = [input.pose.x, input.pose.y];
                            // Drop reached route points; never use a distant point without checking line of sight.
                            while self.global_route.len() > 1
                                && (self.global_route[0][0] - here[0])
                                    .hypot(self.global_route[0][1] - here[1])
                                    < 0.2
                            {
                                self.global_route.remove(0);
                            }
                            let changed_goal = self.route_goal != Some(destination);
                            let due = input.now - self.route_checked_at >= 0.2;
                            let segment_blocked = self.route_map_revision != input.map_revision
                                && !std::iter::once(here)
                                    .chain(self.global_route.iter().copied())
                                    .collect::<Vec<_>>()
                                    .windows(2)
                                    .all(|p| {
                                        terra_navigation::route_segment_clear(
                                            map, input.pose, radius, p[0], p[1],
                                        )
                                    });
                            // Approach routes stop at the frontier. Replan on the same
                            // interval so a rolling map can extend them toward the goal.
                            let needs_route = changed_goal
                                || (due
                                    && (self.global_route.is_empty()
                                        || !self.route_reaches_goal
                                        || segment_blocked));
                            if needs_route {
                                self.last_plan = None;
                                self.planner_trace = None;
                                self.route_goal = Some(destination);
                                self.global_route = match terra_navigation::astar_progress_route(
                                    map,
                                    input.pose,
                                    radius,
                                    out.status.x,
                                    out.status.y,
                                ) {
                                    Ok(route) => {
                                        self.route_reaches_goal =
                                            route.last().is_some_and(|point| {
                                                (point[0] - destination.1).abs() <= 1e-9
                                                    && (point[1] - destination.2).abs() <= 1e-9
                                            });
                                        route
                                    }
                                    Err(why) => {
                                        self.route_reaches_goal = false;
                                        route_failure = Some(why);
                                        Vec::new()
                                    }
                                };
                            }
                            if due || changed_goal {
                                self.route_checked_at = input.now;
                                self.route_map_revision = input.map_revision;
                            }
                            let target = self.global_route.iter().rev().copied().find(|point| {
                                terra_navigation::route_segment_clear(
                                    map, input.pose, radius, here, *point,
                                )
                            });
                            if let Some(point) = target {
                                guidance_heading =
                                    (point[1] - input.pose.y).atan2(point[0] - input.pose.x);
                                // At the final position use the original follower for arrival and final yaw.
                                let (linear, angular) = if point == [out.status.x, out.status.y] {
                                    (out.linear, out.angular)
                                } else {
                                    self.follower.route_intent(
                                        input.pose.x,
                                        input.pose.y,
                                        input.pose.yaw,
                                        point[0],
                                        point[1],
                                    )
                                };
                                Twist { linear, angular }
                            } else {
                                // Localization may move us away from every cached visible point,
                                // even without a map revision change. Retry A* at the bounded interval.
                                self.global_route.clear();
                                self.route_reaches_goal = false;
                                route_failure = Some(route_failure.unwrap_or("route_no_path"));
                                Twist::default()
                            }
                        } else {
                            if self.level == Level::Waypoint {
                                self.global_route.clear();
                                self.route_goal = None;
                                self.route_reaches_goal = false;
                            }
                            Twist {
                                linear: out.linear,
                                angular: out.angular,
                            }
                        }
                    }
                };
                if managed {
                    intent.linear = intent.linear.clamp(
                        -self
                            .planner
                            .config
                            .max_reverse
                            .min(self.planner.config.max_linear),
                        self.planner.config.max_linear,
                    );
                    intent.angular = intent.angular.clamp(
                        -self.planner.config.max_angular,
                        self.planner.config.max_angular,
                    );
                }
                intent_command = intent;
                let mut maneuver_active = false;
                if managed
                    && command_origin == CommandOrigin::Autonomous
                    && !self.turning_config.allow_in_place
                {
                    if self.maneuver_goal != Some(goal.goal_id) || goal.state != GoalState::Active {
                        self.maneuver.clear();
                    }
                    // Segment completion is measured heading, not nominal rollout time.
                    while self
                        .maneuver
                        .front()
                        .is_some_and(|s| wrap_heading(s.end.yaw - input.pose.yaw).abs() < 0.08)
                    {
                        self.maneuver.pop_front();
                    }
                    let needs_turn = goal.state == GoalState::Active
                        && intent.linear.abs() < 1e-6
                        && intent.angular.abs() > 1e-6;
                    if self.maneuver.is_empty() && needs_turn {
                        if let Some(map) =
                            planning_map.filter(|_| fresh(input.now, input.map_time, 0.5))
                        {
                            let target = if goal.distance <= 0.75 {
                                goal.yaw.unwrap_or(input.pose.yaw)
                            } else {
                                guidance_heading
                            };
                            if let Some(segments) = self.planner.heading_maneuver(
                                map,
                                input.pose,
                                target,
                                &self.turning_config,
                            ) {
                                self.maneuver = segments.into();
                                self.maneuver_goal = Some(goal.goal_id);
                            }
                        }
                    }
                    if let Some(segment) = self.maneuver.front() {
                        if fresh(input.now, input.map_time, 0.5) && planning_map.is_some() {
                            intent = segment.velocity;
                            maneuver_active = true;
                        } else {
                            held = true;
                            reason = "map_stale";
                            self.maneuver.clear();
                        }
                    } else if needs_turn {
                        held = true;
                        reason = "heading_requires_spin_or_replan";
                    }
                }
                if let Some(why) = route_failure {
                    held = true;
                    reason = why;
                    source = "none";
                    self.last_plan = None;
                    self.planner_trace = None;
                }
                if maneuver_active && !held {
                    let (planned, mut trace) = self.planner.plan_with_turning(
                        planning_map.unwrap(),
                        input.pose,
                        input.measured,
                        intent,
                        0.1,
                        &self.turning,
                        input.now,
                    );
                    trace.maneuver_segments = self.maneuver.iter().cloned().collect();
                    twist = planned.twist;
                    reason = planned.reason;
                    self.planner_trace = Some((input.now, trace));
                    if twist == Twist::default() || reason == "obstacle_blocked" {
                        held = true;
                        reason = "heading_requires_spin_or_replan";
                        self.maneuver.clear();
                    }
                } else if !held && matches!(self.level, Level::Teleop | Level::WaypointDirect) {
                    twist = intent;
                    if source != "none" {
                        reason = "active";
                    }
                } else if !held && source != "none" {
                    if intent == Twist::default() {
                        twist = intent;
                        reason = "active";
                        self.last_plan = None;
                        self.planner_trace = None;
                    } else if let Some((at, output, why)) = self.last_plan.filter(|p| {
                        !managed
                            && input.now - p.0 < 0.1
                            && planning_map
                                .is_some_and(|m| self.planner.admissible(m, input.pose, p.1))
                    }) {
                        let _ = at;
                        twist = output;
                        reason = why;
                    } else {
                        let mut planner = self.planner.clone();
                        if self.level == Level::Explore {
                            let cap = self.explorer.config().approach_horizon;
                            if cap.is_finite() && cap > 0. {
                                planner.config.horizon = planner.config.horizon.min(cap);
                            }
                        }
                        let (planned, trace) = if managed {
                            planner.plan_with_turning(
                                planning_map.unwrap(),
                                input.pose,
                                input.measured,
                                intent,
                                0.1,
                                &self.turning,
                                input.now,
                            )
                        } else {
                            planner.plan_with_trace(
                                planning_map.unwrap(),
                                input.pose,
                                input.measured,
                                intent,
                                0.1,
                            )
                        };
                        self.planner_trace =
                            (!trace.candidates.is_empty()).then_some((input.now, trace));
                        twist = planned.twist;
                        reason = planned.reason;
                        self.last_plan = Some((input.now, twist, reason));
                    }
                    if reason == "obstacle_blocked" {
                        held = true;
                        source = "none";
                    }
                }

                if self.level == Level::Supervised && goal.state != GoalState::Active {
                    self.rejected.retain(|(_, _, expiry)| *expiry > input.now);
                    if self
                        .proposal
                        .as_ref()
                        .is_some_and(|p| p.expires_at <= input.now)
                    {
                        self.proposal = None;
                    }
                    if self.proposal.is_none() && input.now - self.last_frontier >= 0.1 {
                        self.last_frontier = input.now;
                        let excluded: Vec<_> =
                            self.rejected.iter().map(|&(x, y, _)| (x, y)).collect();
                        if let Some((x, y)) = frontier(
                            input.map.unwrap(),
                            input.pose,
                            self.planner.config.radius,
                            &excluded,
                        ) {
                            self.next_proposal += 1;
                            self.proposal = Some(GoalProposal {
                                proposal_id: self.next_proposal,
                                run_id: self.run_id.clone(),
                                x,
                                y,
                                map_revision: input.map_revision,
                                expires_at: input.now + 30.,
                                reason: "search_unobserved_sector".into(),
                            });
                            self.event("proposal_created", None, "frontier");
                        }
                    }
                    reason = if self.proposal.is_some() {
                        "awaiting_approval"
                    } else {
                        "no_reachable_frontier"
                    };
                }
                if self.level == Level::Explore {
                    self.explore_blocked = reason == "obstacle_blocked";
                }
            }
        }
        if managed && !held {
            if source == "none" || twist == Twist::default() {
                self.turning.reset();
                if command_origin == CommandOrigin::Autonomous
                    && intent_command.angular != 0.
                    && !self.turning_config.allow_in_place
                {
                    held = true;
                    reason = "heading_requires_spin_or_replan";
                }
            } else {
                match self.turning.step(terra_types::VelocityTarget {
                    timestamp: input.now,
                    forward: twist.linear,
                    yaw_rate: twist.angular,
                }) {
                    Ok(t) => {
                        twist = Twist {
                            linear: t.forward,
                            angular: t.yaw_rate,
                        };
                        if command_origin == CommandOrigin::Autonomous
                            && twist == Twist::default()
                            && intent_command.angular != 0.
                        {
                            held = true;
                            reason = "heading_requires_spin_or_replan";
                        }
                    }
                    Err(_) => {
                        held = true;
                        reason = "invalid_turn_timing";
                    }
                }
            }
        }
        if held {
            self.turning.reset();
            self.maneuver.clear();
        }
        if (!input.healthy || !fresh(input.now, Some(input.pose_time), 0.5) || !valid_time)
            && goal.state == GoalState::Arrived
        {
            goal.state = GoalState::Active;
        }
        if held && self.level == Level::TargetSearch && reason == "obstacle_blocked" {
            if let Some(search) = self.search.as_mut() {
                search.navigator.invalidate_goal(input.now);
            }
            self.last_plan = None;
            // Retire cached motion, but keep fresh diagnostics explaining the route failure.
            self.event("route_failed", None, "obstacle_blocked");
        }
        if held
            && self.level == Level::TargetSearch
            && reason != "search_hold"
            && reason != "obstacle_blocked"
        {
            if let Some(search) = self.search.as_mut() {
                search.hold(reason);
            }
            self.follower.cancel();
        }
        self.collect_search_report(input.now);
        let search_status = self.search_status(input.now);
        if let Some(status) = search_status.as_ref() {
            if self.level == Level::TargetSearch && status.phase != SearchPhase::Searching {
                held = true;
                reason = "search_hold";
            }
        }
        if held {
            if reason != "obstacle_blocked" {
                self.last_plan = None;
                self.planner_trace = None;
            }
            twist = Twist::default();
            source = "none";
            self.reset = true;
            if safety == "clear" {
                safety = "hold";
            }
        }
        let status = AutonomyStatus {
            global_route: if self.level == Level::Waypoint
                && route_failure.is_none()
                && valid_time
                && !self.stopped
                && !self.paused
                && input.healthy
                && fresh(input.now, input.map_time, 0.5)
                && goal.state == GoalState::Active
            {
                self.global_route.clone()
            } else {
                Vec::new()
            },
            planner_trace: self
                .planner_trace
                .as_ref()
                .filter(|(at, _)| input.now - at >= 0. && input.now - at < 0.5)
                .map(|(at, trace)| {
                    let mut trace = trace.clone();
                    trace.age = input.now - at;
                    trace
                }),
            run_id: self.run_id.clone(),
            assigned_level: self.assigned_level,
            requested_level: self.level,
            effective_level: if held { None } else { Some(self.level) },
            active_source: source.into(),
            safety: safety.into(),
            reason: reason.into(),
            revision: self.revision,
            token: self.receipt.as_ref().map(|r| r.0.clone()),
            result: self
                .receipt
                .as_ref()
                .map(|r| if r.1 { "accepted" } else { "rejected" }.into()),
            request_reason: self.request_reason.clone(),
            supported_levels: {
                let mut levels = vec![
                    Level::Teleop,
                    Level::AssistedTeleop,
                    Level::Waypoint,
                    Level::WaypointDirect,
                    Level::Supervised,
                    Level::Explore,
                ];
                if !self.detector_classes.is_empty() && fresh(input.now, self.detector_time, 0.5) {
                    levels.push(Level::TargetSearch);
                }
                levels
            },
            paused: self.paused,
            exploration: self.exploration_report(input.now),
        };
        if self.level == Level::TargetSearch {
            goal = self
                .follower
                .step(input.pose.x, input.pose.y, input.pose.yaw)
                .status;
        }
        let exploration = status.exploration.clone();
        self.drain_explore_events();
        ArbiterOutput {
            command_origin,
            intent: intent_command,
            twist,
            status,
            goal,
            proposal: self.proposal.clone(),
            exploration,
            events: std::mem::take(&mut self.events),
            reset_controller: std::mem::take(&mut self.reset),
            search: search_status,
            pending_reports: self.pending_reports.iter().cloned().collect(),
        }
    }
    fn exploration_report(&self, now: f64) -> Option<ExplorationStatus> {
        let status = self.explorer.status(now);
        (status.phase != terra_exploration::ExplorePhase::Idle).then_some(status)
    }
    fn drain_explore_events(&mut self) {
        for event in self.explorer.take_events() {
            self.event(event.kind, None, &event.reason);
        }
    }
    fn halt_explore(&mut self, reason: EndReason) {
        if self.level == Level::Explore {
            self.explorer.stop(reason);
        }
    }
    fn note_map_stale(&mut self, now: f64) {
        if self.level == Level::Explore
            && self.explorer.is_running()
            && sustained(
                &mut self.explore_stale_since,
                now,
                self.explorer.config().map_stale_timeout,
            )
        {
            self.explorer.stop(EndReason::MapStale);
        }
    }
    fn note_safety_hold(&mut self, now: f64) {
        if self.level == Level::Explore
            && self.explorer.is_running()
            && sustained(
                &mut self.explore_unhealthy_since,
                now,
                self.explorer.config().safety_hold_timeout,
            )
        {
            self.explorer.stop(EndReason::SafetyHold);
        }
    }
    /// `Some` when exploration itself is holding or finished. `None` when the follower should move.
    fn drive_explore(&mut self, input: &ArbiterInput<'_>) -> Option<&'static str> {
        self.explorer.set_footprint(self.planner.config.radius);
        let preview = self
            .follower
            .step(input.pose.x, input.pose.y, input.pose.yaw);
        let action = self.explorer.step(ExploreTick {
            now: input.now,
            pose: input.pose,
            map: input.map,
            planner_blocked: self.explore_blocked,
            goal_active: preview.status.state == GoalState::Active,
            goal_arrived: preview.status.state == GoalState::Arrived,
            goal_x: preview.status.x,
            goal_y: preview.status.y,
            goal_distance: preview.status.distance,
        });
        match action {
            ExploreAction::Seek { x, y } => {
                let same = matches!(preview.status.state, GoalState::Active | GoalState::Arrived)
                    && (preview.status.x - x).hypot(preview.status.y - y) < 0.05;
                if !same {
                    let token = format!("explore-{}", self.explorer.frontiers_attempted());
                    let _ = self.follower.accept(
                        &GoalCommand::Local {
                            x,
                            y,
                            yaw: None,
                            token: Some(token),
                        },
                        20_000.,
                    );
                    self.last_plan = None;
                }
                None
            }
            ExploreAction::Hold => {
                self.follower.cancel();
                self.last_plan = None;
                Some("frontier_retry")
            }
            ExploreAction::Finished(reason) => {
                self.follower.cancel();
                self.last_plan = None;
                Some(reason.as_str())
            }
            ExploreAction::Idle => Some("exploration_idle"),
        }
    }
}
fn sustained(since: &mut Option<f64>, now: f64, timeout: f64) -> bool {
    if !now.is_finite() {
        return false;
    }
    let start = *since.get_or_insert(now);
    now - start >= timeout.max(0.)
}
fn fresh(now: f64, time: Option<f64>, limit: f64) -> bool {
    time.is_some_and(|t| t.is_finite() && t >= 0. && now >= t && now - t < limit)
}

fn wrap_heading(x: f64) -> f64 {
    (x + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}
