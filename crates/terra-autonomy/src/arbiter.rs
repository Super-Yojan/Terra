use crate::*;
use serde::Serialize;
use std::collections::VecDeque;
use terra_exploration::{EndReason, ExplorationStatus, ExploreAction, ExploreTick, Explorer};
use terra_mapping::MapSnapshot;
use terra_navigation::{LocalPlanner, Pose, Twist, frontier};
use terra_waypoint::{GoalCommand, GoalState, WaypointConfig, WaypointController, WaypointStatus};
#[derive(Clone, Debug, Serialize)]
pub struct DecisionEvent {
    pub kind: String,
    pub token: Option<String>,
    pub reason: String,
    pub level: Level,
}
pub struct ArbiterInput<'a> {
    pub now: f64,
    pub pose: Pose,
    pub pose_time: f64,
    pub map: Option<&'a MapSnapshot>,
    pub map_time: Option<f64>,
    pub map_revision: u64,
    pub measured: Twist,
    pub healthy: bool,
}
pub struct ArbiterOutput {
    pub intent: Twist,
    pub twist: Twist,
    pub status: AutonomyStatus,
    pub goal: WaypointStatus,
    pub proposal: Option<GoalProposal>,
    pub exploration: Option<ExplorationStatus>,
    pub events: Vec<DecisionEvent>,
    pub reset_controller: bool,
}
pub struct AutonomyArbiter {
    pub planner: LocalPlanner,
    pub assigned_level: Option<Level>,
    level: Level,
    follower: WaypointController,
    operator: Option<(TeleopRequest, f64)>,
    operator_order: std::collections::BTreeMap<String, u64>,
    latest_operator_time: Option<f64>,
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
    last_frontier: f64,
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
            last_frontier: -1.,
            explorer: Explorer::default(),
            exploration_budget: None,
            explore_blocked: false,
            explore_stale_since: None,
            explore_unhealthy_since: None,
        }
    }
}
impl AutonomyArbiter {
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
    fn clear(&mut self) {
        self.operator = None;
        self.follower.cancel();
        self.proposal = None;
        self.reset = true;
        self.last_plan = None;
    }
    /// Store a budget used when an `explore` level request omits one. `seconds` must be in `(0, 86400]`.
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
    pub fn set_level(&mut self, r: LevelRequest) {
        let fp = format!(
            "level:{:?}:{:?}:{:?}",
            r.level, r.budget_seconds, r.budget_minutes
        );
        if !valid_token(&r.token) || self.duplicate(&r.token, &fp) {
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
    pub fn accept_operator(&mut self, r: TeleopRequest, now: f64) {
        if !now.is_finite()
            || now < 0.
            || self.last_time.is_some_and(|t| now < t)
            || self.latest_operator_time.is_some_and(|t| now < t)
            || !r.linear.is_finite()
            || !r.angular.is_finite()
            || r.linear.abs() > 5.
            || r.angular.abs() > 5.
        {
            return;
        }
        if r.run_id.as_ref().is_some_and(|run| run != &self.run_id)
            || r.authority_revision
                .is_some_and(|revision| revision != self.revision)
        {
            return;
        }
        if self.stopped || !matches!(self.level, Level::Teleop | Level::AssistedTeleop) {
            return;
        }
        if let (Some(session), Some(sequence)) = (&r.operator_session_id, r.sequence) {
            if self
                .operator_order
                .get(session)
                .is_some_and(|last| sequence <= *last)
            {
                return;
            }
            if self.operator_order.len() >= 128 && !self.operator_order.contains_key(session) {
                return;
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
    }
    pub fn accept_goal(&mut self, g: &GoalCommand, extent: f64) -> bool {
        if matches!(g, GoalCommand::Cancel) {
            if self.level == Level::Explore && self.explorer.is_running() {
                self.explorer.stop(EndReason::OperatorStop);
            }
            self.clear();
            if self.level == Level::Waypoint {
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
        if self.stopped || !matches!(self.level, Level::Waypoint | Level::Supervised) {
            if let Some(t) = token.clone() {
                self.acknowledge(t, fingerprint, false, "authority_required");
            }
            self.event("goal_rejected", token, "authority_required");
            return false;
        }
        let ok = self.follower.accept(g, extent).unwrap_or(false);
        if ok {
            self.last_plan = None;
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
    pub fn step(&mut self, input: ArbiterInput<'_>) -> ArbiterOutput {
        let valid_time = input.now.is_finite()
            && input.now >= 0.
            && self.last_time.is_none_or(|t| input.now > t);
        if valid_time {
            self.last_time = Some(input.now);
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
        if !valid_time {
            reason = "invalid_time";
            held = true;
        } else if self.stopped {
            safety = "emergency_stop";
            reason = "operator_stop";
            held = true;
            self.halt_explore(EndReason::OperatorStop);
        } else if !input.healthy
            || !input.pose.finite()
            || !fresh(input.now, Some(input.pose_time), 0.5)
            || !input.measured.linear.is_finite()
            || !input.measured.angular.is_finite()
        {
            reason = "sensor_unhealthy";
            held = true;
            self.note_safety_hold(input.now);
        } else if self.level != Level::Teleop
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
                let intent = match self.level {
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
                            source = match self.level {
                                Level::Supervised => "frontier",
                                Level::Explore => "explore",
                                _ => "waypoint",
                            };
                        }
                        Twist {
                            linear: out.linear,
                            angular: out.angular,
                        }
                    }
                };
                intent_command = intent;
                if self.level == Level::Teleop {
                    twist = intent;
                    if source != "none" {
                        reason = "active";
                    }
                } else if source != "none" {
                    if intent == Twist::default() {
                        twist = intent;
                        reason = "active";
                        self.last_plan = None;
                    } else if let Some((at, output, why)) =
                        self.last_plan.filter(|p| input.now - p.0 < 0.1)
                    {
                        let _ = at;
                        twist = output;
                        reason = why;
                    } else {
                        let planned = if self.level == Level::Explore {
                            let mut planner = self.planner.clone();
                            let cap = self.explorer.config().approach_horizon;
                            if cap.is_finite() && cap > 0. {
                                planner.config.horizon = planner.config.horizon.min(cap);
                            }
                            planner.plan(
                                input.map.unwrap(),
                                input.pose,
                                input.measured,
                                intent,
                                0.1,
                            )
                        } else {
                            self.planner.plan(
                                input.map.unwrap(),
                                input.pose,
                                input.measured,
                                intent,
                                0.1,
                            )
                        };
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
        if held {
            if reason != "obstacle_blocked" {
                self.last_plan = None;
            }
            twist = Twist::default();
            source = "none";
            self.reset = true;
            if safety == "clear" {
                safety = "hold";
            }
        }
        let status = AutonomyStatus {
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
            supported_levels: vec![
                Level::Teleop,
                Level::AssistedTeleop,
                Level::Waypoint,
                Level::Supervised,
                Level::Explore,
            ],
            paused: self.paused,
            exploration: self.exploration_report(input.now),
        };
        let exploration = status.exploration.clone();
        self.drain_explore_events();
        ArbiterOutput {
            intent: intent_command,
            twist,
            status,
            goal,
            proposal: self.proposal.clone(),
            exploration,
            events: std::mem::take(&mut self.events),
            reset_controller: std::mem::take(&mut self.reset),
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
