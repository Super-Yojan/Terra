use crate::*;
use serde::Serialize;
use std::collections::VecDeque;
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
    pub fn set_level(&mut self, r: LevelRequest) {
        let fp = format!("level:{:?}", r.level);
        if !valid_token(&r.token) || self.duplicate(&r.token, &fp) {
            return;
        }
        let changed = self.level != r.level;
        self.clear();
        self.level = r.level;
        self.paused = false;
        if r.level == Level::Teleop {
            self.event("takeover", Some(r.token.clone()), "operator_takeover");
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
        let goal = self
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
        } else if !input.healthy
            || !input.pose.finite()
            || !fresh(input.now, Some(input.pose_time), 0.5)
            || !input.measured.linear.is_finite()
            || !input.measured.angular.is_finite()
        {
            reason = "sensor_unhealthy";
            held = true;
        } else if self.level != Level::Teleop
            && (input.map.is_none() || !fresh(input.now, input.map_time, 0.5))
        {
            reason = "map_stale";
            held = true;
        } else if self.paused {
            reason = "supervision_paused";
            held = true;
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
                        source = if self.level == Level::Supervised {
                            "frontier"
                        } else {
                            "waypoint"
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
                    let planned = self.planner.plan(
                        input.map.unwrap(),
                        input.pose,
                        input.measured,
                        intent,
                        0.1,
                    );
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
                    let excluded: Vec<_> = self.rejected.iter().map(|&(x, y, _)| (x, y)).collect();
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
            ],
            paused: self.paused,
        };
        ArbiterOutput {
            intent: intent_command,
            twist,
            status,
            goal,
            proposal: self.proposal.clone(),
            events: std::mem::take(&mut self.events),
            reset_controller: std::mem::take(&mut self.reset),
        }
    }
}
fn fresh(now: f64, time: Option<f64>, limit: f64) -> bool {
    time.is_some_and(|t| t.is_finite() && t >= 0. && now >= t && now - t < limit)
}
