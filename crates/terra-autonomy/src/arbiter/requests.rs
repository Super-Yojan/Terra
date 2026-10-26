//! Apply incoming requests and retain their acknowledgements.
use super::*;

impl AutonomyArbiter {
    pub fn reject_remote_request(&mut self, token: Option<&str>, reason: &str) {
        self.request_reason = Some(reason.into());
        if let Some(token) = token.filter(|token| valid_token(token)) {
            self.receipt = Some((token.into(), false));
        }
        self.event("remote_request_rejected", token.map(str::to_owned), reason);
    }
    /// Latest token-correlated result, including idempotent duplicate requests.
    pub fn request_receipt(&self) -> Option<(&str, bool)> {
        self.receipt
            .as_ref()
            .map(|(token, accepted)| (token.as_str(), *accepted))
    }
    pub fn authority_revision(&self) -> u64 {
        self.revision
    }
    /// Retire previously issued remote commands without changing a local mission.
    pub fn invalidate_remote_authority(&mut self) {
        self.clear_remote_intent();
        self.revision = self.revision.saturating_add(1);
        self.event(
            "authority_generation_changed",
            None,
            "transport_replaced_or_lost",
        );
    }

    /// An already accepted mission, independent of network presence. Tick-time local
    /// sensor, collision and safety checks still apply to every command.
    pub fn accepted_onboard_mission(&self, pose: Pose) -> bool {
        if self.stopped || self.paused {
            return false;
        }
        match self.level {
            Level::Waypoint | Level::WaypointDirect | Level::Supervised => {
                self.has_active_goal(pose)
            }
            Level::Explore => self.explorer.is_running(),
            Level::Task => self.task_running(),
            Level::TargetSearch => self.search.as_ref().is_some_and(|s| {
                matches!(s.phase(), SearchPhase::Searching | SearchPhase::Confirming)
            }),
            _ => false,
        }
    }
    /// Remove remote intent while leaving local manual intent and mission identity alone.
    pub fn clear_remote_intent(&mut self) {
        if self.operator_origin == CommandOrigin::FleetTeleop {
            self.operator = None;
        }
    }
    /// Discard operator input when the actuation profile changes, without replacing an onboard mission.
    pub fn clear_operator_intent(&mut self) {
        self.operator = None;
        self.turning.reset();
        self.turning_identity = None;
        self.reset = true;
    }
    /// Explicit policy pause. Neither a new transport nor a policy edit resumes it.
    pub fn pause_for_router_loss(&mut self) {
        self.clear_remote_intent();
        self.paused = true;
        self.pause_reason = "router_loss_paused";
        self.turning.reset();
        self.maneuver.clear();
        self.reset = true;
        self.event("mission_paused", None, "router_loss_paused");
    }

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
    pub fn is_manual(&self) -> bool {
        self.level == Level::Teleop
    }

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
    pub(super) fn event(&mut self, kind: &str, token: Option<String>, reason: &str) {
        if self.events.len() < 256 {
            self.events.push(DecisionEvent {
                kind: kind.into(),
                token,
                reason: reason.into(),
                level: self.level,
            });
        }
    }
    pub(super) fn duplicate(&mut self, token: &str, fingerprint: &str) -> bool {
        if let Some((_, old, ok)) = self.cache.iter().find(|(t, _, _)| t == token) {
            self.receipt = Some((token.into(), old == fingerprint && *ok));
            true
        } else {
            false
        }
    }
    pub(super) fn acknowledge(
        &mut self,
        token: String,
        fingerprint: String,
        ok: bool,
        reason: &str,
    ) {
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
    pub(super) fn clear(&mut self) {
        self.turning.reset();
        self.turning_identity = None;
        self.maneuver.clear();
        self.maneuver_goal = None;
        // An unfinished L4 look-around must not keep spinning under the next level.
        self.explore_scan = None;
        self.scan_turning = None;
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
        self.set_level_with_task(r, None);
    }
    /// [`Self::set_level`] that also starts `task` when the level is `task`.
    pub fn set_level_with_task(&mut self, r: LevelRequest, task: Option<TaskSpec>) {
        let fp = format!(
            "level:{:?}:{:?}:{:?}:{:?}",
            r.level, r.budget_seconds, r.budget_minutes, task
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
        if r.level == Level::Task && self.task_model.is_none() {
            self.acknowledge(r.token, fp, false, "task_model_unavailable");
            return;
        }
        if task
            .as_ref()
            .is_some_and(|t| r.level != Level::Task || !t.valid())
        {
            self.acknowledge(r.token, fp, false, "invalid_task");
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
        self.end_task(
            TaskPhase::Cancelled,
            if changed {
                "level_changed"
            } else {
                "operator_reselection"
            },
            f64::NAN,
        );
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
        if let Some(task) = task {
            self.begin_task(task, self.last_time.unwrap_or(0.));
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
            // `now` is the receipt time, which can precede the last tick when the
            // control loop runs late; only commands already past freshness are refused.
            || self
                .last_time
                .is_some_and(|t| now < t - timing::OPERATOR_COMMAND_FRESHNESS_SECONDS)
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
            if self.level == Level::Task {
                self.end_task(TaskPhase::Cancelled, "operator_cancel", f64::NAN);
            }
            self.clear();
            if matches!(self.level, Level::Waypoint | Level::WaypointDirect) {
                self.level = Level::Teleop;
            } else if self.level == Level::Supervised {
                self.paused = true;
                self.pause_reason = "supervision_paused";
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
            if r.action == SafetyAction::Stop {
                self.end_task(TaskPhase::Cancelled, "operator_stop", f64::NAN);
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
                    self.rejected
                        .push((p.x, p.y, input.now + timing::REJECTED_PROPOSAL_SECONDS));
                    self.proposal = None;
                    ok = true;
                } else if r.decision == Decision::Approve
                    && self.map_is_fresh(input)
                    && fresh(
                        input.now,
                        Some(input.pose_time),
                        timing::POSE_FRESHNESS_SECONDS,
                    )
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
}
