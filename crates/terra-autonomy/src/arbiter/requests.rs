//! Apply incoming requests and retain their acknowledgements.
use super::*;

impl AutonomyArbiter {
    pub fn reject_remote_request(&mut self, token: Option<&str>, reason: &str) {
        self.request_reason = Some(reason.into());
        if let Some(token) = token.filter(|token| valid_token(token)) {
            self.receipt = Some((token.into(), false));
        }
        self.event(
            EventKind::RemoteRequestRejected,
            token.map(str::to_owned),
            reason,
        );
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
            EventKind::AuthorityGenerationChanged,
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
            level if level.accepts_goal() => self.has_active_goal(pose),
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
        self.event(EventKind::MissionPaused, None, "router_loss_paused");
    }

    /// Operator-selected level, even while safety holds motion.
    pub fn level(&self) -> Level {
        self.level
    }
    /// The level may hold a GPS destination ([`Level::accepts_outdoor_goal`])
    /// and no stop or pause is in force.
    pub fn outdoor_authority(&self) -> bool {
        self.level.accepts_outdoor_goal() && !self.stopped && !self.paused
    }
    /// See [`Level::reads_outdoor_destination`].
    pub fn reads_outdoor_destination(&self) -> bool {
        self.level.reads_outdoor_destination()
    }
    /// See [`Level::needs_map`].
    pub fn needs_map(&self) -> bool {
        self.level.needs_map()
    }
    /// Local-frame GPS destination for explore and task, or `None` to clear it.
    pub fn set_outdoor_destination(&mut self, destination: Option<(f64, f64)>) {
        self.outdoor_destination = destination.filter(|(x, y)| x.is_finite() && y.is_finite());
    }
    pub fn outdoor_destination(&self) -> Option<(f64, f64)> {
        self.outdoor_destination
    }
    /// GPS arrival ends an explore run toward a destination.
    pub fn finish_explore_at_destination(&mut self) {
        if self.level == Level::Explore && self.explorer.is_running() {
            self.explorer.stop(EndReason::ObjectiveComplete);
        }
    }
    pub fn has_active_goal(&self, pose: Pose) -> bool {
        self.follower.step(pose.x, pose.y, pose.yaw).status.state == GoalState::Active
    }
    /// Drop an L2/L3 waypoint goal that a sensor hold has blocked for too
    /// long, naming why in `request_reason` and a `goal_dropped` event.
    /// Other levels keep their own timeouts. Return whether a goal was dropped.
    pub fn drop_goal_for_sensor_hold(&mut self, pose: Pose) -> bool {
        if !matches!(self.level, Level::Waypoint | Level::WaypointDirect)
            || !self.has_active_goal(pose)
        {
            return false;
        }
        self.follower.cancel();
        self.maneuver.clear();
        self.plan.command = None;
        self.request_reason = Some("sensor_hold_timeout".into());
        self.event(EventKind::GoalDropped, None, "sensor_hold_timeout");
        true
    }
    pub fn clear_guidance(&mut self) {
        self.follower.cancel();
        self.maneuver.clear();
    }
    pub fn update_guidance(&mut self, x: f64, y: f64) {
        if self.follower.step(x, y, 0.).status.state == GoalState::Idle {
            self.follow_local(x, y, None);
        } else {
            self.follower.update_guidance(x, y);
        }
        self.maneuver.clear();
    }
    /// Latch a local goal on the follower inside the full local extent. Returns
    /// whether it latched; a refused point (nonfinite or beyond
    /// [`terra_waypoint::MAX_LOCAL_GOAL_M`]) is logged as `goal_out_of_bounds`
    /// and leaves the follower's goal unchanged.
    pub(super) fn follow_local(&mut self, x: f64, y: f64, token: Option<String>) -> bool {
        let goal = GoalCommand::Local {
            x,
            y,
            yaw: None,
            token,
        };
        let ok = follow(&mut self.follower, &goal);
        if !ok {
            self.event(EventKind::GoalRejected, None, "goal_out_of_bounds");
        }
        ok
    }

    /// See [`Level::is_manual`].
    pub fn is_manual(&self) -> bool {
        self.level.is_manual()
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
    /// Apply a validated set of navigation settings. A changed planner retires
    /// cached plans, routes and operator intent and advances the authority
    /// revision; identical planner settings leave all of that alone. Invalid
    /// turning limits are refused before anything changes.
    pub fn configure(&mut self, settings: &ArbiterSettings) -> Result<(), terra_types::InputError> {
        TurningLimiter::new(settings.turning)?;
        if settings.planner != self.planner_config {
            self.planner_config = settings.planner.clone();
            self.apply_planner();
            self.explorer.set_footprint(settings.planner.footprint);
            self.plan.forget();
            self.reset_route();
            self.route_map_revision = 0;
            self.route_checked_at = 0.;
            self.operator = None;
            self.reset = true;
            self.revision = self.revision.saturating_add(1);
        }
        self.configure_turning(settings.turning)?;
        self.explorer.config_mut().through_unknown = settings.explore_through_unknown;
        Ok(())
    }
    /// Settings currently applied by [`Self::configure`].
    pub fn settings(&self) -> ArbiterSettings {
        ArbiterSettings {
            planner: self.planner_config.clone(),
            turning: self.turning_config,
            explore_through_unknown: self.explorer.config().through_unknown,
        }
    }
    /// Cap planner speeds for this and later ticks, for example to the
    /// controller's limits or a slower outdoor mission. A cap that changes the
    /// effective limits retires the cached plan, which was chosen under the old ones.
    pub fn set_speed_caps(&mut self, max_linear: f64, max_angular: f64) {
        if self.speed_caps != (max_linear, max_angular) {
            self.speed_caps = (max_linear, max_angular);
            self.apply_planner();
        }
    }
    /// The rover body the planner checks.
    pub fn footprint(&self) -> terra_navigation::Footprint {
        self.planner.config.footprint
    }
    /// Effective planner limits: the configured planner with the speed caps applied.
    pub fn planner_config(&self) -> &terra_navigation::PlannerConfig {
        &self.planner.config
    }
    fn apply_planner(&mut self) {
        let mut config = self.planner_config.clone();
        config.max_linear = config.max_linear.min(self.speed_caps.0);
        config.max_angular = config.max_angular.min(self.speed_caps.1);
        if config.max_linear != self.planner.config.max_linear
            || config.max_angular != self.planner.config.max_angular
            || config.footprint != self.planner.config.footprint
        {
            self.plan.forget();
        }
        self.planner.config = config;
    }
    /// Run identifier used to correlate commands and telemetry.
    pub fn run_id(&self) -> &str {
        &self.run_id
    }
    /// Bind the arbiter to a run. Commands and searches that name another run are refused.
    pub fn set_run(&mut self, run_id: &str) {
        if self.run_id != run_id {
            self.run_id = run_id.to_owned();
        }
    }
    /// Study condition: see [`Self::level_permitted`].
    pub fn assigned_level(&self) -> Option<Level> {
        self.assigned_level
    }
    /// Whether an exploration run is in progress.
    pub fn exploring(&self) -> bool {
        self.explorer.is_running()
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
    /// Mirror the runtime's tilt latch. While it is set the arbiter holds an
    /// emergency stop at every level (L0 included) with reason `tilt_stop`.
    /// The first latch ends exploration, a task and any dock, like an
    /// operator stop. Clearing it does not clear the stop: the operator's
    /// safety reset does that, and the runtime refuses the reset while the
    /// rover is still tilted.
    pub fn set_tilt_stop(&mut self, latched: bool) {
        if latched && !self.tilt_stop {
            self.end_mission(Some(EndReason::OperatorStop), "tilt_stop");
            self.event(EventKind::TiltStop, None, "tilt_stop");
        }
        if latched {
            self.stopped = true;
        }
        self.tilt_stop = latched;
    }
    /// Whether the tilt latch is mirrored into the stop.
    pub fn tilt_stopped(&self) -> bool {
        self.tilt_stop
    }
    /// Set the waypoint WGS84 origin in degrees. Returns false, keeping the old
    /// origin, for coordinates outside [`terra_waypoint::GeoOrigin::new`]'s range.
    pub fn set_origin(&mut self, lat: f64, lon: f64) -> bool {
        self.follower.set_origin(lat, lon).is_ok()
    }
    pub(super) fn event(&mut self, kind: EventKind, token: Option<String>, reason: &str) {
        if self.events.len() < 256 {
            self.events.push(DecisionEvent {
                kind: kind.as_str().into(),
                token,
                reason: reason.into(),
                level: self.level,
            });
        }
    }
    /// Token admission for every token-correlated request. `None` admits a new
    /// token. A token seen before is not applied again: its receipt is
    /// restored and `Some(accepted)` says whether the earlier request was this
    /// same request and was accepted.
    pub(super) fn repeat(&mut self, token: &str, fingerprint: &str) -> Option<bool> {
        let (_, old, ok) = self.cache.iter().find(|(t, _, _)| t == token)?;
        let accepted = old == fingerprint && *ok;
        self.receipt = Some((token.into(), accepted));
        Some(accepted)
    }
    /// [`Self::repeat`] for requests that answer with a `Result`: a repeat of an
    /// accepted request is `Ok`, anything else `conflicting_or_rejected_token`.
    pub(super) fn replay(
        &mut self,
        token: &str,
        fingerprint: &str,
    ) -> Option<Result<(), &'static str>> {
        self.repeat(token, fingerprint).map(|accepted| {
            accepted
                .then_some(())
                .ok_or("conflicting_or_rejected_token")
        })
    }
    /// End every mission in progress and clear motion intent: the one stop path
    /// for operator and tilt stops, goal cancel and level changes. `explore`
    /// ends a running exploration with that reason (`None` leaves it running);
    /// `task_reason` cancels a running task.
    pub(super) fn end_mission(&mut self, explore: Option<EndReason>, task_reason: &str) {
        if let Some(reason) = explore
            && self.explorer.is_running()
        {
            self.explorer.stop(reason);
        }
        self.end_task(TaskPhase::Cancelled, task_reason, f64::NAN);
        self.clear();
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
                EventKind::RequestAccepted
            } else {
                EventKind::RequestRejected
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
        self.plan.command = None;
        self.reset = true;
        Ok(())
    }
    /// The host started a new local frame (for example an AR session reset):
    /// every local position the arbiter holds now names a different place.
    /// Ends any goal, route, exploration run (`safety_hold`), search, task and
    /// dock, and forgets the long-range map memory, rejected frontiers, task
    /// landmarks and outdoor destination. The WGS84 origin is dropped too, so
    /// geographic goals wait for the host's next [`Self::set_origin`]. The
    /// level, safety stops and settings stay.
    pub fn reset_local_frame(&mut self) {
        self.end_mission(Some(EndReason::SafetyHold), "local_frame_reset");
        self.follower = WaypointController::new(self.follower.config().clone()).unwrap();
        self.reset_route();
        self.route_last_failure = None;
        self.reset_recovery();
        self.memory = Default::default();
        self.memory_revision = None;
        self.rejected.clear();
        self.task_landmarks.clear();
        self.outdoor_destination = None;
        self.explore_bias = None;
        self.event(EventKind::LocalFrameReset, None, "local_frame_reset");
    }
    pub(super) fn clear(&mut self) {
        self.cancel_dock("authority_cleared");
        self.turning.reset();
        self.commanded = Twist::default();
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
        self.plan.forget();
    }
    /// Store a budget used when an `explore` level request omits one.
    ///
    /// Return `true` for finite `seconds` in `(0, 86400]`; invalid values return
    /// `false` and leave the previous budget unchanged.
    pub fn set_exploration_budget(&mut self, seconds: f64) -> bool {
        if terra_exploration::valid_budget(seconds) {
            self.exploration_budget = Some(seconds);
            true
        } else {
            false
        }
    }
    /// Whether the assigned study condition lets an operator select `level`.
    /// L0 takeover is always allowed; an assigned L4 also allows target search.
    pub fn level_permitted(&self, level: Level) -> bool {
        match self.assigned_level {
            None => true,
            Some(assigned) => {
                level == Level::Teleop
                    || level == assigned
                    || (assigned == Level::Explore && level == Level::TargetSearch)
            }
        }
    }
    /// Set or clear the study condition. A current level outside the new
    /// condition drops to L0, as an operator takeover would.
    pub fn assign_level(&mut self, r: AssignmentRequest) {
        let fp = format!("assign:{:?}", r.assigned_level);
        if !valid_token(&r.token) || self.repeat(&r.token, &fp).is_some() {
            return;
        }
        self.assigned_level = r.assigned_level;
        if !self.level_permitted(self.level) {
            let token = format!("assign-l0-{}", self.revision);
            self.set_level(LevelRequest::new(Level::Teleop, token));
        }
        self.acknowledge(
            r.token,
            fp,
            true,
            if r.assigned_level.is_some() {
                "level_assigned"
            } else {
                "assignment_cleared"
            },
        );
    }
    /// Why `level` cannot be selected at `now`, or `None` when it can: target
    /// search needs a fresh detector capability, task a language model or an
    /// MCP agent. Status advertises exactly the levels this allows.
    pub(super) fn level_unavailable(&self, level: Level, now: f64) -> Option<&'static str> {
        match level {
            Level::TargetSearch
                if self.detector_classes.is_empty()
                    || !fresh(now, self.detector_time, timing::DETECTOR_FRESHNESS_SECONDS) =>
            {
                Some("detector_unavailable")
            }
            Level::Task if self.task_model.is_none() && self.task_agent.is_none() => {
                Some("task_model_unavailable")
            }
            _ => None,
        }
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
        if !valid_token(&r.token) || self.repeat(&r.token, &fp).is_some() {
            return;
        }
        if !self.level_permitted(r.level) {
            self.acknowledge(r.token, fp, false, "level_not_assigned");
            return;
        }
        let now = self
            .last_time
            .into_iter()
            .chain(self.detector_time)
            .fold(f64::NEG_INFINITY, f64::max);
        if let Some(reason) = self.level_unavailable(r.level, now) {
            self.acknowledge(r.token, fp, false, reason);
            return;
        }
        if !budget_fields_valid(&r) {
            self.acknowledge(r.token, fp, false, "invalid_budget");
            return;
        }
        if task
            .as_ref()
            .is_some_and(|t| r.level != Level::Task || !t.valid())
        {
            self.acknowledge(r.token, fp, false, "invalid_task");
            return;
        }
        if let Some(reason) = task.as_ref().and_then(|t| self.task_answerer_missing(t)) {
            self.acknowledge(r.token, fp, false, reason);
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
        let changed = self.level != r.level;
        self.end_mission(
            (r.level != Level::Explore).then_some(EndReason::OperatorTakeover),
            if changed {
                "level_changed"
            } else {
                "operator_reselection"
            },
        );
        self.level = r.level;
        // The default 0.75 m arrival radius is for coarse waypoints. Frontiers are
        // often picked well under a metre away, so with it Explore counted a
        // frontier as reached after a few centimetres and moved on to the next.
        let config = match r.level {
            Level::TargetSearch => WaypointConfig {
                arrive_radius: 0.1,
                slow_radius: 0.5,
                ..WaypointConfig::default()
            },
            Level::Explore => WaypointConfig {
                arrive_radius: 0.2,
                slow_radius: 0.6,
                ..WaypointConfig::default()
            },
            _ => WaypointConfig::default(),
        };
        // The defaults validate; a level change keeps the WGS84 origin.
        self.follower.reconfigure(config).unwrap();
        self.paused = false;
        self.explore_blocked = false;
        self.explore_stale_since = None;
        self.explore_unhealthy_since = None;
        if r.level == Level::Teleop {
            self.event(
                EventKind::Takeover,
                Some(r.token.clone()),
                "operator_takeover",
            );
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
        if self.stopped || !self.level.operator_driven() {
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
            EventKind::OperatorReceived,
            r.operator_session_id.clone(),
            &format!("sequence:{:?}", r.sequence),
        );
        if self.dock_active() && (r.linear != 0. || r.angular != 0.) {
            self.cancel_dock("operator_takeover");
        }
        self.operator = Some((r, now));
        self.operator_origin = origin;
        true
    }
    /// Refuse goal `g` for `reason`: acknowledge its token (if any) as
    /// rejected and log a `goal_rejected` event.
    pub fn reject_goal(&mut self, g: &GoalCommand, reason: &str) {
        let token = match g {
            GoalCommand::Local { token, .. } | GoalCommand::Wgs84 { token, .. } => token.clone(),
            GoalCommand::Cancel => None,
        };
        if let Some(t) = token.clone() {
            self.acknowledge(t, format!("goal:{g:?}"), false, reason);
        }
        self.event(EventKind::GoalRejected, token, reason);
    }
    /// Accept or cancel a waypoint and return whether it was accepted.
    ///
    /// Local coordinates use metres (at most [`terra_waypoint::MAX_LOCAL_GOAL_M`] from the origin); WGS84 coordinates use degrees and yaw uses radians.
    /// Invalid, out-of-bounds, unauthorized, or conflicting-token goals return `false`; cancellation clears active intent.
    pub fn accept_goal(&mut self, g: &GoalCommand) -> bool {
        if self.level == Level::TargetSearch {
            self.event(EventKind::GoalRejected, None, "search_active");
            return false;
        }
        if matches!(g, GoalCommand::Cancel) {
            self.end_mission(Some(EndReason::OperatorStop), "operator_cancel");
            if matches!(self.level, Level::Waypoint | Level::WaypointDirect) {
                self.level = Level::Teleop;
            } else if self.level == Level::Supervised {
                self.paused = true;
                self.pause_reason = "supervision_paused";
            }
            self.revision += 1;
            self.event(EventKind::GoalCancel, None, "operator_cancel");
            return true;
        }
        let token = match g {
            GoalCommand::Local { token, .. } | GoalCommand::Wgs84 { token, .. } => token.clone(),
            _ => None,
        };
        if !g.valid() {
            self.event(EventKind::GoalRejected, token, "invalid_goal");
            return false;
        }
        let fingerprint = format!("goal:{g:?}");
        if let Some(accepted) = token.as_ref().and_then(|t| self.repeat(t, &fingerprint)) {
            return accepted;
        }
        let refusal = if self.stopped {
            // A held safety stop, not a level problem: the operator resets first.
            Some(SAFETY_STOPPED)
        } else if !self.level.accepts_goal() {
            Some("authority_required")
        } else {
            None
        };
        if let Some(reason) = refusal {
            if let Some(t) = token.clone() {
                self.acknowledge(t, fingerprint, false, reason);
            }
            self.event(EventKind::GoalRejected, token, reason);
            return false;
        }
        let ok = follow(&mut self.follower, g);
        if ok {
            self.plan.forget();
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
            if ok {
                EventKind::GoalAccepted
            } else {
                EventKind::GoalRejected
            },
            token,
            "operator_goal",
        );
        ok
    }
    /// Apply a valid token-correlated stop/reset; reset is rejected when `healthy` is false.
    ///
    /// Stops clear motion intent immediately. Receipts and events expose acceptance or rejection.
    pub fn set_safety(&mut self, r: SafetyRequest, healthy: bool) {
        self.set_safety_refusing(r, (!healthy).then_some("unhealthy_reset"));
    }
    /// [`Self::set_safety`] with the reason a reset must be refused, if any,
    /// such as `still_tilted` or `pose_not_ready`. Stops are never refused.
    pub fn set_safety_refusing(&mut self, r: SafetyRequest, refusal: Option<&'static str>) {
        let fp = format!("safety:{:?}", r.action);
        if !valid_token(&r.token) || self.repeat(&r.token, &fp).is_some() {
            return;
        }
        let ok = r.action == SafetyAction::Stop || refusal.is_none();
        if ok {
            if r.action == SafetyAction::Stop {
                self.end_mission(Some(EndReason::OperatorStop), "operator_stop");
            } else {
                self.clear();
            }
            self.stopped = r.action == SafetyAction::Stop;
        }
        self.acknowledge(
            r.token,
            fp,
            ok,
            if ok {
                "operator_safety"
            } else {
                refusal.unwrap_or("unhealthy_reset")
            },
        );
    }
    /// Apply a run-correlated supervised decision; approvals recheck freshness, health, expiry, and reachability.
    ///
    /// Rejected or stale decisions produce a rejected receipt; resume requires no proposal identifier.
    pub fn decide_proposal(&mut self, r: ProposalDecision, input: &ArbiterInput<'_>) {
        let fp = format!("decision:{}:{:?}:{:?}", r.run_id, r.decision, r.proposal_id);
        if !valid_token(&r.token) || self.repeat(&r.token, &fp).is_some() {
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
                } else if r.decision == Decision::Approve && self.inputs_fresh(input) {
                    // Revalidate reachability on the latest occupancy snapshot, independently of the proposal revision.
                    if let Some(m) = input.map
                        && self.route_confirms(m, input.pose, p.x, p.y)
                    {
                        ok = self.follow_local(p.x, p.y, Some(r.token.clone()));
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

/// Latch `goal` on `follower` anywhere within
/// [`terra_waypoint::MAX_LOCAL_GOAL_M`] of the origin; whether it latched.
pub(super) fn follow(follower: &mut WaypointController, goal: &GoalCommand) -> bool {
    // The extent is a valid constant, so `accept` cannot return `Err`.
    follower
        .accept(goal, terra_waypoint::MAX_LOCAL_GOAL_M)
        .unwrap_or(false)
}
