//! Choose desired motion, then check it against occupancy and turning limits.
use super::*;

/// Hybrid A* expansions per control tick (about 1.5 ms in a release build).
const ROUTE_EXPANSIONS_PER_TICK: u32 = 150;
/// Expansions before a route search gives up (about 2 s of ticks).
const ROUTE_MAX_EXPANSIONS: u32 = 30_000;
/// Heuristic weight for waypoint routes: far fewer expansions, at most this
/// factor longer than the best route.
const ROUTE_HEURISTIC_WEIGHT: f64 = 2.0;
/// Pure-pursuit lookahead along the route, metres.
const ROUTE_LOOKAHEAD: f64 = 0.5;
/// Distance at which a forward/reverse switch counts as reached, metres.
const ROUTE_SWITCH_REACHED: f64 = 0.1;
/// Further than this from the route, the rover replans from where it is, metres.
const ROUTE_DEVIATION: f64 = 0.4;

impl AutonomyArbiter {
    pub(super) fn search_bounded_map(&self, input: &ArbiterInput<'_>) -> Option<MapSnapshot> {
        if self.level == Level::TargetSearch {
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
        }
    }
    /// Explore through unknown space plans on a copy whose unknown cells are free.
    /// The explorer itself still reads the real map for frontiers and the look-around.
    pub(super) fn explore_planning_map(&self, input: &ArbiterInput<'_>) -> Option<MapSnapshot> {
        (self.level == Level::Explore && self.explorer.config().through_unknown)
            .then(|| input.map.map(terra_navigation::unknown_as_free))
            .flatten()
    }
    pub(super) fn select_motion(
        &mut self,
        input: &ArbiterInput<'_>,
        planning_map: Option<&MapSnapshot>,
        command_origin: CommandOrigin,
        decision: &mut TickDecision,
    ) {
        let managed = command_origin != CommandOrigin::LocalManual;
        self.explore_stale_since = None;
        self.explore_unhealthy_since = None;
        let explore_hold = if self.level == Level::Explore {
            let why = self.drive_explore(input);
            decision.goal = self
                .follower
                .step(input.pose.x, input.pose.y, input.pose.yaw)
                .status;
            why
        } else {
            None
        };
        if let Some(why) = explore_hold {
            decision.reason = why;
            decision.held = true;
            self.explore_blocked = false;
        } else if let Some(yaw_rate) = self.explore_scan.filter(|_| self.level == Level::Explore) {
            self.explore_blocked = false;
            self.scan_motion(input, planning_map, yaw_rate, decision);
        } else {
            self.update_search_goal(input, decision);
            let (mut intent, guidance_heading) = self.choose_intent(input, planning_map, decision);
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
            decision.intent_command = intent;
            // Operator or agent reverse skips the map: it runs as asked.
            let reversing = reverse_drive(decision.source, intent);
            let (intent, maneuver_active) = if reversing {
                // Without the planner, keep its reverse speed cap for every origin.
                let cap = &self.planner.config;
                intent.linear = intent.linear.max(-cap.max_reverse.min(cap.max_linear));
                intent.angular = intent.angular.clamp(-cap.max_angular, cap.max_angular);
                self.maneuver.clear();
                (intent, false)
            } else {
                self.heading_maneuver(
                    input,
                    planning_map,
                    command_origin,
                    decision,
                    intent,
                    guidance_heading,
                )
            };
            if let Some(why) = decision.route_failure {
                decision.held = true;
                decision.reason = why;
                decision.source = "none";
                self.last_plan = None;
                self.planner_trace = None;
            } else if decision.route_pending && intent == Twist::default() {
                decision.held = true;
                decision.reason = "route_planning";
                decision.source = "none";
            }
            self.check_motion(
                input,
                planning_map,
                managed,
                intent,
                maneuver_active,
                decision,
            );

            self.update_frontier_proposal(input, decision);
            if self.level == Level::Explore {
                // A missing route holds the rover just like an obstacle does, so
                // it counts toward the short blocked window, not the no-progress one.
                self.explore_blocked =
                    decision.reason == "obstacle_blocked" || decision.route_failure.is_some();
            }
        }
    }
    pub(super) fn limit_turning(
        &mut self,
        input: &ArbiterInput<'_>,
        command_origin: CommandOrigin,
        decision: &mut TickDecision,
    ) {
        let managed = command_origin != CommandOrigin::LocalManual;
        if decision.reason == "exploration_scan" && !decision.held {
            // The look-around has its own limiter in `scan_motion`.
            self.turning.reset();
            return;
        }
        if managed && !decision.held {
            if decision.source == "none" || decision.twist == Twist::default() {
                self.turning.reset();
                if command_origin == CommandOrigin::Autonomous
                    && decision.intent_command.angular != 0.
                    && !self.turning_config.allow_in_place
                {
                    decision.held = true;
                    decision.reason = "heading_requires_spin_or_replan";
                }
            } else {
                match self.turning.step(terra_types::VelocityTarget {
                    timestamp: input.now,
                    forward: decision.twist.linear,
                    yaw_rate: decision.twist.angular,
                }) {
                    Ok(t) => {
                        decision.twist = Twist {
                            linear: t.forward,
                            angular: t.yaw_rate,
                        };
                        if command_origin == CommandOrigin::Autonomous
                            && decision.twist == Twist::default()
                            && decision.intent_command.angular != 0.
                        {
                            decision.held = true;
                            decision.reason = "heading_requires_spin_or_replan";
                        }
                    }
                    Err(_) => {
                        decision.held = true;
                        decision.reason = "invalid_turn_timing";
                    }
                }
            }
        }
    }

    fn update_search_goal(&mut self, input: &ArbiterInput<'_>, decision: &mut TickDecision) {
        if self.level == Level::TargetSearch {
            let detector = self
                .search
                .as_ref()
                .is_some_and(|s| self.detector_available(&s.request.target_class, input.now));
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
                if (decision.goal.x - x).abs() > 1e-8
                    || (decision.goal.y - y).abs() > 1e-8
                    || decision.goal.state != GoalState::Active
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
                decision.goal = self
                    .follower
                    .step(input.pose.x, input.pose.y, input.pose.yaw)
                    .status;
            } else {
                self.follower.cancel();
                self.last_plan = None;
                self.planner_trace = None;
                decision.held = true;
                decision.reason = "search_hold";
            }
        }
    }

    fn choose_intent(
        &mut self,
        input: &ArbiterInput<'_>,
        planning_map: Option<&MapSnapshot>,
        decision: &mut TickDecision,
    ) -> (Twist, f64) {
        let mut guidance_heading =
            (decision.goal.y - input.pose.y).atan2(decision.goal.x - input.pose.x);
        let agent_twist = self.task_agent_twist(input.now);
        let intent = match self.level {
            // An approved MCP agent drives like assisted teleop: the planner checks every
            // forward command; reverse runs unchecked.
            Level::Task if agent_twist.is_some() => {
                decision.source = "task_agent_drive";
                self.clear_route();
                self.route_goal = None;
                self.route_reaches_goal = false;
                agent_twist.unwrap_or_default()
            }
            Level::Teleop | Level::AssistedTeleop => self
                .operator
                .as_ref()
                .filter(|(_, t)| {
                    fresh(
                        input.now,
                        Some(*t),
                        timing::OPERATOR_COMMAND_FRESHNESS_SECONDS,
                    )
                })
                .map(|(r, _)| {
                    decision.source = if self.level == Level::Teleop {
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
                    decision.source = if self.level == Level::Supervised {
                        "frontier"
                    } else if self.level == Level::Explore {
                        "explore"
                    } else if self.level == Level::TargetSearch {
                        "target_search"
                    } else if self.level == Level::Task {
                        "task"
                    } else {
                        "waypoint"
                    };
                }
                // L5 task waypoints use the same A* route as L3.
                if matches!(self.level, Level::Waypoint | Level::Task)
                    && out.status.state == GoalState::Active
                {
                    self.waypoint_route_intent(
                        input,
                        planning_map.unwrap(),
                        out,
                        &mut guidance_heading,
                        decision,
                    )
                } else {
                    if matches!(self.level, Level::Waypoint | Level::Task) {
                        self.clear_route();
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

        (intent, guidance_heading)
    }

    fn heading_maneuver(
        &mut self,
        input: &ArbiterInput<'_>,
        planning_map: Option<&MapSnapshot>,
        command_origin: CommandOrigin,
        decision: &mut TickDecision,
        mut intent: Twist,
        guidance_heading: f64,
    ) -> (Twist, bool) {
        let managed = command_origin != CommandOrigin::LocalManual;
        let mut maneuver_active = false;
        if managed
            && command_origin == CommandOrigin::Autonomous
            && !self.turning_config.allow_in_place
        {
            if self.maneuver_goal != Some(decision.goal.goal_id)
                || decision.goal.state != GoalState::Active
            {
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
            let needs_turn = decision.goal.state == GoalState::Active
                && intent.linear.abs() < 1e-6
                && intent.angular.abs() > 1e-6;
            if self.maneuver.is_empty()
                && needs_turn
                && let Some(map) = planning_map.filter(|_| self.map_is_fresh(input))
            {
                let target = if decision.goal.distance <= 0.75 {
                    decision.goal.yaw.unwrap_or(input.pose.yaw)
                } else {
                    guidance_heading
                };
                if let Some(segments) =
                    self.planner
                        .heading_maneuver(map, input.pose, target, &self.turning_config)
                {
                    self.maneuver = segments.into();
                    self.maneuver_goal = Some(decision.goal.goal_id);
                }
            }
            if let Some(segment) = self.maneuver.front() {
                if self.map_is_fresh(input) && planning_map.is_some() {
                    intent = segment.velocity;
                    maneuver_active = true;
                } else {
                    decision.held = true;
                    decision.reason = "map_stale";
                    self.maneuver.clear();
                }
            } else if needs_turn {
                decision.held = true;
                decision.reason = "heading_requires_spin_or_replan";
            }
        }

        (intent, maneuver_active)
    }

    fn check_motion(
        &mut self,
        input: &ArbiterInput<'_>,
        planning_map: Option<&MapSnapshot>,
        managed: bool,
        intent: Twist,
        maneuver_active: bool,
        decision: &mut TickDecision,
    ) {
        let reversing = reverse_drive(decision.source, intent);
        if maneuver_active && !decision.held {
            let (planned, mut trace) = self.planner.plan_with_turning(
                planning_map.unwrap(),
                input.pose,
                window_centre(self.commanded, input.measured),
                intent,
                timing::PLANNER_STEP_SECONDS,
                &self.turning,
                input.now,
            );
            trace.maneuver_segments = self.maneuver.iter().cloned().collect();
            decision.twist = planned.twist;
            decision.reason = planned.reason;
            self.planner_trace = Some((input.now, trace));
            if decision.twist == Twist::default() || decision.reason == "obstacle_blocked" {
                decision.held = true;
                decision.reason = "heading_requires_spin_or_replan";
                self.maneuver.clear();
            }
        } else if !decision.held
            && (reversing || matches!(self.level, Level::Teleop | Level::WaypointDirect))
        {
            if reversing {
                self.last_plan = None;
                self.planner_trace = None;
            }
            decision.twist = intent;
            if decision.source != "none" {
                decision.reason = "active";
            }
        } else if !decision.held && decision.source != "none" {
            if intent == Twist::default() {
                decision.twist = intent;
                decision.reason = "active";
                self.last_plan = None;
                self.planner_trace = None;
            } else if let Some((at, output, why)) = self.last_plan.filter(|p| {
                !managed
                    && input.now - p.0 < timing::LOCAL_PLAN_CACHE_SECONDS
                    && planning_map.is_some_and(|m| self.planner.admissible(m, input.pose, p.1))
            }) {
                let _ = at;
                decision.twist = output;
                decision.reason = why;
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
                        window_centre(self.commanded, input.measured),
                        intent,
                        timing::PLANNER_STEP_SECONDS,
                        &self.turning,
                        input.now,
                    )
                } else {
                    planner.plan_with_trace(
                        planning_map.unwrap(),
                        input.pose,
                        window_centre(self.commanded, input.measured),
                        intent,
                        timing::PLANNER_STEP_SECONDS,
                    )
                };
                self.planner_trace = (!trace.candidates.is_empty()).then_some((input.now, trace));
                decision.twist = planned.twist;
                decision.reason = planned.reason;
                self.last_plan = Some((input.now, decision.twist, decision.reason));
            }
            if decision.reason == "obstacle_blocked" {
                decision.held = true;
                decision.source = "none";
            }
        }
    }

    fn update_frontier_proposal(&mut self, input: &ArbiterInput<'_>, decision: &mut TickDecision) {
        if self.level == Level::Supervised && decision.goal.state != GoalState::Active {
            self.rejected.retain(|(_, _, expiry)| *expiry > input.now);
            if self
                .proposal
                .as_ref()
                .is_some_and(|p| p.expires_at <= input.now)
            {
                self.proposal = None;
            }
            if self.proposal.is_none()
                && input.now - self.last_frontier >= timing::FRONTIER_PROPOSAL_INTERVAL_SECONDS
            {
                self.last_frontier = input.now;
                let excluded: Vec<_> = self.rejected.iter().map(|&(x, y, _)| (x, y)).collect();
                if let Some((x, y)) = frontier(
                    input.map.unwrap(),
                    input.pose,
                    self.planner.config.footprint.circumradius(),
                    &excluded,
                ) {
                    self.next_proposal += 1;
                    self.proposal = Some(GoalProposal {
                        proposal_id: self.next_proposal,
                        run_id: self.run_id.clone(),
                        x,
                        y,
                        map_revision: input.map_revision,
                        expires_at: input.now + timing::PROPOSAL_LIFETIME_SECONDS,
                        reason: "search_unobserved_sector".into(),
                    });
                    self.event("proposal_created", None, "frontier");
                }
            }
            decision.reason = if self.proposal.is_some() {
                "awaiting_approval"
            } else {
                "no_reachable_frontier"
            };
        }
    }

    fn waypoint_route_intent(
        &mut self,
        input: &ArbiterInput<'_>,
        map: &MapSnapshot,
        out: terra_waypoint::WaypointOutput,
        guidance_heading: &mut f64,
        decision: &mut TickDecision,
    ) -> Twist {
        let destination = (out.status.goal_id, out.status.x, out.status.y);
        self.advance_route_progress(input.pose);
        let changed_goal = self.route_goal != Some(destination);
        if changed_goal {
            self.clear_route();
            self.route_reaches_goal = false;
        }
        let due = input.now - self.route_checked_at >= timing::ROUTE_RECHECK_SECONDS;
        // Off the route (a localization jump, or drift) or a newly blocked route.
        let off_route = self
            .global_route
            .get(self.route_progress)
            .is_some_and(|w| (w.x - input.pose.x).hypot(w.y - input.pose.y) > ROUTE_DEVIATION);
        let blocked = off_route
            || self.route_map_revision != input.map_revision
                && !self.route_still_clear(map, input.pose);
        if blocked {
            // Never keep driving a route that no longer fits or that the rover
            // has left: hold until the replacement is ready.
            self.clear_route();
            self.route_reaches_goal = false;
        }
        // Approach routes stop at the edge of known space. Replan on the same
        // interval so a rolling map can extend them toward the goal.
        let needs_route = changed_goal
            || blocked
            || (due && (self.global_route.is_empty() || !self.route_reaches_goal));
        if needs_route && self.route_job.is_none() {
            self.route_goal = Some(destination);
            match terra_navigation::RouteJob::new(
                map,
                input.pose,
                out.status.x,
                out.status.y,
                &self.hybrid_config(),
                // A goal against an obstacle moves to where the rover fits,
                // still inside the arrival radius so reaching it arrives.
                (self.follower.config().arrive_radius - 0.1).max(0.),
            ) {
                Ok(job) => self.route_job = Some(job),
                Err(why) => {
                    self.clear_route();
                    self.route_reaches_goal = false;
                    decision.route_failure = Some(why);
                }
            }
        }
        if due || changed_goal {
            self.route_checked_at = input.now;
            self.route_map_revision = input.map_revision;
        }
        if let Some(job) = &mut self.route_job
            && let Some(result) = job.advance(ROUTE_EXPANSIONS_PER_TICK)
        {
            self.route_job = None;
            self.last_plan = None;
            self.planner_trace = None;
            match result {
                Ok(route) => {
                    self.global_route = route.waypoints;
                    self.route_progress = 0;
                    self.route_reaches_goal = route.reaches_goal;
                }
                Err(why) => {
                    self.clear_route();
                    self.route_reaches_goal = false;
                    decision.route_failure = Some(why);
                }
            }
        }
        if self.global_route.len() < 2 {
            decision.route_pending = self.route_job.is_some();
            return Twist::default();
        }
        self.follow_route(input.pose, out, guidance_heading)
    }

    /// Forget the route and any search in progress.
    pub(super) fn clear_route(&mut self) {
        self.global_route.clear();
        self.route_progress = 0;
        self.route_job = None;
    }

    /// Hybrid A* settings from the planner's body and the turning policy.
    fn hybrid_config(&self) -> terra_navigation::HybridConfig {
        terra_navigation::HybridConfig {
            max_expansions: ROUTE_MAX_EXPANSIONS,
            heuristic_weight: ROUTE_HEURISTIC_WEIGHT,
            ..terra_navigation::HybridConfig::for_body(
                self.planner.config.footprint,
                &self.turning_config,
            )
        }
    }

    /// Move the progress marker to the nearest route sample a little way ahead.
    fn advance_route_progress(&mut self, pose: Pose) {
        let route = &self.global_route;
        let window = (self.route_progress + 40).min(route.len());
        if let Some((best, _)) = (self.route_progress..window)
            .map(|i| (i, (route[i].x - pose.x).hypot(route[i].y - pose.y)))
            .min_by(|a, b| a.1.total_cmp(&b.1))
        {
            self.route_progress = best;
        }
    }

    /// Whether the rest of the route still fits the body on this map.
    fn route_still_clear(&self, map: &MapSnapshot, pose: Pose) -> bool {
        self.global_route.iter().skip(self.route_progress).all(|w| {
            let at = Pose {
                x: w.x,
                y: w.y,
                yaw: w.yaw,
            };
            terra_navigation::footprint_fits(map, &self.planner.config.footprint, at, Some(pose))
        })
    }

    /// Pure pursuit along the route with curvature capped by the turn radius.
    /// Reverse sections are driven backwards; the target never looks past a
    /// change of direction, so the rover stops at each switch before reversing.
    fn follow_route(
        &mut self,
        pose: Pose,
        out: terra_waypoint::WaypointOutput,
        guidance_heading: &mut f64,
    ) -> Twist {
        let route = &self.global_route;
        let last = route.len() - 1;
        let dist = |i: usize| (route[i].x - pose.x).hypot(route[i].y - pose.y);
        // The section being driven runs from `at` to `end`, all in one direction.
        // Within reach of a direction switch, move on to the next section.
        let mut at = self.route_progress.min(last - 1);
        let (reverse, end) = loop {
            let reverse = route[at + 1].reverse;
            let mut end = at + 1;
            while end < last && route[end + 1].reverse == reverse {
                end += 1;
            }
            if end < last && dist(end) < ROUTE_SWITCH_REACHED {
                at = end;
                continue;
            }
            break (reverse, end);
        };
        self.route_progress = at;
        let route = &self.global_route;
        let length = |from: usize, to: usize| -> f64 {
            route[from..=to]
                .windows(2)
                .map(|w| (w[1].x - w[0].x).hypot(w[1].y - w[0].y))
                .sum()
        };
        // Lookahead target along this section.
        let mut target = at + 1;
        while target < end && length(at, target) < ROUTE_LOOKAHEAD {
            target += 1;
        }
        let to_section_end = length(at, end) + (route[at].x - pose.x).hypot(route[at].y - pose.y);
        // On the final forward stretch to the real goal, the waypoint follower
        // handles arrival and the final heading.
        if self.route_reaches_goal && end == last && !reverse && to_section_end <= ROUTE_LOOKAHEAD {
            *guidance_heading = (out.status.y - pose.y).atan2(out.status.x - pose.x);
            return Twist {
                linear: out.linear,
                angular: out.angular,
            };
        }
        let goal = route[target];
        *guidance_heading = goal.yaw;
        let heading = pose.yaw + if reverse { std::f64::consts::PI } else { 0. };
        let (dx, dy) = (goal.x - pose.x, goal.y - pose.y);
        let ahead = dx * heading.cos() + dy * heading.sin();
        let left = -dx * heading.sin() + dy * heading.cos();
        let distance_sq = ahead * ahead + left * left;
        let kappa_max = 1. / self.turning_config.min_turn_radius;
        let curvature = if distance_sq < 1e-6 {
            0.
        } else {
            (2. * left / distance_sq).clamp(-kappa_max, kappa_max)
        };
        let config = self.follower.config();
        let mut speed = config.cruise;
        if reverse {
            speed = speed.min(self.planner.config.max_reverse);
        }
        // The turning limiter caps yaw rate: slow down so this arc stays drivable.
        if curvature.abs() > 1e-9 {
            speed = speed.min(self.turning_config.max_yaw_rate / curvature.abs());
        }
        // Slow toward a direction switch and toward the end of the route; stop
        // only at the end (a switch is passed within ROUTE_SWITCH_REACHED).
        if to_section_end < config.slow_radius {
            speed *= (to_section_end / config.slow_radius).max(0.2);
        }
        if end == last && to_section_end < 0.05 {
            speed = 0.;
        }
        Twist {
            linear: if reverse { -speed } else { speed },
            angular: speed * curvature,
        }
    }
}

/// Velocity the local planner's window grows from, per axis: the measured
/// value when the rover is already going faster the same way, otherwise the
/// last command. A stalled wheel reads zero, and centring on it alone kept
/// L3 at 0.1 m/s, inside the motor deadband, after every hold.
fn window_centre(commanded: Twist, measured: Twist) -> Twist {
    let pick = |c: f64, m: f64| {
        if c * m >= 0. && m.abs() > c.abs() {
            m
        } else {
            c
        }
    };
    Twist {
        linear: pick(commanded.linear, measured.linear),
        angular: pick(commanded.angular, measured.angular),
    }
}

/// Operator or agent reverse runs without the map or the local planner.
fn reverse_drive(source: &str, intent: Twist) -> bool {
    intent.linear < 0. && matches!(source, "assisted_operator" | "task_agent_drive")
}
