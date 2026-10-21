//! Choose desired motion, then check it against occupancy and turning limits.
use super::*;

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
            let (intent, maneuver_active) = self.heading_maneuver(
                input,
                planning_map,
                command_origin,
                decision,
                intent,
                guidance_heading,
            );
            if let Some(why) = decision.route_failure {
                decision.held = true;
                decision.reason = why;
                decision.source = "none";
                self.last_plan = None;
                self.planner_trace = None;
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
                self.explore_blocked = decision.reason == "obstacle_blocked";
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
        let intent = match self.level {
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
                    } else {
                        "waypoint"
                    };
                }
                if self.level == Level::Waypoint && out.status.state == GoalState::Active {
                    self.waypoint_route_intent(
                        input,
                        planning_map.unwrap(),
                        out,
                        &mut guidance_heading,
                        decision,
                    )
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
        if maneuver_active && !decision.held {
            let (planned, mut trace) = self.planner.plan_with_turning(
                planning_map.unwrap(),
                input.pose,
                input.measured,
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
        } else if !decision.held && matches!(self.level, Level::Teleop | Level::WaypointDirect) {
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
                        input.measured,
                        intent,
                        timing::PLANNER_STEP_SECONDS,
                        &self.turning,
                        input.now,
                    )
                } else {
                    planner.plan_with_trace(
                        planning_map.unwrap(),
                        input.pose,
                        input.measured,
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
        let radius = self.planner.config.radius;
        let destination = (out.status.goal_id, out.status.x, out.status.y);
        let here = [input.pose.x, input.pose.y];
        // Drop reached route points; never use a distant point without checking line of sight.
        while self.global_route.len() > 1
            && (self.global_route[0][0] - here[0]).hypot(self.global_route[0][1] - here[1]) < 0.2
        {
            self.global_route.remove(0);
        }
        let changed_goal = self.route_goal != Some(destination);
        let due = input.now - self.route_checked_at >= timing::ROUTE_RECHECK_SECONDS;
        let segment_blocked = self.route_map_revision != input.map_revision
            && !std::iter::once(here)
                .chain(self.global_route.iter().copied())
                .collect::<Vec<_>>()
                .windows(2)
                .all(|p| {
                    terra_navigation::route_segment_clear(map, input.pose, radius, p[0], p[1])
                });
        // Approach routes stop at the frontier. Replan on the same
        // interval so a rolling map can extend them toward the goal.
        let needs_route = changed_goal
            || (due
                && (self.global_route.is_empty() || !self.route_reaches_goal || segment_blocked));
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
                    self.route_reaches_goal = route.last().is_some_and(|point| {
                        (point[0] - destination.1).abs() <= 1e-9
                            && (point[1] - destination.2).abs() <= 1e-9
                    });
                    route
                }
                Err(why) => {
                    self.route_reaches_goal = false;
                    decision.route_failure = Some(why);
                    Vec::new()
                }
            };
        }
        if due || changed_goal {
            self.route_checked_at = input.now;
            self.route_map_revision = input.map_revision;
        }
        let target = self.global_route.iter().rev().copied().find(|point| {
            terra_navigation::route_segment_clear(map, input.pose, radius, here, *point)
        });
        if let Some(point) = target {
            *guidance_heading = (point[1] - input.pose.y).atan2(point[0] - input.pose.x);
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
            decision.route_failure = Some(decision.route_failure.unwrap_or("route_no_path"));
            Twist::default()
        }
    }
}
