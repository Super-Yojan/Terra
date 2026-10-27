//! Frontier exploration and its hold timers.
use super::*;

impl AutonomyArbiter {
    pub(super) fn exploration_report(&self, now: f64) -> Option<ExplorationStatus> {
        let status = self.explorer.status(now);
        (status.phase != terra_exploration::ExplorePhase::Idle).then_some(status)
    }
    pub(super) fn drain_explore_events(&mut self) {
        for event in self.explorer.take_events() {
            self.event(event.kind, None, &event.reason);
        }
    }
    pub(super) fn halt_explore(&mut self, reason: EndReason) {
        if self.level == Level::Explore {
            self.explorer.stop(reason);
        }
    }
    pub(super) fn note_map_stale(&mut self, now: f64) {
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
    pub(super) fn note_safety_hold(&mut self, now: f64) {
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
    pub(super) fn drive_explore(&mut self, input: &ArbiterInput<'_>) -> Option<&'static str> {
        self.explorer.set_footprint(self.planner.config.radius);
        self.update_explore_bias();
        self.explore_scan = None;
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
        if !matches!(action, ExploreAction::Scan { .. }) {
            self.scan_turning = None;
        }
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
            ExploreAction::Scan { yaw_rate } => {
                self.follower.cancel();
                self.last_plan = None;
                self.explore_scan = Some(yaw_rate);
                return None;
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
    /// Turn in place for the look-around. Unknown cells stay obstacles: the turn
    /// sweeps only the rover's own footprint, which must hold no occupied cell.
    pub(super) fn scan_motion(
        &mut self,
        input: &ArbiterInput<'_>,
        planning_map: Option<&MapSnapshot>,
        yaw_rate: f64,
        decision: &mut TickDecision,
    ) {
        let intent = Twist {
            linear: 0.,
            angular: yaw_rate,
        };
        decision.intent_command = intent;
        decision.source = "explore";
        if !planning_map.is_some_and(|map| self.planner.admissible(map, input.pose, intent)) {
            decision.held = true;
            decision.reason = "obstacle_blocked";
            return;
        }
        let config = TurningConfig {
            allow_in_place: true,
            ..self.turning_config
        };
        let limiter = self
            .scan_turning
            .get_or_insert_with(|| TurningLimiter::new(config).unwrap());
        match limiter.step(terra_types::VelocityTarget {
            timestamp: input.now,
            forward: 0.,
            yaw_rate,
        }) {
            Ok(target) => {
                decision.twist = Twist {
                    linear: 0.,
                    angular: target.yaw_rate,
                };
                decision.reason = "exploration_scan";
            }
            Err(_) => {
                self.scan_turning = None;
                decision.held = true;
                decision.reason = "invalid_turn_timing";
            }
        }
    }
}
impl AutonomyArbiter {
    /// Pull frontier choice toward the GPS destination while one is set. The bias
    /// moves only when the destination shifts by more than a metre, so GPS noise
    /// does not reshuffle frontiers every tick. A new destination, or one that
    /// moved several metres, drops the frontier chosen for the old one.
    fn update_explore_bias(&mut self) {
        let wanted = self.outdoor_destination;
        let shift = match (wanted, self.explore_bias) {
            (Some(a), Some(b)) => (a.0 - b.0).hypot(a.1 - b.1),
            (None, None) => 0.,
            _ => f64::INFINITY,
        };
        if shift <= 1. {
            return;
        }
        if shift > 3. && wanted.is_some() {
            self.explorer.reselect();
        }
        self.explore_bias = wanted;
        match wanted {
            Some((x, y)) => self
                .explorer
                .set_scorer(Box::new(TowardDestination { x, y })),
            None => self
                .explorer
                .set_scorer(Box::new(terra_exploration::InformationGainScorer)),
        }
    }
}

/// Frontier score for explore with a GPS destination: some new information,
/// but mostly a short drive that ends close to the destination.
pub(crate) struct TowardDestination {
    pub x: f64,
    pub y: f64,
}
impl terra_exploration::GoalScorer for TowardDestination {
    fn score(&self, c: &terra_exploration::FrontierCandidate) -> f64 {
        let to_goal = (c.x - self.x).hypot(c.y - self.y);
        f64::from(c.information_gain).sqrt() / (1. + c.distance_m + 3. * to_goal)
    }
}

fn sustained(since: &mut Option<f64>, now: f64, timeout: f64) -> bool {
    if !now.is_finite() {
        return false;
    }
    let start = *since.get_or_insert(now);
    now - start >= timeout.max(0.)
}
