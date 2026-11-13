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
                timing::EXPLORATION_MAP_STALE_GRACE_SECONDS,
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
                timing::EXPLORATION_SAFETY_HOLD_GRACE_SECONDS,
            )
        {
            self.explorer.stop(EndReason::SafetyHold);
        }
    }
    /// `Some` when exploration itself is holding or finished. `None` when the follower should move.
    pub(super) fn drive_explore(&mut self, input: &ArbiterInput<'_>) -> Option<&'static str> {
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
                    self.follow_local(x, y, Some(token));
                    self.plan.command = None;
                }
                None
            }
            ExploreAction::Scan { yaw_rate } => {
                self.follower.cancel();
                self.plan.command = None;
                self.explore_scan = Some(yaw_rate);
                return None;
            }
            ExploreAction::Hold => {
                self.follower.cancel();
                self.plan.command = None;
                Some("frontier_retry")
            }
            ExploreAction::Finished(reason) => {
                self.follower.cancel();
                self.plan.command = None;
                Some(reason.as_str())
            }
            ExploreAction::Idle => Some("exploration_idle"),
        }
    }
    /// The look-around. A rover that may spin turns in place: unknown cells stay
    /// obstacles, and the turn sweeps only the rover's own footprint. A rover
    /// that must not spin drives slowly round its tightest circle instead
    /// (forward, else in reverse), checked like any other motion; the look-around
    /// counts heading turned, so either way it ends after one full turn or on
    /// its timeout. It never falls back to spinning.
    pub(super) fn scan_motion(
        &mut self,
        input: &ArbiterInput<'_>,
        planning_map: Option<&MapSnapshot>,
        yaw_rate: f64,
        decision: &mut TickDecision,
    ) {
        decision.source = Source::Explore;
        let spin = self.turning_config.allow_in_place;
        let circle = yaw_rate.abs() * self.turning_config.min_turn_radius;
        let candidates: &[Twist] = if spin {
            &[Twist {
                linear: 0.,
                angular: yaw_rate,
            }]
        } else {
            &[
                Twist {
                    linear: circle,
                    angular: yaw_rate,
                },
                Twist {
                    linear: -circle,
                    angular: -yaw_rate,
                },
            ]
        };
        decision.intent_command = candidates[0];
        let Some(intent) = candidates.iter().copied().find(|&intent| {
            planning_map.is_some_and(|map| self.planner.admissible(map, input.pose, intent))
        }) else {
            decision.held = true;
            decision.reason = Reason::ObstacleBlocked;
            return;
        };
        decision.intent_command = intent;
        let config = TurningConfig {
            allow_in_place: spin,
            ..self.turning_config
        };
        let limiter = self
            .scan_turning
            .get_or_insert_with(|| TurningLimiter::new(config).unwrap());
        match limiter.step(terra_types::VelocityTarget {
            timestamp: input.now,
            forward: intent.linear,
            yaw_rate: intent.angular,
        }) {
            Ok(target) => {
                decision.twist = Twist {
                    linear: target.forward,
                    angular: target.yaw_rate,
                };
                decision.reason = Reason::ExplorationScan;
            }
            Err(_) => {
                self.scan_turning = None;
                decision.held = true;
                decision.reason = Reason::InvalidTurnTiming;
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
