//! Getting unstuck on a routed level (Waypoint, Task, Supervised, Explore,
//! TargetSearch) when the local planner keeps holding with `obstacle_blocked`.
//!
//! 1. Blocked for [`BLOCKED_SECONDS`]: drop the route and replan from here.
//!    Hybrid A* includes reverse moves, so the new route may back out.
//! 2. Still blocked after that, or no route can start from here
//!    (`route_start_blocked`): reverse in a straight line, slowly, up to
//!    [`BACKOFF_DISTANCE`], checked by the local planner like any motion. Then
//!    replan.
//! 3. Still blocked after backing off, or it cannot reverse: hold with
//!    `boxed_in` (a route failure). Start over after [`BOXED_IN_RETRY_SECONDS`],
//!    or at once for a new goal.
//!
//! Real progress resets the sequence.
use super::*;

/// How long the local planner must hold before recovery starts, seconds.
pub(super) const BLOCKED_SECONDS: f64 = 1.0;
/// Back-off speed, metres/second.
pub(super) const BACKOFF_SPEED: f64 = 0.08;
/// Furthest a back-off reverses, metres.
pub(super) const BACKOFF_DISTANCE: f64 = 0.25;
/// Longest a back-off runs, seconds.
const BACKOFF_SECONDS: f64 = 5.0;
/// How long `boxed_in` holds before recovery starts over, seconds.
pub(super) const BOXED_IN_RETRY_SECONDS: f64 = 5.0;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(super) enum Recovery {
    #[default]
    Idle,
    /// Replanned from where the rover stood.
    Replanned,
    /// Reversing from `from`, started at `since`.
    BackingOff { from: Pose, since: f64 },
    /// Backed off and replanned.
    BackedOff,
    /// Nothing worked; holding until `until`.
    BoxedIn { until: f64 },
}

impl AutonomyArbiter {
    /// Called by the waypoint route step. Returns the twist to use instead of
    /// following the route while recovery is driving or holding.
    pub(super) fn recovery_step(
        &mut self,
        input: &ArbiterInput<'_>,
        map: &MapSnapshot,
        decision: &mut TickDecision,
    ) -> Option<Twist> {
        let blocked_long = self
            .blocked_since
            .is_some_and(|since| input.now - since >= BLOCKED_SECONDS);
        match self.recovery {
            Recovery::Idle if blocked_long => {
                self.recovery = Recovery::Replanned;
                self.force_replan();
                None
            }
            Recovery::Replanned if blocked_long => {
                self.start_backoff(input);
                Some(self.backoff(input, map, decision))
            }
            Recovery::BackingOff { from, since } => {
                let reversed = (input.pose.x - from.x).hypot(input.pose.y - from.y);
                if reversed >= BACKOFF_DISTANCE || input.now - since >= BACKOFF_SECONDS {
                    self.recovery = Recovery::BackedOff;
                    self.force_replan();
                    None
                } else {
                    Some(self.backoff(input, map, decision))
                }
            }
            Recovery::BackedOff if blocked_long => {
                self.boxed_in(input, decision);
                Some(Twist::default())
            }
            Recovery::BoxedIn { until } => {
                if input.now >= until {
                    self.reset_recovery();
                    self.force_replan();
                    None
                } else {
                    decision.route_failure = Some("boxed_in");
                    Some(Twist::default())
                }
            }
            _ => None,
        }
    }

    /// A route cannot start where the rover stands: back off if not yet tried.
    /// Returns true when recovery took over.
    pub(super) fn recover_from_start_blocked(&mut self, input: &ArbiterInput<'_>) -> bool {
        match self.recovery {
            Recovery::Idle | Recovery::Replanned => {
                self.start_backoff(input);
                true
            }
            _ => false,
        }
    }

    /// Track how long the local planner has held a routed level; real
    /// progress resets recovery. Called after the motion check each tick.
    pub(super) fn note_blocked(&mut self, input: &ArbiterInput<'_>, decision: &TickDecision) {
        if !self.routed_level() {
            self.blocked_since = None;
            self.reset_recovery();
            return;
        }
        if decision.reason == "obstacle_blocked" {
            self.blocked_since.get_or_insert(input.now);
            self.progress_since = None;
        } else {
            self.blocked_since = None;
            let moving = decision.twist != Twist::default()
                && !matches!(self.recovery, Recovery::BackingOff { .. });
            if moving {
                let since = *self.progress_since.get_or_insert(input.now);
                if input.now - since >= 2. * BLOCKED_SECONDS {
                    self.reset_recovery();
                }
            } else {
                self.progress_since = None;
            }
        }
    }

    pub(super) fn reset_recovery(&mut self) {
        self.recovery = Recovery::Idle;
        self.progress_since = None;
    }

    fn start_backoff(&mut self, input: &ArbiterInput<'_>) {
        self.recovery = Recovery::BackingOff {
            from: input.pose,
            since: input.now,
        };
        self.blocked_since = None;
        self.clear_route();
        self.route_reaches_goal = false;
    }

    /// One back-off tick: reverse straight back while that keeps the body off
    /// anything it does not already touch, else give up as boxed in. The motion
    /// is checked here (re-checked every tick), not by the local planner, which
    /// refuses every motion while the body already overlaps an obstacle.
    fn backoff(
        &mut self,
        input: &ArbiterInput<'_>,
        map: &MapSnapshot,
        decision: &mut TickDecision,
    ) -> Twist {
        let Recovery::BackingOff { from, .. } = self.recovery else {
            return Twist::default();
        };
        let left = BACKOFF_DISTANCE - (input.pose.x - from.x).hypot(input.pose.y - from.y);
        // The rest of the back-off plus room to stop.
        let speed = BACKOFF_SPEED.min(self.planner.config.max_reverse);
        let check = left.max(0.) + speed * speed / (2. * self.planner.config.braking);
        if !terra_navigation::backoff_clear(map, &self.planner.config.footprint, input.pose, check)
        {
            self.boxed_in(input, decision);
            return Twist::default();
        }
        decision.source = "recovery";
        Twist {
            linear: -speed,
            angular: 0.,
        }
    }

    fn boxed_in(&mut self, input: &ArbiterInput<'_>, decision: &mut TickDecision) {
        self.recovery = Recovery::BoxedIn {
            until: input.now + BOXED_IN_RETRY_SECONDS,
        };
        self.blocked_since = None;
        self.clear_route();
        self.route_reaches_goal = false;
        decision.route_failure = Some("boxed_in");
    }

    /// Drop the route and plan again on this tick.
    fn force_replan(&mut self) {
        self.blocked_since = None;
        self.clear_route();
        self.route_reaches_goal = false;
        self.route_checked_at = f64::NEG_INFINITY;
    }
}
