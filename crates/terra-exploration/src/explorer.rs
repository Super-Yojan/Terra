use crate::{
    Detection, EndReason, FrontierCandidate, GoalScorer, InformationGainScorer, MissionObjective,
    ObjectiveContext, TimeBudgetObjective, map_coverage, select_frontier,
};
use serde::{Deserialize, Serialize};
use terra_mapping::MapSnapshot;
use terra_navigation::{Pose, reachable};
/// Lifecycle phase of a frontier exploration run.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ExplorePhase {
    /// No run is active and no goal should be issued.
    Idle,
    /// An exploration run is active.
    Running,
    /// The exploration run has a frozen terminal result.
    Ended,
}
/// Selected world-metre frontier centre and its scorer output.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct FrontierTarget {
    /// World-frame x coordinate, in metres.
    pub x: f64,
    /// World-frame y coordinate, in metres.
    pub y: f64,
    /// Scorer output; larger finite values are preferred.
    pub score: f64,
}
/// Published on `terra/rover/<id>/exploration/status` and copied onto autonomy status.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ExplorationStatus {
    /// Run identifier used to correlate commands and telemetry.
    pub run_id: String,
    /// Current lifecycle phase.
    pub phase: ExplorePhase,
    /// Elapsed run time in seconds; frozen once the run ends.
    pub elapsed_seconds: f64,
    /// Remaining budget in seconds, clamped to zero.
    pub remaining_seconds: f64,
    /// Run time budget in seconds; positive values enable budget expiry.
    pub budget_seconds: f64,
    /// Selected frontier destination, if any.
    pub target: Option<FrontierTarget>,
    /// Observed free-cell area in square metres.
    pub coverage_m2: f64,
    /// Fraction of occupancy cells observed, including occupied cells, from zero to one.
    pub explored_ratio: f64,
    /// Terminal reason, absent before the run ends.
    pub end_reason: Option<EndReason>,
    /// Number of remembered blocked-frontier entries, including expired entries.
    pub blacklist_count: u32,
    /// Number of frontier assignments made during this run.
    pub frontiers_attempted: u64,
    /// True while the rover turns in place to map its blind zone.
    #[serde(default)]
    pub scanning: bool,
}
impl Default for ExplorationStatus {
    fn default() -> Self {
        Self {
            run_id: String::new(),
            phase: ExplorePhase::Idle,
            elapsed_seconds: 0.,
            remaining_seconds: 0.,
            budget_seconds: 0.,
            target: None,
            coverage_m2: 0.,
            explored_ratio: 0.,
            end_reason: None,
            blacklist_count: 0,
            frontiers_attempted: 0,
            scanning: false,
        }
    }
}
/// Knobs for one explorer. The time budget is per run, not part of this struct.
#[derive(Clone, Debug)]
pub struct ExplorationConfig {
    /// Circumscribed radius passed to the shared clearance check. The arbiter overwrites this from the planner.
    pub footprint_radius: f64,
    /// Seconds a goal may stay blocked or fail to get closer before it is blacklisted.
    pub stuck_timeout: f64,
    /// Seconds before a blacklisted frontier may be selected again.
    pub blacklist_seconds: f64,
    /// Failures before a frontier stays excluded for the rest of the run.
    pub max_attempts: u32,
    /// Metres. A candidate this close to a blacklisted point is skipped.
    pub exclude_radius: f64,
    /// Metres of progress that resets the stuck timer.
    pub progress_epsilon: f64,
    /// Cap on the local planner horizon while exploring.
    ///
    /// Frontier goals sit on the boundary of unknown space, and the shared planner treats
    /// unknown cells as occupied. A multi-second rollout therefore stops short of every
    /// frontier. The obstacle check still runs; only the lookahead is shortened.
    pub approach_horizon: f64,
    /// Seconds between `exploration_progress` log events.
    pub progress_period: f64,
    /// Continuous seconds of map staleness before the run ends. `0` ends on the first stale tick.
    pub map_stale_timeout: f64,
    /// Continuous seconds of an unhealthy sensor hold before the run ends. Emergency stop is immediate.
    pub safety_hold_timeout: f64,
    /// Look-around turn rate in rad/s. Zero or less disables the look-around.
    ///
    /// A front depth camera cannot see beside or behind the chassis. When no
    /// frontier is reachable and unknown cells lie within [`Self::scan_radius`],
    /// the rover turns in place once so the camera maps around it before the run
    /// gives up. Unknown space stays an obstacle throughout.
    pub scan_yaw_rate: f64,
    /// Metres beyond the footprint radius in which unknown cells trigger a look-around.
    pub scan_radius: f64,
    /// Seconds a look-around may take before it counts as finished.
    pub scan_timeout: f64,
}
impl Default for ExplorationConfig {
    fn default() -> Self {
        Self {
            footprint_radius: 0.65,
            stuck_timeout: terra_types::timing::EXPLORATION_STUCK_SECONDS,
            blacklist_seconds: terra_types::timing::EXPLORATION_BLACKLIST_SECONDS,
            max_attempts: 3,
            exclude_radius: 1.,
            progress_epsilon: 0.2,
            approach_horizon: terra_types::timing::EXPLORATION_APPROACH_HORIZON_SECONDS,
            progress_period: terra_types::timing::EXPLORATION_PROGRESS_PERIOD_SECONDS,
            map_stale_timeout: terra_types::timing::EXPLORATION_MAP_STALE_GRACE_SECONDS,
            safety_hold_timeout: terra_types::timing::EXPLORATION_SAFETY_HOLD_GRACE_SECONDS,
            scan_yaw_rate: 0.35,
            scan_radius: 1.,
            scan_timeout: terra_types::timing::EXPLORATION_SCAN_TIMEOUT_SECONDS,
        }
    }
}
/// One in-place look-around: accumulated turn since it began.
#[derive(Clone, Copy, Debug)]
struct Scan {
    started_at: f64,
    last_yaw: f64,
    turned: f64,
}
#[derive(Clone, Debug)]
struct BlackEntry {
    x: f64,
    y: f64,
    attempts: u32,
    retry_at: f64,
}
/// Exploration event drained by integrations for recording or publication.
#[derive(Clone, Debug)]
pub struct ExploreEvent {
    /// Machine-readable event category.
    pub kind: &'static str,
    /// Machine-readable explanation of the current decision or transition.
    pub reason: String,
}
/// What the arbiter should do with the waypoint follower after one exploration tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ExploreAction {
    /// No run is active and no goal should be issued.
    Idle,
    /// Issue or continue the world-metre frontier goal.
    Seek {
        /// Frontier x coordinate, in world metres.
        x: f64,
        /// Frontier y coordinate, in world metres.
        y: f64,
    },
    /// No usable frontier yet. The blacklist will expire; do not end the run.
    Hold,
    /// Turn in place at `yaw_rate` rad/s to map the blind zone, then look again.
    Scan {
        /// Signed yaw rate in rad/s; positive turns left.
        yaw_rate: f64,
    },
    /// Stop following exploration goals and report the terminal reason.
    Finished(EndReason),
}
/// Pose, map, and the goal the follower is already tracking.
pub struct ExploreTick<'a> {
    /// Current control-clock time, in seconds.
    pub now: f64,
    /// World-frame rover pose; position is metres and yaw is radians.
    pub pose: Pose,
    /// Latest observed occupancy snapshot, when available.
    pub map: Option<&'a MapSnapshot>,
    /// Whether the local planner could not provide admissible motion.
    pub planner_blocked: bool,
    /// Whether the follower is currently tracking a goal.
    pub goal_active: bool,
    /// Whether the follower has reached its current goal.
    pub goal_arrived: bool,
    /// Tracked follower goal x coordinate, in world metres.
    pub goal_x: f64,
    /// Tracked follower goal y coordinate, in world metres.
    pub goal_y: f64,
    /// Follower distance to its goal, in metres.
    pub goal_distance: f64,
}
/// Frontier selection and the run state machine. Platform code stays outside.
pub struct Explorer {
    config: ExplorationConfig,
    objective: Box<dyn MissionObjective>,
    scorer: Box<dyn GoalScorer>,
    phase: ExplorePhase,
    end_reason: Option<EndReason>,
    run_id: String,
    budget_seconds: f64,
    started_at: Option<f64>,
    ended_elapsed: Option<f64>,
    last_now: f64,
    target: Option<FrontierTarget>,
    blacklist: Vec<BlackEntry>,
    blocked_since: Option<f64>,
    best_distance: Option<f64>,
    progress_since: Option<f64>,
    frontiers_attempted: u64,
    last_progress_emit: f64,
    coverage_m2: f64,
    explored_ratio: f64,
    detection_count: usize,
    scan: Option<Scan>,
    /// A look-around finished since the last frontier assignment.
    scan_done: bool,
    events: Vec<ExploreEvent>,
}
impl Default for Explorer {
    fn default() -> Self {
        Self {
            config: ExplorationConfig::default(),
            objective: Box::new(TimeBudgetObjective::default()),
            scorer: Box::new(InformationGainScorer),
            phase: ExplorePhase::Idle,
            end_reason: None,
            run_id: String::new(),
            budget_seconds: 0.,
            started_at: None,
            ended_elapsed: None,
            last_now: 0.,
            target: None,
            blacklist: Vec::new(),
            blocked_since: None,
            best_distance: None,
            progress_since: None,
            frontiers_attempted: 0,
            last_progress_emit: 0.,
            coverage_m2: 0.,
            explored_ratio: 0.,
            detection_count: 0,
            scan: None,
            scan_done: false,
            events: Vec::new(),
        }
    }
}
impl Explorer {
    /// Borrow the exploration configuration.
    pub fn config(&self) -> &ExplorationConfig {
        &self.config
    }
    /// Mutably borrow configuration; callers are responsible for supplying meaningful finite distances and durations.
    pub fn config_mut(&mut self) -> &mut ExplorationConfig {
        &mut self.config
    }
    /// Replace the stop policy. A subsequent run start resets it and supplies the run budget.
    pub fn set_objective(&mut self, objective: Box<dyn MissionObjective>) {
        self.objective = objective;
    }
    /// Replace frontier ranking; larger finite scores win and nonfinite scores are skipped.
    pub fn set_scorer(&mut self, scorer: Box<dyn GoalScorer>) {
        self.scorer = scorer;
    }
    /// Set a finite footprint radius in `(0, 5]` metres; ignore invalid values.
    pub fn set_footprint(&mut self, radius: f64) {
        if radius.is_finite() && radius > 0. && radius <= 5. {
            self.config.footprint_radius = radius;
        }
    }
    /// Return whether exploration is currently running.
    pub fn is_running(&self) -> bool {
        self.phase == ExplorePhase::Running
    }
    /// Return the number of frontier assignments made in this run.
    pub fn frontiers_attempted(&self) -> u64 {
        self.frontiers_attempted
    }
    /// Return the current lifecycle phase without advancing time.
    pub fn phase(&self) -> ExplorePhase {
        self.phase
    }
    /// Return the terminal reason, or `None` before the run ends.
    pub fn end_reason(&self) -> Option<EndReason> {
        self.end_reason
    }
    /// Begin a run. The budget clock starts on the first valid [`Self::step`] call.
    ///
    /// Returns `false` without changing state unless the seconds budget is finite
    /// and in `(0, 86400]`. A valid start clears targets, blacklist, counters, and
    /// objective state, and queues a start event; the run identifier is copied.
    pub fn start(&mut self, budget_seconds: f64, run_id: &str) -> bool {
        if !budget_seconds.is_finite() || budget_seconds <= 0. || budget_seconds > 86_400. {
            return false;
        }
        self.budget_seconds = budget_seconds;
        self.run_id = run_id.to_string();
        self.phase = ExplorePhase::Running;
        self.end_reason = None;
        self.started_at = None;
        self.ended_elapsed = None;
        self.target = None;
        self.blacklist.clear();
        self.blocked_since = None;
        self.best_distance = None;
        self.progress_since = None;
        self.frontiers_attempted = 0;
        self.last_progress_emit = 0.;
        self.detection_count = 0;
        self.scan = None;
        self.scan_done = false;
        self.objective.reset();
        self.objective.set_budget(budget_seconds);
        self.events.push(ExploreEvent {
            kind: "exploration_started",
            reason: format!("budget_seconds={budget_seconds}"),
        });
        true
    }
    /// Record an external detection. The next [`Self::step`] applies `should_stop`.
    ///
    /// Ignore detections outside a running session, empty/over-64-byte labels, or
    /// nonfinite position, confidence, or timestamp values. This hook does not
    /// enforce confidence bounds, freshness, or sensor provenance.
    pub fn notify_detection(&mut self, detection: Detection) {
        if self.phase != ExplorePhase::Running {
            return;
        }
        if !detection.x.is_finite()
            || !detection.y.is_finite()
            || !detection.confidence.is_finite()
            || !detection.observed_at.is_finite()
            || detection.label.is_empty()
            || detection.label.len() > 64
        {
            return;
        }
        self.detection_count = self.detection_count.saturating_add(1);
        self.objective.on_detection(&detection);
        self.events.push(ExploreEvent {
            kind: "detection_reported",
            reason: format!("label={}", detection.label),
        });
    }
    /// End an active run, freezing elapsed time at its last tick; idle and ended runs are unchanged.
    pub fn stop(&mut self, reason: EndReason) {
        if self.phase != ExplorePhase::Running {
            return;
        }
        self.finish(reason);
    }
    /// Drain all queued exploration events in emission order.
    pub fn take_events(&mut self) -> Vec<ExploreEvent> {
        std::mem::take(&mut self.events)
    }
    /// Snapshot run progress at `now` seconds without changing state; terminal elapsed time stays frozen.
    pub fn status(&self, now: f64) -> ExplorationStatus {
        let elapsed = self.elapsed(now);
        ExplorationStatus {
            run_id: self.run_id.clone(),
            phase: self.phase,
            elapsed_seconds: elapsed,
            remaining_seconds: (self.budget_seconds - elapsed).max(0.),
            budget_seconds: self.budget_seconds,
            target: self.target,
            coverage_m2: self.coverage_m2,
            explored_ratio: self.explored_ratio,
            end_reason: self.end_reason,
            blacklist_count: self.blacklist.len() as u32,
            frontiers_attempted: self.frontiers_attempted,
            scanning: self.scan.is_some(),
        }
    }
    /// Update coverage and advance one frontier-selection tick.
    ///
    /// Use finite nonnegative seconds on a consistent clock, a valid occupancy snapshot, and world-metre follower coordinates.
    /// Invalid time returns the current terminal/idle action; missing usable frontiers ends the run unless a blacklist retry is pending.
    pub fn step(&mut self, tick: ExploreTick<'_>) -> ExploreAction {
        if let Some(map) = tick.map
            && let Some((area, ratio)) = map_coverage(map)
        {
            self.coverage_m2 = area;
            self.explored_ratio = ratio;
        }
        if !tick.now.is_finite() || tick.now < 0. {
            return self.terminal_action();
        }
        self.last_now = tick.now;
        if self.phase != ExplorePhase::Running {
            return self.terminal_action();
        }
        if self.started_at.is_none() {
            self.started_at = Some(tick.now);
        }
        let elapsed = self.elapsed(tick.now);
        if elapsed >= self.last_progress_emit + self.config.progress_period.max(0.1) {
            self.last_progress_emit = elapsed;
            self.events.push(ExploreEvent {
                kind: "exploration_progress",
                reason: format!(
                    "coverage_m2={:.3},elapsed={elapsed:.3},remaining={:.3}",
                    self.coverage_m2,
                    (self.budget_seconds - elapsed).max(0.)
                ),
            });
        }
        let ctx = ObjectiveContext {
            now: tick.now,
            elapsed_seconds: elapsed,
            remaining_seconds: (self.budget_seconds - elapsed).max(0.),
            pose_x: tick.pose.x,
            pose_y: tick.pose.y,
            coverage_m2: self.coverage_m2,
            detection_count: self.detection_count,
        };
        if let Some(reason) = self.objective.should_stop(&ctx) {
            self.finish(reason);
            return ExploreAction::Finished(reason);
        }
        if let Some(target) = self.target {
            let tracking = (tick.goal_active || tick.goal_arrived)
                && (tick.goal_x - target.x).hypot(tick.goal_y - target.y) < 0.05;
            if tracking && tick.goal_arrived {
                self.target = None;
                self.blocked_since = None;
                self.best_distance = None;
            } else if tracking && tick.goal_active {
                let unreachable = tick.map.is_none_or(|map| {
                    !reachable(
                        map,
                        tick.pose,
                        self.config.footprint_radius,
                        target.x,
                        target.y,
                    )
                });
                if unreachable {
                    self.block_target(tick.now);
                } else if tick.planner_blocked {
                    let since = *self.blocked_since.get_or_insert(tick.now);
                    if tick.now - since >= self.config.stuck_timeout.max(0.) {
                        self.block_target(tick.now);
                    }
                } else {
                    self.blocked_since = None;
                    let improved = self.best_distance.is_none_or(|best| {
                        tick.goal_distance < best - self.config.progress_epsilon
                    });
                    if improved {
                        self.best_distance = Some(tick.goal_distance);
                        self.progress_since = Some(tick.now);
                    } else {
                        let since = self.progress_since.unwrap_or(tick.now);
                        if tick.now - since >= self.config.stuck_timeout.max(0.) {
                            self.block_target(tick.now);
                        }
                    }
                }
            }
        }
        if self.phase != ExplorePhase::Running {
            return self.terminal_action();
        }
        if self.target.is_none()
            && let Some(map) = tick.map
            && let Some(candidate) = self.choose(map, tick.pose, tick.now)
        {
            self.assign(candidate, tick.now);
        }
        if let Some(target) = self.target {
            ExploreAction::Seek {
                x: target.x,
                y: target.y,
            }
        } else if self.retry_pending(tick.now) {
            ExploreAction::Hold
        } else if let Some(yaw_rate) = self.scan_step(tick.map, tick.pose, tick.now) {
            ExploreAction::Scan { yaw_rate }
        } else {
            self.finish(EndReason::NoFrontiersLeft);
            ExploreAction::Finished(EndReason::NoFrontiersLeft)
        }
    }
    fn terminal_action(&self) -> ExploreAction {
        match self.phase {
            ExplorePhase::Ended => {
                ExploreAction::Finished(self.end_reason.unwrap_or(EndReason::OperatorStop))
            }
            ExplorePhase::Idle => ExploreAction::Idle,
            ExplorePhase::Running => ExploreAction::Idle,
        }
    }
    fn choose(&self, map: &MapSnapshot, pose: Pose, now: f64) -> Option<FrontierCandidate> {
        let excluded = self.excluded(now);
        select_frontier(
            map,
            pose,
            self.config.footprint_radius,
            &excluded,
            self.config.exclude_radius,
            self.scorer.as_ref(),
        )
    }
    /// Advance the look-around. `None` once it is finished, disabled, or not needed.
    ///
    /// Frontier selection runs before this every tick, so a frontier that comes
    /// into view mid-turn ends the look-around at once.
    fn scan_step(&mut self, map: Option<&MapSnapshot>, pose: Pose, now: f64) -> Option<f64> {
        let rate = self.config.scan_yaw_rate;
        if self.scan_done || !rate.is_finite() || rate <= 0. || !pose.finite() {
            return None;
        }
        let scan = match self.scan.as_mut() {
            Some(scan) => {
                let step = (pose.yaw - scan.last_yaw + std::f64::consts::PI)
                    .rem_euclid(std::f64::consts::TAU)
                    - std::f64::consts::PI;
                scan.turned += step.abs();
                scan.last_yaw = pose.yaw;
                *scan
            }
            None => {
                let blind = map.is_some_and(|map| {
                    unknown_within(
                        map,
                        pose,
                        self.config.footprint_radius + self.config.scan_radius,
                    )
                });
                if !blind {
                    return None;
                }
                let scan = Scan {
                    started_at: now,
                    last_yaw: pose.yaw,
                    turned: 0.,
                };
                self.scan = Some(scan);
                self.events.push(ExploreEvent {
                    kind: "exploration_scan_started",
                    reason: "no_reachable_frontier".into(),
                });
                scan
            }
        };
        if scan.turned >= std::f64::consts::TAU || now - scan.started_at >= self.config.scan_timeout
        {
            self.scan = None;
            self.scan_done = true;
            self.events.push(ExploreEvent {
                kind: "exploration_scan_done",
                reason: format!(
                    "turned_rad={:.3},seconds={:.3}",
                    scan.turned,
                    now - scan.started_at
                ),
            });
            return None;
        }
        Some(rate)
    }
    fn excluded(&self, now: f64) -> Vec<(f64, f64)> {
        self.blacklist
            .iter()
            .filter(|entry| entry.retry_at > now)
            .map(|entry| (entry.x, entry.y))
            .collect()
    }
    fn retry_pending(&self, now: f64) -> bool {
        self.blacklist.iter().any(|entry| {
            entry.attempts < self.config.max_attempts.max(1)
                && entry.retry_at.is_finite()
                && entry.retry_at > now
        })
    }
    fn assign(&mut self, candidate: FrontierCandidate, now: f64) {
        self.scan = None;
        self.scan_done = false;
        self.target = Some(FrontierTarget {
            x: candidate.x,
            y: candidate.y,
            score: candidate.score,
        });
        self.blocked_since = None;
        self.best_distance = None;
        self.progress_since = Some(now);
        self.frontiers_attempted = self.frontiers_attempted.saturating_add(1);
        self.events.push(ExploreEvent {
            kind: "frontier_selected",
            reason: format!(
                "x={:.2},y={:.2},score={:.3}",
                candidate.x, candidate.y, candidate.score
            ),
        });
    }
    fn block_target(&mut self, now: f64) {
        let Some(target) = self.target.take() else {
            return;
        };
        self.blocked_since = None;
        self.best_distance = None;
        self.progress_since = None;
        let radius = self.config.exclude_radius;
        if let Some(entry) = self
            .blacklist
            .iter_mut()
            .find(|entry| (entry.x - target.x).hypot(entry.y - target.y) <= radius)
        {
            entry.attempts = entry.attempts.saturating_add(1);
            entry.x = target.x;
            entry.y = target.y;
            entry.retry_at = if entry.attempts >= self.config.max_attempts.max(1) {
                f64::INFINITY
            } else {
                now + self.config.blacklist_seconds.max(0.)
            };
        } else {
            let attempts = 1;
            self.blacklist.push(BlackEntry {
                x: target.x,
                y: target.y,
                attempts,
                retry_at: if attempts >= self.config.max_attempts.max(1) {
                    f64::INFINITY
                } else {
                    now + self.config.blacklist_seconds.max(0.)
                },
            });
            if self.blacklist.len() > 256 {
                self.blacklist.remove(0);
            }
        }
        self.events.push(ExploreEvent {
            kind: "frontier_blocked",
            reason: format!("x={:.2},y={:.2}", target.x, target.y),
        });
    }
    fn finish(&mut self, reason: EndReason) {
        if self.phase != ExplorePhase::Running {
            return;
        }
        self.ended_elapsed = Some(self.elapsed(self.last_now));
        self.phase = ExplorePhase::Ended;
        self.end_reason = Some(reason);
        self.target = None;
        self.scan = None;
        self.events.push(ExploreEvent {
            kind: "exploration_ended",
            reason: format!(
                "{},coverage_m2={:.3},elapsed={:.3}",
                reason.as_str(),
                self.coverage_m2,
                self.ended_elapsed.unwrap_or(0.)
            ),
        });
    }
    fn elapsed(&self, now: f64) -> f64 {
        if let Some(frozen) = self.ended_elapsed {
            return frozen;
        }
        self.started_at
            .map(|start| (now - start).max(0.))
            .unwrap_or(0.)
    }
}
/// Whether any unknown cell centre lies within `distance` metres of `pose`.
fn unknown_within(map: &MapSnapshot, pose: Pose, distance: f64) -> bool {
    let Some((cx, cy)) = terra_navigation::grid_cell(map, pose.x, pose.y) else {
        return false;
    };
    let reach = (distance / map.resolution).ceil() as isize + 1;
    for dy in -reach..=reach {
        for dx in -reach..=reach {
            let a = cx as isize + dx;
            let b = cy as isize + dy;
            if a < 0 || b < 0 || a >= map.width as isize || b >= map.height as isize {
                continue;
            }
            let x = map.origin_x + (a as f64 + 0.5) * map.resolution;
            let y = map.origin_y + (b as f64 + 0.5) * map.resolution;
            if (x - pose.x).hypot(y - pose.y) <= distance
                && map.occupancy[b as usize * map.width as usize + a as usize] < 0
            {
                return true;
            }
        }
    }
    false
}
