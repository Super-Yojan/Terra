use crate::{
    Detection, EndReason, FrontierCandidate, GoalScorer, InformationGainScorer, MissionObjective,
    ObjectiveContext, TimeBudgetObjective, map_coverage, select_frontier,
};
use serde::{Deserialize, Serialize};
use terra_mapping::MapSnapshot;
use terra_navigation::{Pose, reachable};
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ExplorePhase {
    Idle,
    Running,
    Ended,
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct FrontierTarget {
    pub x: f64,
    pub y: f64,
    pub score: f64,
}
/// Published on `terra/rover/<id>/exploration/status` and copied onto autonomy status.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ExplorationStatus {
    pub run_id: String,
    pub phase: ExplorePhase,
    pub elapsed_seconds: f64,
    pub remaining_seconds: f64,
    pub budget_seconds: f64,
    pub target: Option<FrontierTarget>,
    pub coverage_m2: f64,
    pub explored_ratio: f64,
    pub end_reason: Option<EndReason>,
    pub blacklist_count: u32,
    pub frontiers_attempted: u64,
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
}
impl Default for ExplorationConfig {
    fn default() -> Self {
        Self {
            footprint_radius: 0.65,
            stuck_timeout: 5.,
            blacklist_seconds: 30.,
            max_attempts: 3,
            exclude_radius: 1.,
            progress_epsilon: 0.2,
            approach_horizon: 1.,
            progress_period: 1.,
            map_stale_timeout: 0.,
            safety_hold_timeout: 0.,
        }
    }
}
#[derive(Clone, Debug)]
struct BlackEntry {
    x: f64,
    y: f64,
    attempts: u32,
    retry_at: f64,
}
#[derive(Clone, Debug)]
pub struct ExploreEvent {
    pub kind: &'static str,
    pub reason: String,
}
/// What the arbiter should do with the waypoint follower after one exploration tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ExploreAction {
    Idle,
    Seek {
        x: f64,
        y: f64,
    },
    /// No usable frontier yet. The blacklist will expire; do not end the run.
    Hold,
    Finished(EndReason),
}
/// Pose, map, and the goal the follower is already tracking.
pub struct ExploreTick<'a> {
    pub now: f64,
    pub pose: Pose,
    pub map: Option<&'a MapSnapshot>,
    pub planner_blocked: bool,
    pub goal_active: bool,
    pub goal_arrived: bool,
    pub goal_x: f64,
    pub goal_y: f64,
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
            events: Vec::new(),
        }
    }
}
impl Explorer {
    pub fn config(&self) -> &ExplorationConfig {
        &self.config
    }
    pub fn config_mut(&mut self) -> &mut ExplorationConfig {
        &mut self.config
    }
    pub fn set_objective(&mut self, objective: Box<dyn MissionObjective>) {
        self.objective = objective;
    }
    pub fn set_scorer(&mut self, scorer: Box<dyn GoalScorer>) {
        self.scorer = scorer;
    }
    pub fn set_footprint(&mut self, radius: f64) {
        if radius.is_finite() && radius > 0. && radius <= 5. {
            self.config.footprint_radius = radius;
        }
    }
    pub fn is_running(&self) -> bool {
        self.phase == ExplorePhase::Running
    }
    pub fn frontiers_attempted(&self) -> u64 {
        self.frontiers_attempted
    }
    pub fn phase(&self) -> ExplorePhase {
        self.phase
    }
    pub fn end_reason(&self) -> Option<EndReason> {
        self.end_reason
    }
    /// Begin a run. The budget clock starts on the first [`Self::step`] call.
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
        self.objective.reset();
        self.objective.set_budget(budget_seconds);
        self.events.push(ExploreEvent {
            kind: "exploration_started",
            reason: format!("budget_seconds={budget_seconds}"),
        });
        true
    }
    /// Record an external detection. The next [`Self::step`] applies `should_stop`.
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
    pub fn stop(&mut self, reason: EndReason) {
        if self.phase != ExplorePhase::Running {
            return;
        }
        self.finish(reason);
    }
    pub fn take_events(&mut self) -> Vec<ExploreEvent> {
        std::mem::take(&mut self.events)
    }
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
        }
    }
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
