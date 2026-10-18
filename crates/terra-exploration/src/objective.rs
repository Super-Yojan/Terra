use serde::{Deserialize, Serialize};
/// Why an exploration run stopped. Serialized on `exploration/status` and `autonomy/status`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EndReason {
    /// The configured exploration time budget elapsed.
    BudgetExpired,
    /// No eligible reachable frontier or pending retry remains.
    NoFrontiersLeft,
    /// Operator stop or goal cancellation ended exploration.
    OperatorStop,
    /// Authority changed away from exploration.
    OperatorTakeover,
    /// Required occupancy evidence remained stale beyond the hold allowance.
    MapStale,
    /// Sensor or safety health prevented continued exploration.
    SafetyHold,
    /// Phase 2 stub: a caller reported a wanted detection through [`MissionObjective::on_detection`].
    ObjectiveComplete,
}
impl EndReason {
    /// Return the stable snake-case telemetry spelling of this end reason.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::BudgetExpired => "budget_expired",
            Self::NoFrontiersLeft => "no_frontiers_left",
            Self::OperatorStop => "operator_stop",
            Self::OperatorTakeover => "operator_takeover",
            Self::MapStale => "map_stale",
            Self::SafetyHold => "safety_hold",
            Self::ObjectiveComplete => "objective_complete",
        }
    }
}
/// A detection fed in by a later perception stage. This crate never produces one.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Detection {
    /// Externally supplied target label; the explorer accepts 1–64 bytes.
    pub label: String,
    /// World-frame x coordinate, in metres.
    pub x: f64,
    /// World-frame y coordinate, in metres.
    pub y: f64,
    /// Externally supplied confidence, conventionally 0–1; this hook validates finiteness only.
    pub confidence: f64,
    /// Observation time in seconds on the run clock.
    pub observed_at: f64,
}
/// Read-only view passed to [`MissionObjective::should_stop`].
#[derive(Clone, Copy, Debug)]
pub struct ObjectiveContext {
    /// Current control-clock time, in seconds.
    pub now: f64,
    /// Elapsed run time in seconds; frozen once the run ends.
    pub elapsed_seconds: f64,
    /// Remaining budget in seconds, clamped to zero.
    pub remaining_seconds: f64,
    /// Rover world x coordinate, in metres.
    pub pose_x: f64,
    /// Rover world y coordinate, in metres.
    pub pose_y: f64,
    /// Observed free-cell area in square metres.
    pub coverage_m2: f64,
    /// Number of valid detections forwarded during this run.
    pub detection_count: usize,
}
/// Phase 2 extension point. Phase 1 uses [`TimeBudgetObjective`].
///
/// `should_stop` ends the run. `on_detection` records an external sighting so a
/// later objective can bias the stop decision. Goal choice uses [`crate::GoalScorer`].
pub trait MissionObjective: Send {
    /// Supply the run budget in seconds; the default hook does nothing.
    fn set_budget(&mut self, _seconds: f64) {}
    /// Reset per-run objective state; the default hook does nothing.
    fn reset(&mut self) {}
    /// Return a terminal reason when the objective is satisfied or exhausted; `None` continues exploration.
    fn should_stop(&self, ctx: &ObjectiveContext) -> Option<EndReason>;
    /// Process an externally supplied detection; implementations choose whether it affects completion.
    fn on_detection(&mut self, detection: &Detection);
}
/// Stop when the configured budget has elapsed. Ignores detections.
#[derive(Clone, Debug)]
pub struct TimeBudgetObjective {
    /// Run time budget in seconds; positive values enable budget expiry.
    pub budget_seconds: f64,
}
impl Default for TimeBudgetObjective {
    fn default() -> Self {
        Self { budget_seconds: 0. }
    }
}
impl MissionObjective for TimeBudgetObjective {
    fn set_budget(&mut self, seconds: f64) {
        self.budget_seconds = seconds;
    }
    fn should_stop(&self, ctx: &ObjectiveContext) -> Option<EndReason> {
        (ctx.elapsed_seconds >= self.budget_seconds && self.budget_seconds > 0.)
            .then_some(EndReason::BudgetExpired)
    }
    fn on_detection(&mut self, _detection: &Detection) {}
}
/// Phase 2 stub. Stops when a wanted label is reported, or when the budget elapses.
///
/// Nothing in this crate classifies camera frames. A simulator or a future detector
/// calls [`crate::Explorer::notify_detection`].
#[derive(Clone, Debug)]
pub struct FindTargetsObjective {
    /// Run time budget in seconds; positive values enable budget expiry.
    pub budget_seconds: f64,
    /// Exact labels whose detections satisfy this objective.
    pub wanted: Vec<String>,
    /// Matching detections retained by the objective, capped at 64.
    pub found: Vec<Detection>,
}
impl FindTargetsObjective {
    /// Create a label-matching objective with no found detections and no active time budget.
    pub fn new(wanted: impl IntoIterator<Item = impl Into<String>>) -> Self {
        Self {
            budget_seconds: 0.,
            wanted: wanted.into_iter().map(Into::into).collect(),
            found: Vec::new(),
        }
    }
}
impl MissionObjective for FindTargetsObjective {
    fn set_budget(&mut self, seconds: f64) {
        self.budget_seconds = seconds;
    }
    fn reset(&mut self) {
        self.found.clear();
    }
    fn should_stop(&self, ctx: &ObjectiveContext) -> Option<EndReason> {
        if !self.found.is_empty() {
            Some(EndReason::ObjectiveComplete)
        } else if self.budget_seconds > 0. && ctx.elapsed_seconds >= self.budget_seconds {
            Some(EndReason::BudgetExpired)
        } else {
            None
        }
    }
    fn on_detection(&mut self, detection: &Detection) {
        if self.found.len() >= 64 {
            return;
        }
        if self.wanted.iter().any(|label| label == &detection.label) {
            self.found.push(detection.clone());
        }
    }
}
/// One reachable vantage on the edge of unknown space.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FrontierCandidate {
    /// World-frame x coordinate, in metres.
    pub x: f64,
    /// World-frame y coordinate, in metres.
    pub y: f64,
    /// Number of unknown cells in the candidate neighbourhood.
    pub information_gain: u32,
    /// Breadth-first path length in cardinal grid-cell steps.
    pub travel_cells: usize,
    /// Straight-line distance from the rover to this candidate, in metres.
    pub distance_m: f64,
    /// Scorer output; larger finite values are preferred.
    pub score: f64,
}
/// Phase 2 hook. Higher scores are chosen first. Ties keep the earlier cell in BFS order.
pub trait GoalScorer: Send {
    /// Rank a reachable candidate; larger finite values are preferred and ties retain breadth-first order.
    fn score(&self, candidate: &FrontierCandidate) -> f64;
}
/// `information_gain / (1 + travel_cells)`, matching `terra_navigation::frontier`.
#[derive(Clone, Copy, Debug, Default)]
pub struct InformationGainScorer;
impl GoalScorer for InformationGainScorer {
    fn score(&self, candidate: &FrontierCandidate) -> f64 {
        candidate.information_gain as f64 / (1. + candidate.travel_cells as f64)
    }
}
/// Test and integration helper for a custom score.
pub struct FnScorer<F>(
    /// Closure mapping a frontier candidate to a ranking score.
    pub F,
);
impl<F> GoalScorer for FnScorer<F>
where
    F: Fn(&FrontierCandidate) -> f64 + Send,
{
    fn score(&self, candidate: &FrontierCandidate) -> f64 {
        (self.0)(candidate)
    }
}
