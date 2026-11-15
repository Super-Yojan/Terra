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
    /// The host reached the run's goal, such as the GPS destination of an outdoor explore.
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
/// Frontier ranking. Higher scores are chosen first. Ties keep the earlier cell in BFS order.
pub trait GoalScorer: Send + Sync {
    /// Rank a reachable candidate; larger finite values are preferred and ties retain breadth-first order.
    fn score(&self, candidate: &FrontierCandidate) -> f64;
}
/// `information_gain / (1 + travel_cells)`.
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
    F: Fn(&FrontierCandidate) -> f64 + Send + Sync,
{
    fn score(&self, candidate: &FrontierCandidate) -> f64 {
        (self.0)(candidate)
    }
}
