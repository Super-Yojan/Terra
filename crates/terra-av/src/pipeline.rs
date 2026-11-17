//! Wires the stages together. One call per control tick.
use crate::estimation::StateEstimator;
use crate::execution::SafetyGate;
use crate::mapping::Mapper;
use crate::planning::{global::PathPlanner, local::LocalPlanner, Goal};
use crate::sense::SensorFrame;
use crate::vehicle::{Twist, VehicleModel};

pub struct Pipeline {
    pub vehicle: Box<dyn VehicleModel>,
    pub estimator: Box<dyn StateEstimator>,
    pub mapper: Box<dyn Mapper>,
    pub path_planner: Box<dyn PathPlanner>,
    pub local_planner: Box<dyn LocalPlanner>,
    pub safety: SafetyGate,
}

/// What the platform's velocity loop needs: where to go, and how fast the
/// rover is going now (from estimation; `None` until the pose is tracked).
pub struct DriveCommand { pub target: Twist, pub measured: Option<Twist> }

impl Pipeline {
    /// sense → estimate → map → global plan → local plan → safety.
    /// The global planner re-runs only when the goal or map changed.
    pub fn tick(&mut self, frame: &SensorFrame, goal: &Goal) -> DriveCommand { todo!() }
}
