//! Wires the stages together. One call per control tick.
use crate::estimation::{State, StateEstimator};
use crate::execution::{Controller, SafetyGate, WheelCommand};
use crate::mapping::Mapper;
use crate::planning::{global::PathPlanner, local::LocalPlanner, Goal, Path};
use crate::sense::SensorFrame;
use crate::vehicle::{Twist, VehicleModel};

pub struct Pipeline {
    pub vehicle: Box<dyn VehicleModel>,
    pub estimator: Box<dyn StateEstimator>,
    pub mapper: Box<dyn Mapper>,
    pub path_planner: Box<dyn PathPlanner>,
    pub local_planner: Box<dyn LocalPlanner>,
    pub controller: Box<dyn Controller>,
    pub safety: SafetyGate,
}

pub struct TickOutput { pub state: State, pub path: Path, pub target: Twist, pub command: WheelCommand }

impl Pipeline {
    /// sense → estimate → map → global plan → local plan → control → safety.
    /// The global planner re-runs only when the goal or map changed.
    pub fn tick(&mut self, frame: &SensorFrame, goal: &Goal) -> TickOutput { todo!() }
}
