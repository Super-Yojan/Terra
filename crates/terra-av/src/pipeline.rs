//! Wires the stages together. One call per control tick.
use crate::estimation::{State, StateEstimator};
use crate::execution::{Controller, SafetyGate, WheelCommand};
use crate::mapping::Mapper;
use crate::planning::{Goal, Planner};
use crate::sense::SensorFrame;
use crate::vehicle::{Twist, VehicleModel};

pub struct Pipeline {
    pub vehicle: Box<dyn VehicleModel>,
    pub estimator: Box<dyn StateEstimator>,
    pub mapper: Box<dyn Mapper>,
    pub planner: Box<dyn Planner>,
    pub controller: Box<dyn Controller>,
    pub safety: SafetyGate,
}

pub struct TickOutput { pub state: State, pub target: Twist, pub command: WheelCommand }

impl Pipeline {
    /// sense → estimate → map → plan → control → safety.
    pub fn tick(&mut self, frame: &SensorFrame, goal: &Goal) -> TickOutput { todo!() }
}
