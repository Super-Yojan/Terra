//! Integrate measured motion with the vehicle model. Drifts; the baseline.
use super::{State, StateEstimator};
use crate::{sense::SensorFrame, vehicle::VehicleModel};

pub struct DeadReckoning;

impl StateEstimator for DeadReckoning {
    fn update(&mut self, frame: &SensorFrame, vehicle: &dyn VehicleModel) -> State { todo!() }
}
