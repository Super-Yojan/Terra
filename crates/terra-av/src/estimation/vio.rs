//! ARKit (or any VIO) already estimates the pose: move it from the camera
//! frame to the vehicle body (mount rotation + offset) and pass it through.
//! The simulator's ground-truth pose uses the same path.
use super::{State, StateEstimator};
use crate::{sense::SensorFrame, vehicle::VehicleModel};

pub struct Vio;

impl StateEstimator for Vio {
    fn update(&mut self, frame: &SensorFrame, vehicle: &dyn VehicleModel) -> State { todo!() }
}
