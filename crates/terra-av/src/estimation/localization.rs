//! Localization (roadmap 6): fuse odometry with absolute pose fixes (EKF).
use super::{State, StateEstimator};
use crate::{sense::SensorFrame, vehicle::VehicleModel};

pub struct Localization;

impl StateEstimator for Localization {
    fn update(&mut self, frame: &SensorFrame, vehicle: &dyn VehicleModel) -> State { todo!() }
}
