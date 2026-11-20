//! Localization (roadmap 6): fuse VIO with GPS so the pose holds a world
//! position over long outdoor runs where VIO drifts (EKF).

use super::{State, StateEstimator};
use crate::{sense::SensorFrame, vehicle::VehicleModel};

pub struct Localization;

impl StateEstimator for Localization {
    fn update(&mut self, frame: &SensorFrame, vehicle: &dyn VehicleModel) -> Option<State> {
        todo!()
    }
}
