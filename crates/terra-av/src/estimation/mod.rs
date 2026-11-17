//! Stage 2, State estimation: where am I, how fast am I moving.
mod vio;
mod localization;
pub use vio::Vio;
pub use localization::Localization;

use crate::geometry::Pose2;
use crate::sense::SensorFrame;
use crate::vehicle::{Twist, VehicleModel};

pub struct State { pub t: f64, pub pose: Pose2, pub twist: Twist }

pub trait StateEstimator: Send {
    fn update(&mut self, frame: &SensorFrame, vehicle: &dyn VehicleModel) -> State;
}
