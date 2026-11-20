//! Stage 2, State estimation: where am I, how fast am I moving.
mod vio;
mod localization;
pub use vio::Vio;
pub use localization::Localization;

use crate::geometry::Pose2;
use crate::sense::SensorFrame;
use crate::vehicle::{Twist, VehicleModel};

/// The rover's estimated state at time `t`: body pose in the world frame
/// and body twist (forward m/s, yaw rad/s).
pub struct State { pub t: f64, pub pose: Pose2, pub twist: Twist }

pub trait StateEstimator: Send {
    /// Fold in this frame's readings and return the current state.
    ///
    /// `None` means there is no trustworthy estimate (no tracked VIO pose
    /// yet, or tracking lost). Never invent a pose to fill the gap: the
    /// pipeline stops the rover on `None` and sends `measured: None`, so
    /// the velocity loop runs feedforward only.
    fn update(&mut self, frame: &SensorFrame, vehicle: &dyn VehicleModel) -> Option<State>;
}
