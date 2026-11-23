//! Stage 2, State estimation: where am I, how fast am I moving.
mod gps;
mod localization;
mod vio;
pub use gps::GpsAnchor;
pub use localization::Localization;
pub use vio::Vio;
pub(crate) use vio::arkit_to_world;

use crate::geometry::Pose2;
use crate::sense::SensorFrame;
use crate::vehicle::{Twist, VehicleModel};

/// The rover's estimated state at time `t`: body pose in the world frame
/// and body twist (forward m/s, yaw rad/s).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct State {
    pub t: f64,
    pub pose: Pose2,
    pub twist: Twist,
}

pub trait StateEstimator: Send {
    /// Fold in this frame's readings and return the current state.
    ///
    /// `None` means there is no trustworthy estimate (no tracked VIO pose
    /// yet, or tracking lost). Never invent a pose to fill the gap: the
    /// pipeline stops the rover on `None` and sends `measured: None`, so
    /// the velocity loop runs feedforward only.
    fn update(&mut self, frame: &SensorFrame, vehicle: &dyn VehicleModel) -> Option<State>;

    /// Where the world frame sits on Earth, if this estimator knows. Needed
    /// to drive to a GPS goal. `None` for an estimator without GPS, and
    /// until there is a fit.
    fn gps_anchor(&self) -> Option<GpsAnchor> {
        None
    }
}
