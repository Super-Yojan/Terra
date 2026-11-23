//! Stage 1, Sensors. Raw phone readings exactly as the phone reports them
//! (ARKit, Core Motion, Core Location, LiDAR depth), in the sensor's own
//! frame. Turning them into the rover's body frame is estimation's job.
use crate::geometry::{Rotation3, Vec3};

pub struct SensorFrame {
    pub t: f64,
    pub estop: bool,          // phone e-stop pressed
    pub vio: Option<VioPose>, // ARKit camera pose
    pub gyro: Option<Vec3>,   // Core Motion rotation rate, phone frame (rad/s)
    pub gps: Option<GpsFix>,  // Core Location
    pub depth: Vec<Vec3>,     // thinned LiDAR points, camera frame (m)
}

/// ARKit `camera.transform`: camera → ARKit world.
#[derive(Debug)]
pub struct VioPose {
    pub position: Vec3,
    pub rotation: Rotation3,
    pub tracked: bool,
}

/// Core Location fix: WGS84 degrees, horizontal accuracy in metres (1σ).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GpsFix {
    pub latitude: f64,
    pub longitude: f64,
    pub accuracy: f64,
}

/// Where the phone sits on the rover. Set once in the config (TOML), not in
/// the app. Used by estimation (VIO pose) and mapping (depth points).
#[derive(Clone, Debug)]
pub struct Mount {
    /// Camera axes → body axes (+X forward, +Y left, +Z up).
    pub camera_to_body: Rotation3,
    /// Camera position in the body frame (m).
    pub camera_offset: Vec3,
}
