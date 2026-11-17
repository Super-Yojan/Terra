//! Shared foundation: the vehicle model (roadmap 2). Every stage that
//! reasons about motion takes a `&dyn VehicleModel`.
use crate::geometry::Pose2;

/// Body velocity: forward `v` (m/s), yaw rate `w` (rad/s, + = left).
pub struct Twist { pub v: f64, pub w: f64 }
/// Left and right wheel ground speeds, m/s.
pub struct WheelSpeeds { pub left: f64, pub right: f64 }

pub trait VehicleModel: Send {
    /// Inverse kinematics: twist → wheel speeds.
    fn wheels_for(&self, twist: Twist) -> WheelSpeeds;
    /// Forward kinematics: wheel speeds → twist.
    fn twist_for(&self, wheels: WheelSpeeds) -> Twist;
    /// Motion model: pose after driving `twist` for `dt` seconds.
    fn step(&self, pose: Pose2, twist: Twist, dt: f64) -> Pose2;
    /// Speed and acceleration limits the planner and controller respect.
    fn limits(&self) -> Limits;
}

pub struct Limits { pub max_wheel_speed: f64, pub max_accel: f64, pub max_yaw_accel: f64 }

/// Two wheel sides `track` metres apart (Terra, terra-mini).
pub struct DiffDrive { pub track: f64, pub limits: Limits }

impl VehicleModel for DiffDrive {
    fn wheels_for(&self, twist: Twist) -> WheelSpeeds { todo!() }
    fn twist_for(&self, wheels: WheelSpeeds) -> Twist { todo!() }
    fn step(&self, pose: Pose2, twist: Twist, dt: f64) -> Pose2 { todo!() }
    fn limits(&self) -> Limits { todo!() }
}
