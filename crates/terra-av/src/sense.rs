//! Stage 1, Sensors. Plain data filled by the phone, a simulator or a log
//! replay; nothing below this line talks to hardware.
use crate::geometry::{Pose2, Vec2};
use crate::vehicle::WheelSpeeds;

pub struct SensorFrame {
    pub t: f64,
    pub wheels: Option<WheelSpeeds>,   // encoders
    pub gyro_z: Option<f64>,           // IMU yaw rate
    pub pose_fix: Option<PoseFix>,     // VIO / GPS / mocap
    pub obstacles: Vec<Vec2>,          // depth or lidar returns, body frame
}

pub struct PoseFix { pub pose: Pose2, pub position_sigma: f64, pub yaw_sigma: f64 }
