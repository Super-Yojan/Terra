//! Stage 1, Sensors. Plain data filled by the phone (ARKit, Core Motion,
//! Core Location), a simulator or a log replay; nothing below this line
//! talks to hardware.
use crate::geometry::{Pose2, Vec2};

pub struct SensorFrame {
    pub t: f64,
    pub vio: Option<VioPose>,          // ARKit pose, camera frame
    pub gyro_z: Option<f64>,           // IMU yaw rate
    pub gps: Option<GpsFix>,           // Core Location
    pub obstacles: Vec<Vec2>,          // depth returns, body frame
}

pub struct VioPose { pub pose: Pose2, pub vx: f64, pub vy: f64, pub tracked: bool }
pub struct GpsFix { pub latitude: f64, pub longitude: f64, pub accuracy: f64 }
