//! Shared foundation: rotation matrices and poses (roadmap 1).

pub struct Vec2 { pub x: f64, pub y: f64 }
pub struct Vec3 { pub x: f64, pub y: f64, pub z: f64 }

/// 2×2 rotation matrix.
pub struct Rotation2 { m: [[f64; 2]; 2] }
/// 3×3 rotation matrix, for sensor → body frame changes (phone mount, IMU).
pub struct Rotation3 { m: [[f64; 3]; 3] }

/// Planar pose: position and heading `yaw` (rad, + = left of +X).
pub struct Pose2 { pub x: f64, pub y: f64, pub yaw: f64 }

impl Rotation2 {
    pub fn new(theta: f64) -> Self { todo!() }
    pub fn transpose(&self) -> Self { todo!() }
    pub fn apply(&self, v: Vec2) -> Vec2 { todo!() }
}

impl Rotation3 {
    pub fn from_euler(roll: f64, pitch: f64, yaw: f64) -> Self { todo!() }
    /// Row-major 3×3, e.g. the rotation part of ARKit's camera transform.
    pub fn from_matrix(m: [[f64; 3]; 3]) -> Self { Self { m } }
    pub fn transpose(&self) -> Self { todo!() }
    pub fn apply(&self, v: Vec3) -> Vec3 { todo!() }
}

impl Pose2 {
    /// Body-frame point → world.
    pub fn to_world(&self, p: Vec2) -> Vec2 { todo!() }
    /// World point → body frame (+X forward, +Y left).
    pub fn to_body(&self, p: Vec2) -> Vec2 { todo!() }
}
