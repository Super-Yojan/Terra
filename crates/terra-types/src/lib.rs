//! SI units; rover body axes are +X forward, +Y left, +Z up.
//! All timestamps use the same monotonic clock, in seconds.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}
impl Vector3 {
    pub const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    pub fn finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
    pub fn scaled(self, scale: f64) -> Self {
        Self {
            x: self.x * scale,
            y: self.y * scale,
            z: self.z * scale,
        }
    }
    pub fn plus(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
    pub fn cross(self, other: Self) -> Self {
        Self {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}
impl Default for Quaternion {
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        }
    }
}
impl Quaternion {
    pub fn normalized(self) -> Option<Self> {
        let norm = self.x.hypot(self.y).hypot(self.z).hypot(self.w);
        if !norm.is_finite() || norm < 1e-12 {
            return None;
        }
        Some(Self {
            x: self.x / norm,
            y: self.y / norm,
            z: self.z / norm,
            w: self.w / norm,
        })
    }
    pub fn conjugate(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
            w: self.w,
        }
    }
    pub fn compose(self, b: Self) -> Self {
        Self {
            x: self.w * b.x + self.x * b.w + self.y * b.z - self.z * b.y,
            y: self.w * b.y - self.x * b.z + self.y * b.w + self.z * b.x,
            z: self.w * b.z + self.x * b.y - self.y * b.x + self.z * b.w,
            w: self.w * b.w - self.x * b.x - self.y * b.y - self.z * b.z,
        }
    }
    /// Rotate a vector; callers must normalize the quaternion first.
    pub fn rotate(self, v: Vector3) -> Vector3 {
        let q = Vector3 {
            x: self.x,
            y: self.y,
            z: self.z,
        };
        let t = q.cross(v).scaled(2.0);
        v.plus(t.scaled(self.w)).plus(q.cross(t))
    }
    pub fn from_rotation_vector(v: Vector3) -> Self {
        let angle = v.x.hypot(v.y).hypot(v.z);
        let scale = if angle < 1e-8 {
            0.5
        } else {
            (angle * 0.5).sin() / angle
        };
        Self {
            x: v.x * scale,
            y: v.y * scale,
            z: v.z * scale,
            w: (angle * 0.5).cos(),
        }
    }
}
#[derive(Clone, Copy, Debug)]
pub struct ImuSample {
    pub timestamp: f64,
    /// Gravity removed and calibrated, expressed in rover body axes, m/s².
    pub acceleration: Vector3,
    /// Calibrated rover body angular velocity, rad/s.
    pub angular_velocity: Vector3,
}
#[derive(Clone, Copy, Debug)]
pub struct VioSample {
    pub timestamp: f64,
    pub position: Vector3,
    /// Rover body to world rotation. World must be metric and +Z up.
    pub orientation: Quaternion,
    /// Rover origin velocity in world axes, m/s.
    pub velocity: Vector3,
    pub tracked: bool,
}
#[derive(Clone, Copy, Debug, Default)]
pub struct VelocityTarget {
    pub timestamp: f64,
    pub forward: f64,
    pub yaw_rate: f64,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Health {
    Ready,
    MissingImu,
    MissingVio,
    TrackingLost,
    StaleImu,
    StaleVio,
    InvalidTime,
}
#[derive(Clone, Copy, Debug)]
pub struct VelocityEstimate {
    pub timestamp: f64,
    pub forward: f64,
    pub yaw_rate: f64,
    pub health: Health,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StopReason {
    None,
    SensorNotReady,
    TrackingLost,
    StaleSensors,
    StaleTarget,
    InvalidTime,
}
#[derive(Clone, Copy, Debug)]
pub struct MotorOutput {
    /// Signed effort [-1,1]; motor driver maps sign to direction and magnitude to duty.
    pub left: f64,
    pub right: f64,
    pub estimated_forward: f64,
    pub estimated_yaw_rate: f64,
    pub target_forward: f64,
    pub target_yaw_rate: f64,
    pub stop_reason: StopReason,
}
impl MotorOutput {
    pub fn stopped(reason: StopReason) -> Self {
        Self {
            left: 0.0,
            right: 0.0,
            estimated_forward: 0.0,
            estimated_yaw_rate: 0.0,
            target_forward: 0.0,
            target_yaw_rate: 0.0,
            stop_reason: reason,
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InputError {
    NonFinite,
    InvalidQuaternion,
    OutOfOrder,
    OutOfRange,
    InvalidConfiguration,
}
impl std::fmt::Display for InputError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for InputError {}
