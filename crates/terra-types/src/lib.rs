//! Shared math, sensor samples, velocity targets, and safety results.
//!
//! All timestamps are seconds on one monotonic clock. Length and velocity use SI
//! units; rover body axes are +X forward, +Y left, +Z up. [`Quaternion`] carries
//! orientation and [`MotorOutput`] carries normalized wheel effort. This crate
//! holds the vocabulary used by estimation, control, and hardware adapters.

pub mod connectivity_policy;
pub mod point_capture;
pub mod timing;

/// Three-component vector; units and reference frame are defined by the containing field.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vector3 {
    /// X component.
    pub x: f64,
    /// Y component.
    pub y: f64,
    /// Z component.
    pub z: f64,
}
impl Vector3 {
    /// The additive identity `(0, 0, 0)`.
    pub const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    /// Return whether every component is finite.
    pub fn finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
    /// Multiply each component by `scale`, without validating finite values.
    pub fn scaled(self, scale: f64) -> Self {
        Self {
            x: self.x * scale,
            y: self.y * scale,
            z: self.z * scale,
        }
    }
    /// Add another vector component by component in the same reference frame.
    pub fn plus(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
    /// Compute the right-handed cross product `self × other`.
    pub fn cross(self, other: Self) -> Self {
        Self {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }
}
/// Quaternion stored as vector components `(x, y, z)` and scalar `w`.
///
/// The default is identity rotation. Rotation operations require unit length;
/// use [`Self::normalized`] when accepting an external orientation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quaternion {
    /// X component.
    pub x: f64,
    /// Y component.
    pub y: f64,
    /// Z component.
    pub z: f64,
    /// Scalar component.
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
    /// Return a unit quaternion, or `None` for a nonfinite norm or norm below `1e-12`.
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
    /// Negate the vector part; for a unit quaternion this is the inverse rotation.
    pub fn conjugate(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
            w: self.w,
        }
    }
    /// Compute the Hamilton product `self * other` without normalization.
    /// For unit quaternions, the composed rotation applies `other` first.
    pub fn compose(self, other: Self) -> Self {
        Self {
            x: self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            y: self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            z: self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
            w: self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
        }
    }
    /// Rotate a vector; callers must normalize the quaternion first.
    ///
    /// # Examples
    ///
    /// Rotate the forward axis a quarter turn about +Z:
    ///
    /// ```
    /// use terra_types::{Quaternion, Vector3};
    ///
    /// let rotation = Quaternion::from_rotation_vector(Vector3 {
    ///     z: std::f64::consts::FRAC_PI_2,
    ///     ..Vector3::ZERO
    /// }).normalized().unwrap();
    /// let left = rotation.rotate(Vector3 { x: 1.0, ..Vector3::ZERO });
    /// assert!(left.x.abs() < 1e-12);
    /// assert!((left.y - 1.0).abs() < 1e-12);
    /// ```
    pub fn rotate(self, vector: Vector3) -> Vector3 {
        let axis = Vector3 {
            x: self.x,
            y: self.y,
            z: self.z,
        };
        let twice_cross = axis.cross(vector).scaled(2.0);
        vector
            .plus(twice_cross.scaled(self.w))
            .plus(axis.cross(twice_cross))
    }
    /// Construct an axis-angle rotation: vector direction is the axis and length
    /// is the angle in radians. Inputs must be finite; zero yields identity.
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
/// Calibrated inertial measurement at a monotonic timestamp.
#[derive(Clone, Copy, Debug)]
pub struct ImuSample {
    /// Seconds on the shared monotonic clock.
    pub timestamp: f64,
    /// Gravity removed and calibrated, expressed in rover body axes, m/s².
    pub acceleration: Vector3,
    /// Calibrated rover body angular velocity, rad/s.
    pub angular_velocity: Vector3,
}
/// Metric visual-inertial odometry anchor in a caller-defined world frame.
#[derive(Clone, Copy, Debug)]
pub struct VioSample {
    /// Seconds on the shared monotonic clock.
    pub timestamp: f64,
    /// Rover origin position in world metres.
    pub position: Vector3,
    /// Rover body to world rotation. World must be metric and +Z up.
    pub orientation: Quaternion,
    /// Rover origin velocity in world axes, m/s.
    pub velocity: Vector3,
    /// Whether visual tracking is currently valid.
    pub tracked: bool,
}
/// Requested body-forward speed and yaw rate, with a freshness timestamp.
#[derive(Clone, Copy, Debug, Default)]
pub struct VelocityTarget {
    /// Seconds on the shared monotonic clock.
    pub timestamp: f64,
    /// Body-forward speed in metres per second.
    pub forward: f64,
    /// Body yaw rate in radians per second.
    pub yaw_rate: f64,
}
/// Availability and validity of evidence used for a velocity estimate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Health {
    /// Both sensor streams provide usable evidence.
    Ready,
    /// IMU evidence required for prediction is absent.
    MissingImu,
    /// No VIO anchor has been accepted.
    MissingVio,
    /// The latest VIO sample reports lost tracking.
    TrackingLost,
    /// The newest IMU sample has reached its timeout.
    StaleImu,
    /// The VIO anchor has reached its timeout.
    StaleVio,
    /// Time is invalid or inconsistent with the retained evidence.
    InvalidTime,
}
/// Body-frame velocity estimate and the health of its supporting evidence.
#[derive(Clone, Copy, Debug)]
pub struct VelocityEstimate {
    /// Seconds on the shared monotonic clock.
    pub timestamp: f64,
    /// Body-forward speed in metres per second.
    pub forward: f64,
    /// Body yaw rate in radians per second.
    pub yaw_rate: f64,
    /// Whether the estimate is usable and, if not, why.
    pub health: Health,
}
/// Reason the controller suppressed wheel effort.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StopReason {
    /// No safety stop was requested.
    None,
    /// Required sensor evidence is missing.
    SensorNotReady,
    /// The latest VIO sample reports lost tracking.
    TrackingLost,
    /// Required sensor evidence has expired.
    StaleSensors,
    /// The command has expired.
    StaleTarget,
    /// Time is invalid or inconsistent with the retained evidence.
    InvalidTime,
}
/// Normalized wheel efforts and the velocities used to compute them.
#[derive(Clone, Copy, Debug)]
pub struct MotorOutput {
    /// Signed effort [-1, 1]. Positive is forward.
    /// `terra-motors` maps magnitude to PWM duty and sign to direction.
    /// Zero is coast, not a mechanical brake.
    pub left: f64,
    /// Right-wheel effort in `[-1, 1]`; positive is forward and zero coasts.
    pub right: f64,
    /// Estimated body-forward speed in metres per second.
    pub estimated_forward: f64,
    /// Estimated yaw rate in radians per second.
    pub estimated_yaw_rate: f64,
    /// Effective requested body-forward speed in metres per second.
    pub target_forward: f64,
    /// Effective requested yaw rate in radians per second.
    pub target_yaw_rate: f64,
    /// Safety stop status associated with these outputs.
    pub stop_reason: StopReason,
}
impl MotorOutput {
    /// Construct zero wheel effort and zero velocity diagnostics with the supplied reason.
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
/// Rejected sensor input or configuration; no automatic correction is implied.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InputError {
    /// A numeric input is nonfinite or violates a nonnegative timestamp requirement.
    NonFinite,
    /// An orientation cannot be normalized to a usable rotation.
    InvalidQuaternion,
    /// A timestamp does not strictly follow the previous sample in its stream.
    OutOfOrder,
    /// A finite value exceeds a supported physical or numeric limit.
    OutOfRange,
    /// Configuration values violate the consuming component's requirements.
    InvalidConfiguration,
}
impl std::fmt::Display for InputError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for InputError {}
