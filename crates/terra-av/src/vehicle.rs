//! Shared foundation: the vehicle model (roadmap 2). Every stage that
//! reasons about motion takes a `&dyn VehicleModel`.
//!
//! The model works in twists only. Turning a twist into wheel efforts is the
//! platform's velocity loop (`Motors/VelocityController.swift` on
//! TerraPhone), so wheel speeds never leave this module.
use crate::geometry::Pose2;

/// Body velocity: forward `v` (m/s), yaw rate `w` (rad/s, + = left).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Twist {
    pub v: f64,
    pub w: f64,
}

pub trait VehicleModel: Send {
    /// Scale `twist` down so every wheel stays within `max_wheel_speed`,
    /// keeping the ratio `w / v` (the turn radius). A reachable twist comes
    /// back unchanged, so `clamp(t) == t` is the feasibility check.
    fn clamp(&self, twist: Twist) -> Twist;
    /// Motion model: where the rover ends up if it starts at `pose` (world
    /// frame) and holds `twist` (body frame) for `dt` seconds.
    ///
    /// The twist is treated as constant over `dt`, so the result is exact:
    /// a circular arc of radius `v / w`, or a straight line when `w == 0`.
    /// `yaw` in the result is not wrapped to (−π, π].
    ///
    /// Pure kinematics: no limits, no slip, no acceleration. Pass the twist
    /// through [`clamp`](Self::clamp) first if it may be unreachable. DWA
    /// calls this to roll out candidate twists over its horizon.
    fn step(&self, pose: Pose2, twist: Twist, dt: f64) -> Pose2;
    /// What this vehicle can physically do. Read by the Rust planners only:
    /// [`clamp`](Self::clamp) enforces `max_wheel_speed`, and DWA uses the
    /// accelerations to bound which twists are reachable in one tick. The
    /// Swift velocity loop has its own settings and does not read these.
    fn limits(&self) -> Limits;
}

/// Physical limits of a vehicle, built from the `[vehicle]` section of
/// `terra-av.toml`. All values are positive magnitudes.
pub struct Limits {
    /// Top ground speed of any one wheel side (m/s). Caps both forward speed
    /// and turn rate, since turning speeds up the outer side.
    pub max_wheel_speed: f64,
    /// Largest change in forward speed `v` (m/s²), speeding up or braking.
    pub max_accel: f64,
    /// Largest change in yaw rate `w` (rad/s²).
    pub max_yaw_accel: f64,
}

/// Two wheel sides `track` metres apart (Terra, terra-mini).
///
/// `track` is the geometric width. A skid-steer base scrubs when it turns
/// and behaves wider (the Swift loop's `skidFactor`), so turn-rate limits
/// from this model are optimistic until a skid correction is added.
pub struct DiffDrive {
    pub track: f64,
    pub limits: Limits,
}

impl VehicleModel for DiffDrive {
    fn clamp(&self, twist: Twist) -> Twist {
        // Inverse kinematics: wheel speeds are v ± w·track/2, so the
        // fastest wheel runs at |v| + |w|·track/2.
        let fastest = twist.v.abs() + twist.w.abs() * self.track / 2.0;
        let max = self.limits.max_wheel_speed;
        if fastest <= max {
            return twist;
        }
        let scale = max / fastest;
        Twist { v: twist.v * scale, w: twist.w * scale }
    }
    fn step(&self, pose: Pose2, twist: Twist, dt: f64) -> Pose2 {
        todo!()
    }
    fn limits(&self) -> Limits {
        todo!()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn terra() -> DiffDrive {
        DiffDrive {
            track: 0.5,
            limits: Limits { max_wheel_speed: 0.8, max_accel: 1.0, max_yaw_accel: 2.0 },
        }
    }

    #[test]
    fn reachable_twist_is_unchanged() {
        let t = Twist { v: 0.5, w: 0.4 }; // fastest wheel 0.6 m/s
        assert_eq!(terra().clamp(t), t);
    }

    #[test]
    fn too_fast_twist_is_scaled_keeping_turn_radius() {
        let t = Twist { v: 1.0, w: 1.2 }; // fastest wheel 1.3 m/s
        let c = terra().clamp(t);
        let fastest = c.v.abs() + c.w.abs() * 0.25;
        assert!((fastest - 0.8).abs() < 1e-12);
        assert!((c.w / c.v - t.w / t.v).abs() < 1e-12);
    }

    #[test]
    fn spin_in_place_is_limited_by_wheel_speed() {
        let c = terra().clamp(Twist { v: 0.0, w: -10.0 });
        assert_eq!(c.v, 0.0);
        assert!((c.w - -3.2).abs() < 1e-12); // 0.8 / (0.5 / 2)
    }
}
