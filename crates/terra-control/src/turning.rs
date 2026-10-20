//! Managed motion limits. Local Bluetooth manual driving does not use this policy.
use serde::{Deserialize, Serialize};
use terra_types::{InputError, VelocityTarget};
/// Units: rad/s, rad/s², rad/s² and metres, respectively.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct TurningConfig {
    pub max_yaw_rate: f64,
    pub angular_acceleration: f64,
    pub angular_deceleration: f64,
    pub min_turn_radius: f64,
    pub allow_in_place: bool,
}
impl Default for TurningConfig {
    fn default() -> Self {
        Self {
            max_yaw_rate: 0.5,
            angular_acceleration: 0.5,
            angular_deceleration: 0.8,
            min_turn_radius: 0.5,
            allow_in_place: false,
        }
    }
}
impl TurningConfig {
    pub fn validate(&self) -> Result<(), InputError> {
        if [
            (self.max_yaw_rate, 5.),
            (self.min_turn_radius, 5.),
            (self.angular_acceleration, 10.),
            (self.angular_deceleration, 10.),
        ]
        .iter()
        .any(|(v, max)| !v.is_finite() || *v <= 0. || v > max)
        {
            return Err(InputError::InvalidConfiguration);
        }
        Ok(())
    }
    /// Never increase wheel magnitude or allow opposite directions by default.
    pub fn constrain_wheels(&self, left: f64, right: f64) -> (f64, f64) {
        if !left.is_finite() || !right.is_finite() {
            return (0., 0.);
        }
        if self.allow_in_place || left * right >= 0. {
            return (left, right);
        }
        let forward = (left + right) * 0.5;
        if forward > 0. {
            (left.max(0.), right.max(0.))
        } else if forward < 0. {
            (left.min(0.), right.min(0.))
        } else {
            (0., 0.)
        }
    }
}
/// Clone to preview a candidate, then commit only the selected command.
#[derive(Clone, Debug)]
pub struct TurningLimiter {
    config: TurningConfig,
    previous: Option<VelocityTarget>,
}
impl TurningLimiter {
    pub fn new(config: TurningConfig) -> Result<Self, InputError> {
        config.validate()?;
        Ok(Self {
            config,
            previous: None,
        })
    }
    pub fn reset(&mut self) {
        self.previous = None;
    }
    pub fn step(&mut self, mut target: VelocityTarget) -> Result<VelocityTarget, InputError> {
        let dt = self
            .previous
            .map_or(0.01, |p| target.timestamp - p.timestamp);
        if !target.timestamp.is_finite()
            || target.timestamp < 0.
            || !target.forward.is_finite()
            || !target.yaw_rate.is_finite()
            || dt <= 0.
            || dt > 0.100000001
        {
            self.reset();
            return Err(InputError::OutOfOrder);
        }
        let c = self.config;
        let cap = if c.allow_in_place {
            c.max_yaw_rate
        } else {
            c.max_yaw_rate.min(target.forward.abs() / c.min_turn_radius)
        };
        let desired = target.yaw_rate.clamp(-cap, cap);
        let previous = self.previous.map_or(0., |p| p.yaw_rate);
        let reversal = previous * desired < 0.;
        let goal = if reversal { 0. } else { desired };
        let rate = if reversal || goal.abs() < previous.abs() {
            c.angular_deceleration
        } else {
            c.angular_acceleration
        };
        target.yaw_rate =
            (previous + (goal - previous).clamp(-rate * dt, rate * dt)).clamp(-cap, cap);
        self.previous = Some(target);
        Ok(target)
    }
}
