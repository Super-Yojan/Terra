//! Closed-loop velocity control: desired motion to normalized wheel effort.
//!
//! [`VelocityController::set_target`] accepts a timestamped velocity target.
//! [`VelocityController::step`] checks sensor health and freshness before applying
//! feedforward and PI feedback. Wheel efforts share one saturation scale so their
//! ratio is preserved; anti-windup feeds the achievable effort back into the integrals.
//!
//! Targets and estimates share a nonnegative monotonic clock in seconds. Forward
//! velocity is metres/second and yaw rate is radians/second, with positive yaw
//! producing greater right-wheel effort. Hardware enable, PWM conversion, and
//! obstacle checks belong to other layers.
//!
//! ```
//! use terra_control::{ControllerConfig, VelocityController};
//! use terra_types::{Health, StopReason, VelocityEstimate, VelocityTarget};
//! let mut controller = VelocityController::new(ControllerConfig::default()).unwrap();
//! controller.set_target(VelocityTarget {
//!     timestamp: 0.0, forward: 0.5, yaw_rate: 0.0,
//! }).unwrap();
//! let output = controller.step(VelocityEstimate {
//!     timestamp: 0.0, forward: 0.0, yaw_rate: 0.0, health: Health::Ready,
//! });
//! assert_eq!(output.stop_reason, StopReason::None);
//! assert!(output.left > 0.0 && output.left == output.right);
//! ```

use terra_types::*;
mod turning;
pub use turning::{TurningConfig, TurningLimiter};
/// PI gains and safety limits for differential wheel control.
///
/// Defaults target a small rover. All numbers must be finite; call [`Self::validate`]
/// after changing settings. Gains convert velocity errors into normalized effort.
#[derive(Clone, Copy, Debug)]
pub struct ControllerConfig {
    /// Forward proportional gain in normalized effort per metre/second.
    pub linear_kp: f64,
    /// Forward integral gain in normalized effort per metre of accumulated error.
    pub linear_ki: f64,
    /// Yaw proportional gain in normalized effort per radian/second.
    pub yaw_kp: f64,
    /// Yaw integral gain in normalized effort per radian of accumulated error.
    pub yaw_ki: f64,
    /// Forward target feedforward gain in effort per metre/second.
    pub linear_feedforward: f64,
    /// Yaw target feedforward gain in effort per radian/second.
    pub yaw_feedforward: f64,
    /// Symmetric target speed limit in metres/second; must be in `(0, 20]`.
    pub max_forward: f64,
    /// Symmetric target yaw-rate limit in radians/second; must be in `(0, 20]`.
    pub max_yaw_rate: f64,
    /// Maximum absolute wheel effort, in `(0, 1]`.
    pub max_effort: f64,
    /// Target age in seconds at which the controller stops; must be positive.
    pub target_timeout: f64,
    /// Largest accepted elapsed step in seconds, in `(0, 1]`.
    pub max_step: f64,
    /// Back-calculation rate in inverse seconds, applied to saturation error.
    pub anti_windup: f64,
}
impl Default for ControllerConfig {
    fn default() -> Self {
        Self {
            linear_kp: 0.8,
            linear_ki: 0.6,
            yaw_kp: 0.4,
            yaw_ki: 0.3,
            linear_feedforward: 1.0 / 3.0,
            yaw_feedforward: 0.2,
            max_forward: 2.0,
            max_yaw_rate: 2.0,
            max_effort: 1.0,
            target_timeout: 0.5,
            max_step: 0.1,
            anti_windup: 4.0,
        }
    }
}
impl ControllerConfig {
    /// Check finite gains in `[0, 100]` and positive limits with their stated bounds.
    ///
    /// Returns [`InputError::InvalidConfiguration`] if any setting violates those requirements.
    pub fn validate(&self) -> Result<(), InputError> {
        let gains = [
            self.linear_kp,
            self.linear_ki,
            self.yaw_kp,
            self.yaw_ki,
            self.linear_feedforward,
            self.yaw_feedforward,
            self.anti_windup,
        ];
        let limits = [
            self.max_forward,
            self.max_yaw_rate,
            self.max_effort,
            self.target_timeout,
            self.max_step,
        ];
        if gains
            .iter()
            .any(|value| !value.is_finite() || *value < 0.0 || *value > 100.0)
            || limits
                .iter()
                .any(|value| !value.is_finite() || *value <= 0.0)
            || self.max_effort > 1.0
            || self.max_forward > 20.0
            || self.max_yaw_rate > 20.0
            || self.max_step > 1.0
        {
            return Err(InputError::InvalidConfiguration);
        }
        Ok(())
    }
}
/// Stateful feedforward/PI controller with freshness and sensor-health holds.
///
/// Times are nonnegative monotonic seconds in the same clock domain as the target.
/// A new controller has no target and produces a stale-target hold until one is set.
pub struct VelocityController {
    config: ControllerConfig,
    turning: Option<TurningConfig>,
    target: Option<VelocityTarget>,
    last_step: Option<f64>,
    linear_integral: f64,
    yaw_integral: f64,
}
impl VelocityController {
    /// Construct an idle controller with zero integrals.
    ///
    /// Returns [`InputError::InvalidConfiguration`] when `config` fails validation.
    pub fn new(config: ControllerConfig) -> Result<Self, InputError> {
        config.validate()?;
        Ok(Self {
            config,
            turning: None,
            target: None,
            last_step: None,
            linear_integral: 0.0,
            yaw_integral: 0.0,
        })
    }
    /// Discard the target, prior step time, and both accumulated feedback terms.
    pub fn reset(&mut self) {
        self.target = None;
        self.last_step = None;
        self.clear_integrals();
    }
    /// Apply a managed output constraint; local manual callers leave it disabled.
    pub fn set_turning_constraint(&mut self, config: Option<TurningConfig>) {
        if self.turning != config {
            self.clear_integrals();
        }
        self.turning = config;
    }
    fn clear_integrals(&mut self) {
        self.linear_integral = 0.0;
        self.yaw_integral = 0.0;
    }
    /// Latch a target, clamping speeds to the configured symmetric limits.
    ///
    /// Forward speed is metres/second and yaw rate is radians/second. A non-finite
    /// value or negative timestamp clears the target and integrals and returns
    /// [`InputError::NonFinite`]. An older timestamp returns [`InputError::OutOfOrder`]
    /// without replacing the target; equal timestamps are allowed.
    pub fn set_target(&mut self, mut target: VelocityTarget) -> Result<(), InputError> {
        if !target.timestamp.is_finite()
            || target.timestamp < 0.0
            || !target.forward.is_finite()
            || !target.yaw_rate.is_finite()
        {
            self.target = None;
            self.clear_integrals();
            return Err(InputError::NonFinite);
        }
        if self
            .target
            .is_some_and(|last| target.timestamp < last.timestamp)
        {
            return Err(InputError::OutOfOrder);
        }
        target.forward = target
            .forward
            .clamp(-self.config.max_forward, self.config.max_forward);
        target.yaw_rate = target
            .yaw_rate
            .clamp(-self.config.max_yaw_rate, self.config.max_yaw_rate);
        self.target = Some(target);
        Ok(())
    }
    /// Produce wheel effort only after time, sensor health, and target checks pass.
    /// A safety hold clears accumulated feedback so it cannot resume as a surge.
    /// Use estimated forward metres/second and yaw radians/second to compute effort.
    ///
    /// The first accepted timing calculation uses 0.01 seconds. Missing or expired
    /// targets, future targets, non-increasing or overlong steps, invalid numbers,
    /// and unhealthy sensors return zero effort with the corresponding stop reason.
    /// Safety holds clear integrals. Successful efforts lie within `max_effort`.
    /// A zero target with measured speed and yaw-rate magnitude both below 0.015
    /// (in their respective units) clears integrals and produces exact zero effort.
    pub fn step(&mut self, estimate: VelocityEstimate) -> MotorOutput {
        let now = estimate.timestamp;
        let dt = self.last_step.map_or(0.01, |last| now - last);
        let stop_reason = if !now.is_finite()
            || now < 0.0
            || !estimate.forward.is_finite()
            || !estimate.yaw_rate.is_finite()
        {
            Some(StopReason::InvalidTime)
        } else if estimate.health != Health::Ready {
            Some(match estimate.health {
                Health::TrackingLost => StopReason::TrackingLost,
                Health::StaleImu | Health::StaleVio => StopReason::StaleSensors,
                Health::InvalidTime => StopReason::InvalidTime,
                _ => StopReason::SensorNotReady,
            })
        } else if self
            .target
            .is_none_or(|target| now - target.timestamp >= self.config.target_timeout)
        {
            Some(StopReason::StaleTarget)
        } else if self.target.is_some_and(|target| target.timestamp > now)
            || dt <= 0.0
            || dt > self.config.max_step
        {
            Some(StopReason::InvalidTime)
        } else {
            None
        };
        if now.is_finite() && now >= 0.0 && self.last_step.is_none_or(|last| now > last) {
            self.last_step = Some(now);
        }
        if let Some(reason) = stop_reason {
            self.clear_integrals();
            return MotorOutput::stopped(reason);
        }
        let target = self.target.unwrap();
        let linear_error = target.forward - estimate.forward;
        let yaw_error = target.yaw_rate - estimate.yaw_rate;
        let linear = self.config.linear_feedforward * target.forward
            + self.config.linear_kp * linear_error
            + self.linear_integral;
        let yaw = self.config.yaw_feedforward * target.yaw_rate
            + self.config.yaw_kp * yaw_error
            + self.yaw_integral;
        // Scale both wheels together to preserve the requested turn/forward ratio.
        let scale =
            ((linear - yaw).abs().max((linear + yaw).abs()) / self.config.max_effort).max(1.0);
        let (mut left, mut right) = ((linear - yaw) / scale, (linear + yaw) / scale);
        if let Some(config) = self.turning {
            (left, right) = config.constrain_wheels(left, right);
        }
        let achieved_linear = (left + right) * 0.5;
        let achieved_yaw = (right - left) * 0.5;
        // Back-calculation prevents the integrals from growing against saturation.
        self.linear_integral = (self.linear_integral
            + (self.config.linear_ki * linear_error
                + self.config.anti_windup * (achieved_linear - linear))
                * dt)
            .clamp(-self.config.max_effort, self.config.max_effort);
        self.yaw_integral = (self.yaw_integral
            + (self.config.yaw_ki * yaw_error + self.config.anti_windup * (achieved_yaw - yaw))
                * dt)
            .clamp(-self.config.max_effort, self.config.max_effort);
        if target.forward == 0.0
            && target.yaw_rate == 0.0
            && estimate.forward.abs() < 0.015
            && estimate.yaw_rate.abs() < 0.015
        {
            self.clear_integrals();
            left = 0.0;
            right = 0.0;
        }
        MotorOutput {
            left,
            right,
            estimated_forward: estimate.forward,
            estimated_yaw_rate: estimate.yaw_rate,
            target_forward: target.forward,
            target_yaw_rate: target.yaw_rate,
            stop_reason: StopReason::None,
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    /// Build a ready sensor fixture at time `t` seconds with forward speed `v`
    /// in metres/second and yaw rate `w` in radians/second.
    fn estimate(t: f64, v: f64, w: f64) -> VelocityEstimate {
        VelocityEstimate {
            timestamp: t,
            forward: v,
            yaw_rate: w,
            health: Health::Ready,
        }
    }
    /// Checks that a forward-left target produces unequal positive wheel efforts,
    /// then times out at the exact deadline and stops when tracking is lost.
    #[test]
    fn produces_differential_effort_and_neutral_on_timeout_or_tracking_loss() {
        let mut c = VelocityController::new(ControllerConfig::default()).unwrap();
        c.set_target(VelocityTarget {
            timestamp: 0.0,
            forward: 1.0,
            yaw_rate: 0.5,
        })
        .unwrap();
        let output = c.step(estimate(0.0, 0.0, 0.0));
        assert!(output.left > 0.0 && output.right > output.left);
        assert!(output.right <= 1.0);
        assert_eq!(
            c.step(estimate(0.5, 0.0, 0.0)).stop_reason,
            StopReason::StaleTarget
        );
        c.set_target(VelocityTarget {
            timestamp: 0.51,
            forward: 1.0,
            yaw_rate: 0.0,
        })
        .unwrap();
        let mut state = estimate(0.51, 0.0, 0.0);
        state.health = Health::TrackingLost;
        assert_eq!(c.step(state).left, 0.0);
    }
    /// Simulates a loaded first-order motor plant at 100 Hz, reducing its speed target
    /// after sustained saturation and checking final forward and yaw convergence.
    #[test]
    fn closes_the_loop_on_a_motor_plant_with_load_and_recovers_from_saturation() {
        let mut c = VelocityController::new(ControllerConfig::default()).unwrap();
        let (mut v, mut w) = (0.0, 0.0);
        for step in 0..1000 {
            let t = step as f64 * 0.01;
            let desired = if step < 300 { 2.0 } else { 0.8 };
            c.set_target(VelocityTarget {
                timestamp: t,
                forward: desired,
                yaw_rate: 0.3,
            })
            .unwrap();
            let out = c.step(estimate(t, v, w));
            assert!(out.left.abs() <= 1.0 && out.right.abs() <= 1.0);
            v += (3.0 * (out.left + out.right) * 0.5 - v - 0.15) * 0.01;
            w += (5.0 * (out.right - out.left) * 0.5 - w) * 0.01;
        }
        assert!((v - 0.8).abs() < 0.04, "v={v}");
        assert!((w - 0.3).abs() < 0.04, "w={w}");
    }
}
