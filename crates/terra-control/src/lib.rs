//! Feedback PI velocity control with feedforward, coupled effort limits and anti-windup.
use terra_types::*;
#[derive(Clone, Copy, Debug)]
pub struct ControllerConfig {
    pub linear_kp: f64,
    pub linear_ki: f64,
    pub yaw_kp: f64,
    pub yaw_ki: f64,
    pub linear_feedforward: f64,
    pub yaw_feedforward: f64,
    pub max_forward: f64,
    pub max_yaw_rate: f64,
    pub max_effort: f64,
    pub target_timeout: f64,
    pub max_step: f64,
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
pub struct VelocityController {
    config: ControllerConfig,
    target: Option<VelocityTarget>,
    last_step: Option<f64>,
    linear_integral: f64,
    yaw_integral: f64,
}
impl VelocityController {
    pub fn new(config: ControllerConfig) -> Result<Self, InputError> {
        config.validate()?;
        Ok(Self {
            config,
            target: None,
            last_step: None,
            linear_integral: 0.0,
            yaw_integral: 0.0,
        })
    }
    pub fn reset(&mut self) {
        self.target = None;
        self.last_step = None;
        self.clear_integrals();
    }
    fn clear_integrals(&mut self) {
        self.linear_integral = 0.0;
        self.yaw_integral = 0.0;
    }
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
        let error_v = target.forward - estimate.forward;
        let error_w = target.yaw_rate - estimate.yaw_rate;
        let linear = self.config.linear_feedforward * target.forward
            + self.config.linear_kp * error_v
            + self.linear_integral;
        let yaw = self.config.yaw_feedforward * target.yaw_rate
            + self.config.yaw_kp * error_w
            + self.yaw_integral;
        let scale =
            ((linear - yaw).abs().max((linear + yaw).abs()) / self.config.max_effort).max(1.0);
        let (mut left, mut right) = ((linear - yaw) / scale, (linear + yaw) / scale);
        let achieved_linear = (left + right) * 0.5;
        let achieved_yaw = (right - left) * 0.5;
        self.linear_integral = (self.linear_integral
            + (self.config.linear_ki * error_v
                + self.config.anti_windup * (achieved_linear - linear))
                * dt)
            .clamp(-self.config.max_effort, self.config.max_effort);
        self.yaw_integral = (self.yaw_integral
            + (self.config.yaw_ki * error_w + self.config.anti_windup * (achieved_yaw - yaw)) * dt)
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
    fn estimate(t: f64, v: f64, w: f64) -> VelocityEstimate {
        VelocityEstimate {
            timestamp: t,
            forward: v,
            yaw_rate: w,
            health: Health::Ready,
        }
    }
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
