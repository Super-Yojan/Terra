//! Thread-safe UniFFI boundary. Robotics packages have no UniFFI dependencies.
use std::sync::{Arc, Mutex};
use terra_control::{ControllerConfig, VelocityController};
use terra_state::{EstimatorConfig, VelocityEstimator};
use terra_types::*;
uniffi::setup_scaffolding!();
mod mapping;
mod transport;
pub use mapping::*;
pub use transport::*;

#[derive(Clone, Debug, uniffi::Record)]
pub struct ControlSettings {
    pub linear_kp: f64,
    pub linear_ki: f64,
    pub yaw_kp: f64,
    pub yaw_ki: f64,
    pub linear_feedforward: f64,
    pub yaw_feedforward: f64,
    pub max_forward: f64,
    pub max_yaw_rate: f64,
    pub max_effort: f64,
    pub imu_timeout: f64,
    pub vio_timeout: f64,
    pub target_timeout: f64,
    pub max_step: f64,
    pub anti_windup: f64,
}
impl ControlSettings {
    fn controller(&self) -> ControllerConfig {
        ControllerConfig {
            linear_kp: self.linear_kp,
            linear_ki: self.linear_ki,
            yaw_kp: self.yaw_kp,
            yaw_ki: self.yaw_ki,
            linear_feedforward: self.linear_feedforward,
            yaw_feedforward: self.yaw_feedforward,
            max_forward: self.max_forward,
            max_yaw_rate: self.max_yaw_rate,
            max_effort: self.max_effort,
            target_timeout: self.target_timeout,
            max_step: self.max_step,
            anti_windup: self.anti_windup,
        }
    }
    fn estimator(&self) -> EstimatorConfig {
        EstimatorConfig {
            imu_timeout: self.imu_timeout,
            vio_timeout: self.vio_timeout,
        }
    }
}
#[uniffi::export]
pub fn default_control_settings() -> ControlSettings {
    let c = ControllerConfig::default();
    let e = EstimatorConfig::default();
    ControlSettings {
        linear_kp: c.linear_kp,
        linear_ki: c.linear_ki,
        yaw_kp: c.yaw_kp,
        yaw_ki: c.yaw_ki,
        linear_feedforward: c.linear_feedforward,
        yaw_feedforward: c.yaw_feedforward,
        max_forward: c.max_forward,
        max_yaw_rate: c.max_yaw_rate,
        max_effort: c.max_effort,
        imu_timeout: e.imu_timeout,
        vio_timeout: e.vio_timeout,
        target_timeout: c.target_timeout,
        max_step: c.max_step,
        anti_windup: c.anti_windup,
    }
}
#[derive(Clone, Debug, uniffi::Record)]
pub struct ImuReading {
    pub timestamp: f64,
    pub acceleration_forward: f64,
    pub acceleration_left: f64,
    pub acceleration_up: f64,
    pub gyro_roll: f64,
    pub gyro_pitch: f64,
    pub gyro_yaw: f64,
}
#[derive(Clone, Debug, uniffi::Record)]
pub struct VioReading {
    pub timestamp: f64,
    pub position_x: f64,
    pub position_y: f64,
    pub position_z: f64,
    pub quaternion_x: f64,
    pub quaternion_y: f64,
    pub quaternion_z: f64,
    pub quaternion_w: f64,
    pub velocity_x: f64,
    pub velocity_y: f64,
    pub velocity_z: f64,
    pub tracked: bool,
}
#[derive(Clone, Debug, uniffi::Record)]
pub struct TwistSetpoint {
    pub timestamp: f64,
    pub forward: f64,
    pub yaw_rate: f64,
}
#[derive(Clone, Copy, Debug, PartialEq, uniffi::Enum)]
pub enum SafetyState {
    Active,
    SensorNotReady,
    TrackingLost,
    StaleSensors,
    StaleTarget,
    InvalidTime,
}
#[derive(Clone, Debug, uniffi::Record)]
pub struct ControlOutput {
    pub left_effort: f64,
    pub right_effort: f64,
    pub estimated_forward: f64,
    pub estimated_yaw_rate: f64,
    pub target_forward: f64,
    pub target_yaw_rate: f64,
    pub safety: SafetyState,
}
impl From<MotorOutput> for ControlOutput {
    fn from(value: MotorOutput) -> Self {
        Self {
            left_effort: value.left,
            right_effort: value.right,
            estimated_forward: value.estimated_forward,
            estimated_yaw_rate: value.estimated_yaw_rate,
            target_forward: value.target_forward,
            target_yaw_rate: value.target_yaw_rate,
            safety: match value.stop_reason {
                StopReason::None => SafetyState::Active,
                StopReason::SensorNotReady => SafetyState::SensorNotReady,
                StopReason::TrackingLost => SafetyState::TrackingLost,
                StopReason::StaleSensors => SafetyState::StaleSensors,
                StopReason::StaleTarget => SafetyState::StaleTarget,
                StopReason::InvalidTime => SafetyState::InvalidTime,
            },
        }
    }
}
#[derive(Debug, thiserror::Error, uniffi::Error)]
pub enum ControllerError {
    #[error("{message}")]
    InvalidInput { message: String },
    #[error("controller lock was poisoned")]
    Internal,
}
impl From<InputError> for ControllerError {
    fn from(error: InputError) -> Self {
        Self::InvalidInput {
            message: error.to_string(),
        }
    }
}
struct Brain {
    estimator: VelocityEstimator,
    controller: VelocityController,
}
#[derive(uniffi::Object)]
pub struct MobileController {
    brain: Mutex<Brain>,
}
#[uniffi::export]
impl MobileController {
    #[uniffi::constructor]
    pub fn new(settings: ControlSettings) -> Result<Arc<Self>, ControllerError> {
        Ok(Arc::new(Self {
            brain: Mutex::new(Brain {
                estimator: VelocityEstimator::new(settings.estimator())?,
                controller: VelocityController::new(settings.controller())?,
            }),
        }))
    }
    pub fn push_imu(&self, sample: ImuReading) -> Result<(), ControllerError> {
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .estimator
            .push_imu(ImuSample {
                timestamp: sample.timestamp,
                acceleration: Vector3 {
                    x: sample.acceleration_forward,
                    y: sample.acceleration_left,
                    z: sample.acceleration_up,
                },
                angular_velocity: Vector3 {
                    x: sample.gyro_roll,
                    y: sample.gyro_pitch,
                    z: sample.gyro_yaw,
                },
            })?;
        Ok(())
    }
    pub fn push_vio(&self, sample: VioReading) -> Result<(), ControllerError> {
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .estimator
            .push_vio(VioSample {
                timestamp: sample.timestamp,
                position: Vector3 {
                    x: sample.position_x,
                    y: sample.position_y,
                    z: sample.position_z,
                },
                orientation: Quaternion {
                    x: sample.quaternion_x,
                    y: sample.quaternion_y,
                    z: sample.quaternion_z,
                    w: sample.quaternion_w,
                },
                velocity: Vector3 {
                    x: sample.velocity_x,
                    y: sample.velocity_y,
                    z: sample.velocity_z,
                },
                tracked: sample.tracked,
            })?;
        Ok(())
    }
    pub fn set_target(&self, target: TwistSetpoint) -> Result<(), ControllerError> {
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .controller
            .set_target(VelocityTarget {
                timestamp: target.timestamp,
                forward: target.forward,
                yaw_rate: target.yaw_rate,
            })?;
        Ok(())
    }
    pub fn step(&self, timestamp: f64) -> Result<ControlOutput, ControllerError> {
        let mut brain = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        let estimate = brain.estimator.estimate(timestamp);
        Ok(brain.controller.step(estimate).into())
    }
    pub fn reset(&self) -> Result<(), ControllerError> {
        let mut brain = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        brain.estimator.reset();
        brain.controller.reset();
        Ok(())
    }
}
#[derive(Clone, Debug, uniffi::Record)]
pub struct BenchmarkReport {
    pub final_forward: f64,
    pub final_yaw_rate: f64,
    pub forward_error: f64,
    pub yaw_error: f64,
    pub passed: bool,
}
/// Deterministic 10 s motor plant experiment using the same facade as iOS.
#[uniffi::export]
pub fn run_velocity_benchmark() -> Result<BenchmarkReport, ControllerError> {
    let brain = MobileController::new(default_control_settings())?;
    let (mut v, mut w, mut yaw, mut ax) = (0.0, 0.0, 0.0, 0.0);
    for tick in 0..1000 {
        let t = tick as f64 * 0.01;
        brain.push_imu(ImuReading {
            timestamp: t,
            acceleration_forward: ax,
            acceleration_left: v * w,
            acceleration_up: 0.0,
            gyro_roll: 0.0,
            gyro_pitch: 0.0,
            gyro_yaw: w,
        })?;
        if tick % 5 == 0 {
            brain.push_vio(VioReading {
                timestamp: t,
                position_x: 0.0,
                position_y: 0.0,
                position_z: 0.0,
                quaternion_x: 0.0,
                quaternion_y: 0.0,
                quaternion_z: (yaw * 0.5_f64).sin(),
                quaternion_w: (yaw * 0.5_f64).cos(),
                velocity_x: v * yaw.cos(),
                velocity_y: v * yaw.sin(),
                velocity_z: 0.0,
                tracked: true,
            })?;
        }
        brain.set_target(TwistSetpoint {
            timestamp: t,
            forward: if tick < 300 { 2.0 } else { 0.8 },
            yaw_rate: 0.3,
        })?;
        let output = brain.step(t)?;
        ax = 3.0 * (output.left_effort + output.right_effort) * 0.5 - v - 0.15;
        v += ax * 0.01;
        w += (5.0 * (output.right_effort - output.left_effort) * 0.5 - w) * 0.01;
        yaw += w * 0.01;
    }
    let forward_error = (v - 0.8).abs();
    let yaw_error = (w - 0.3).abs();
    Ok(BenchmarkReport {
        final_forward: v,
        final_yaw_rate: w,
        forward_error,
        yaw_error,
        passed: forward_error < 0.05 && yaw_error < 0.05,
    })
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn mobile_facade_runs_the_same_sensor_and_motor_loop() {
        let report = run_velocity_benchmark().unwrap();
        assert!(report.passed, "{report:?}");
    }
    #[test]
    fn validates_settings_before_crossing_the_ffi_boundary() {
        let mut settings = default_control_settings();
        settings.max_effort = 1.5;
        assert!(MobileController::new(settings).is_err());
    }
}
