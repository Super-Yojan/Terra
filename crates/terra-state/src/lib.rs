//! VIO-anchored velocity estimation with bounded IMU prediction.
//!
//! Feed [`VelocityEstimator::push_imu`] and [`VelocityEstimator::push_vio`] with
//! samples from one monotonic clock, then call [`VelocityEstimator::estimate`].
//! A delayed VIO anchor is advanced using retained IMU history. Missing, stale,
//! or untracked evidence yields a health state rather than a usable velocity.
//!
//! # Example
//!
//! ```
//! use terra_state::{EstimatorConfig, VelocityEstimator};
//! use terra_types::{Health, ImuSample, Quaternion, Vector3, VioSample};
//!
//! let mut estimator = VelocityEstimator::new(EstimatorConfig::default())?;
//! estimator.push_imu(ImuSample {
//!     timestamp: 1.0,
//!     acceleration: Vector3::ZERO,
//!     angular_velocity: Vector3::ZERO,
//! })?;
//! estimator.push_vio(VioSample {
//!     timestamp: 1.0,
//!     position: Vector3::ZERO,
//!     orientation: Quaternion::default(),
//!     velocity: Vector3 { x: 0.5, ..Vector3::ZERO },
//!     tracked: true,
//! })?;
//! let estimate = estimator.estimate(1.0);
//! assert_eq!(estimate.health, Health::Ready);
//! assert_eq!(estimate.forward, 0.5);
//! # Ok::<(), terra_types::InputError>(())
//! ```

use std::collections::VecDeque;
use terra_types::*;
/// Maximum ages of the sensor evidence, in seconds on a shared monotonic clock.
#[derive(Clone, Copy, Debug)]
pub struct EstimatorConfig {
    /// IMU age at which estimates become stale; defaults to 0.1 s.
    pub imu_timeout: f64,
    /// VIO age at which estimates become stale; defaults to 0.35 s.
    pub vio_timeout: f64,
}
impl Default for EstimatorConfig {
    fn default() -> Self {
        Self {
            imu_timeout: timing::IMU_FRESHNESS_SECONDS,
            vio_timeout: timing::VIO_FRESHNESS_SECONDS,
        }
    }
}
/// Retains at most 512 IMU samples and the latest VIO anchor for velocity prediction.
/// Samples must be ordered within each stream; VIO may arrive behind the IMU stream.
pub struct VelocityEstimator {
    config: EstimatorConfig,
    imu: VecDeque<ImuSample>,
    vio: Option<VioSample>,
}
impl VelocityEstimator {
    /// Create an estimator with empty sensor history.
    ///
    /// # Errors
    ///
    /// Returns [`InputError::InvalidConfiguration`] unless both timeouts are finite
    /// and positive and `vio_timeout` is at most 2 s.
    pub fn new(config: EstimatorConfig) -> Result<Self, InputError> {
        if !config.imu_timeout.is_finite()
            || !config.vio_timeout.is_finite()
            || config.imu_timeout <= 0.0
            || config.vio_timeout <= 0.0
            || config.vio_timeout > 2.0
        {
            return Err(InputError::InvalidConfiguration);
        }
        Ok(Self {
            config,
            imu: VecDeque::new(),
            vio: None,
        })
    }
    /// Discard both streams, requiring new sensor evidence before reporting readiness.
    pub fn reset(&mut self) {
        self.imu.clear();
        self.vio = None;
    }
    /// Append a calibrated body-frame IMU sample, evicting the oldest beyond 512 samples.
    ///
    /// # Errors
    ///
    /// Returns [`InputError::NonFinite`] for nonfinite components or a negative
    /// timestamp, [`InputError::OutOfRange`] for acceleration magnitude above
    /// 1,000 m/s² or angular speed above 100 rad/s, and [`InputError::OutOfOrder`]
    /// unless its timestamp strictly increases within the retained IMU stream.
    pub fn push_imu(&mut self, sample: ImuSample) -> Result<(), InputError> {
        if !sample.timestamp.is_finite()
            || sample.timestamp < 0.0
            || !sample.acceleration.finite()
            || !sample.angular_velocity.finite()
        {
            return Err(InputError::NonFinite);
        }
        if sample
            .acceleration
            .x
            .hypot(sample.acceleration.y)
            .hypot(sample.acceleration.z)
            > 1000.0
            || sample
                .angular_velocity
                .x
                .hypot(sample.angular_velocity.y)
                .hypot(sample.angular_velocity.z)
                > 100.0
        {
            return Err(InputError::OutOfRange);
        }
        if self
            .imu
            .back()
            .is_some_and(|last| sample.timestamp <= last.timestamp)
        {
            return Err(InputError::OutOfOrder);
        }
        self.imu.push_back(sample);
        if self.imu.len() > 512 {
            self.imu.pop_front();
        }
        Ok(())
    }
    /// Replace the VIO anchor after validating and normalizing its orientation.
    /// Tracking-loss samples are retained so [`Self::estimate`] can report their status.
    ///
    /// # Errors
    ///
    /// Returns [`InputError::NonFinite`] for nonfinite time, position, or velocity
    /// or negative time; [`InputError::OutOfRange`] above 100 m/s; and
    /// [`InputError::InvalidQuaternion`] for an unusable rotation. Timestamps must
    /// strictly increase within the VIO stream, or [`InputError::OutOfOrder`] is returned.
    pub fn push_vio(&mut self, mut sample: VioSample) -> Result<(), InputError> {
        if !sample.timestamp.is_finite()
            || sample.timestamp < 0.0
            || !sample.velocity.finite()
            || !sample.position.finite()
        {
            return Err(InputError::NonFinite);
        }
        if sample
            .velocity
            .x
            .hypot(sample.velocity.y)
            .hypot(sample.velocity.z)
            > 100.0
        {
            return Err(InputError::OutOfRange);
        }
        sample.orientation = sample
            .orientation
            .normalized()
            .ok_or(InputError::InvalidQuaternion)?;
        if self
            .vio
            .is_some_and(|last| sample.timestamp <= last.timestamp)
        {
            return Err(InputError::OutOfOrder);
        }
        self.vio = Some(sample);
        Ok(())
    }
    /// Advance the latest tracked VIO anchor with bounded IMU history.
    ///
    /// Unusable evidence returns zero velocity with the reason in `health`.
    /// `now` is a nonnegative finite timestamp in seconds, no earlier than the
    /// latest accepted samples. Ages equal to a timeout already count as stale.
    /// Prediction requires a retained IMU sample at or before the VIO timestamp.
    /// Forward velocity is projected into predicted body axes; yaw rate comes
    /// directly from the latest IMU sample's body z angular velocity.
    pub fn estimate(&self, now: f64) -> VelocityEstimate {
        let stopped = |health| VelocityEstimate {
            timestamp: now,
            forward: 0.0,
            yaw_rate: 0.0,
            health,
        };
        if !now.is_finite() || now < 0.0 {
            return stopped(Health::InvalidTime);
        }
        let Some(vio) = self.vio else {
            return stopped(Health::MissingVio);
        };
        if !vio.tracked {
            return stopped(Health::TrackingLost);
        }
        let Some(latest) = self.imu.back() else {
            return stopped(Health::MissingImu);
        };
        if vio.timestamp > now || latest.timestamp > now {
            return stopped(Health::InvalidTime);
        }
        if now - latest.timestamp >= self.config.imu_timeout {
            return stopped(Health::StaleImu);
        }
        if now - vio.timestamp >= self.config.vio_timeout {
            return stopped(Health::StaleVio);
        }
        // A delayed VIO sample is accepted independently of the IMU clock order.
        // Replay the bounded IMU history from that VIO timestamp to the current tick.
        let mut held = None;
        for sample in &self.imu {
            if sample.timestamp <= vio.timestamp {
                held = Some(*sample);
            } else {
                break;
            }
        }
        let Some(mut held) = held else {
            return stopped(Health::MissingImu);
        };
        let (mut cursor, mut orientation, mut velocity) =
            (vio.timestamp, vio.orientation, vio.velocity);
        let integrate =
            |dt: f64, held: ImuSample, orientation: &mut Quaternion, velocity: &mut Vector3| {
                let delta = held.angular_velocity.scaled(dt);
                let midpoint =
                    orientation.compose(Quaternion::from_rotation_vector(delta.scaled(0.5)));
                *velocity = velocity.plus(midpoint.rotate(held.acceleration).scaled(dt));
                *orientation = orientation
                    .compose(Quaternion::from_rotation_vector(delta))
                    .normalized()
                    .unwrap_or_default();
            };
        for sample in &self.imu {
            if sample.timestamp <= vio.timestamp {
                continue;
            }
            integrate(
                sample.timestamp - cursor,
                held,
                &mut orientation,
                &mut velocity,
            );
            cursor = sample.timestamp;
            held = *sample;
        }
        integrate(now - cursor, held, &mut orientation, &mut velocity);
        let body = orientation.conjugate().rotate(velocity);
        if !body.finite() {
            return stopped(Health::InvalidTime);
        }
        VelocityEstimate {
            timestamp: now,
            forward: body.x,
            yaw_rate: latest.angular_velocity.z,
            health: Health::Ready,
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    /// Generate forward acceleration with no angular motion on the shared clock.
    fn imu(t: f64, ax: f64) -> ImuSample {
        ImuSample {
            timestamp: t,
            acceleration: Vector3 {
                x: ax,
                ..Vector3::ZERO
            },
            angular_velocity: Vector3::ZERO,
        }
    }
    /// Generate a tracked, world-aligned anchor moving along world +X.
    fn vio(t: f64, v: f64) -> VioSample {
        VioSample {
            timestamp: t,
            position: Vector3::ZERO,
            orientation: Quaternion::default(),
            velocity: Vector3 {
                x: v,
                ..Vector3::ZERO
            },
            tracked: true,
        }
    }
    /// IMU acceleration advances the anchor between VIO frames, and a 90-degree
    /// heading projects world +Y motion onto the rover's forward axis.
    #[test]
    fn predicts_between_vio_frames_and_projects_world_velocity_into_body() {
        let mut estimator = VelocityEstimator::new(EstimatorConfig::default()).unwrap();
        estimator.push_imu(imu(0.0, 2.0)).unwrap();
        estimator.push_vio(vio(0.0, 1.0)).unwrap();
        estimator.push_imu(imu(0.05, 2.0)).unwrap();
        let estimate = estimator.estimate(0.05);
        assert_eq!(estimate.health, Health::Ready);
        assert!((estimate.forward - 1.1).abs() < 1e-8);
        let mut sample = vio(0.05, 0.0);
        sample.orientation = Quaternion::from_rotation_vector(Vector3 {
            z: std::f64::consts::FRAC_PI_2,
            ..Vector3::ZERO
        });
        sample.velocity = Vector3 {
            y: 2.0,
            ..Vector3::ZERO
        };
        estimator.push_vio(sample).unwrap();
        assert!((estimator.estimate(0.05).forward - 2.0).abs() < 1e-8);
    }
    /// Expired IMU evidence, regressed stream timestamps, and explicit tracking
    /// loss must be surfaced rather than producing a healthy velocity estimate.
    #[test]
    fn lost_tracking_staleness_and_out_of_order_samples_are_detected() {
        let mut estimator = VelocityEstimator::new(EstimatorConfig::default()).unwrap();
        estimator.push_imu(imu(1.0, 0.0)).unwrap();
        estimator.push_vio(vio(1.0, 1.0)).unwrap();
        assert_eq!(estimator.estimate(1.2).health, Health::StaleImu);
        assert_eq!(
            estimator.push_imu(imu(0.9, 0.0)),
            Err(InputError::OutOfOrder)
        );
        let mut sample = vio(1.01, 1.0);
        sample.tracked = false;
        estimator.push_vio(sample).unwrap();
        assert_eq!(estimator.estimate(1.01).health, Health::TrackingLost);
    }
    /// Fresh IMU data cannot refresh an old VIO anchor; future samples and invalid
    /// rotations are rejected, and reset requires both streams to be supplied again.
    #[test]
    fn fresh_imu_cannot_hide_expired_vio_and_reset_requires_new_sensors() {
        let mut estimator = VelocityEstimator::new(EstimatorConfig::default()).unwrap();
        estimator.push_imu(imu(0.0, 0.0)).unwrap();
        estimator.push_vio(vio(0.0, 1.0)).unwrap();
        estimator.push_imu(imu(0.4, 0.0)).unwrap();
        assert_eq!(estimator.estimate(0.4).health, Health::StaleVio);
        assert_eq!(estimator.estimate(0.39).health, Health::InvalidTime);
        let mut invalid = vio(0.4, 1.0);
        invalid.orientation = Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        };
        assert_eq!(
            estimator.push_vio(invalid),
            Err(InputError::InvalidQuaternion)
        );
        estimator.reset();
        assert_eq!(estimator.estimate(0.4).health, Health::MissingVio);
        estimator.push_imu(imu(0.4, 0.0)).unwrap();
        estimator.push_vio(vio(0.4, 1.0)).unwrap();
        assert_eq!(estimator.estimate(0.4).health, Health::Ready);
    }
}
