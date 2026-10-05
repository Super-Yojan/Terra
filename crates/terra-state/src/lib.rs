//! VIO-anchored velocity with bounded IMU prediction, not a full VIO or EKF stack.
use std::collections::VecDeque;
use terra_types::*;
#[derive(Clone, Copy, Debug)]
pub struct EstimatorConfig {
    pub imu_timeout: f64,
    pub vio_timeout: f64,
}
impl Default for EstimatorConfig {
    fn default() -> Self {
        Self {
            imu_timeout: 0.1,
            vio_timeout: 0.35,
        }
    }
}
pub struct VelocityEstimator {
    config: EstimatorConfig,
    imu: VecDeque<ImuSample>,
    vio: Option<VioSample>,
}
impl VelocityEstimator {
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
    pub fn reset(&mut self) {
        self.imu.clear();
        self.vio = None;
    }
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
