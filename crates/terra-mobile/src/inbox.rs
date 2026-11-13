//! Sensor samples pushed from app threads and held until the control tick.
//!
//! Each sensor has its own bounded queue behind its own small lock, separate
//! from the controller state, so a sensor callback never waits for a running
//! tick. [`crate::MobileController::step`] drains every queue, orders the
//! samples by timestamp and applies them before computing the output.
//!
//! A full queue drops its oldest sample. A sample the runtime rejects when it is
//! applied (out of order, invalid values) is dropped and counted; it does not
//! stop the controller. If a sensor really stops, the freshness checks turn the
//! output neutral as before.
use crate::{CourseFix, GpsFix, HeadingFix, ImuReading, VioReading};
use std::collections::VecDeque;
use std::sync::Mutex;

/// IMU arrives at about 100 Hz; this holds 2.5 s if the tick stalls.
const IMU_CAPACITY: usize = 256;
/// VIO arrives at the AR frame rate (about 60 Hz).
const VIO_CAPACITY: usize = 64;
/// GPS, heading and course arrive at about 1 to 10 Hz.
const OUTDOOR_CAPACITY: usize = 32;

/// One queued sensor reading. Outdoor fixes keep the `now` given at push time,
/// because their freshness is judged against it.
#[derive(Clone, Debug)]
pub(crate) enum Sample {
    Imu(ImuReading),
    Vio(VioReading),
    Gps(GpsFix, f64),
    Heading(HeadingFix, f64),
    Course(CourseFix, f64),
}

impl Sample {
    fn timestamp(&self) -> f64 {
        match self {
            Self::Imu(s) => s.timestamp,
            Self::Vio(s) => s.timestamp,
            Self::Gps(s, _) => s.timestamp,
            Self::Heading(s, _) => s.timestamp,
            Self::Course(s, _) => s.timestamp,
        }
    }
}

struct Lane {
    samples: VecDeque<Sample>,
    capacity: usize,
    overflow_dropped: u64,
}

impl Lane {
    fn new(capacity: usize) -> Mutex<Self> {
        Mutex::new(Self {
            samples: VecDeque::with_capacity(capacity),
            capacity,
            overflow_dropped: 0,
        })
    }
}

#[derive(Default)]
struct Rejections {
    count: u64,
    last: Option<String>,
}

/// Per-sensor queues between the app's sensor threads and the control tick.
pub(crate) struct SensorInbox {
    imu: Mutex<Lane>,
    vio: Mutex<Lane>,
    outdoor: Mutex<Lane>,
    rejections: Mutex<Rejections>,
}

impl Default for SensorInbox {
    fn default() -> Self {
        Self {
            imu: Lane::new(IMU_CAPACITY),
            vio: Lane::new(VIO_CAPACITY),
            outdoor: Lane::new(OUTDOOR_CAPACITY),
            rejections: Mutex::default(),
        }
    }
}

/// Queue depths and drop counters, for diagnostics.
#[derive(Clone, Debug, Default, PartialEq, uniffi::Record)]
pub struct SensorQueueStats {
    /// IMU samples waiting for the next tick.
    pub imu_queued: u32,
    /// VIO samples waiting for the next tick.
    pub vio_queued: u32,
    /// GPS, heading and course fixes waiting for the next tick.
    pub outdoor_queued: u32,
    /// Samples dropped because their queue was full, since the controller was created.
    pub overflow_dropped: u64,
    /// Samples the runtime rejected when they were applied.
    pub rejected: u64,
    /// Why the most recent sample was rejected, if any was.
    pub last_rejection: Option<String>,
}

impl SensorInbox {
    /// Queue one sample. Never waits for the control tick.
    pub(crate) fn push(&self, sample: Sample) -> Result<(), crate::ControllerError> {
        if !sample.timestamp().is_finite() {
            return Err(crate::ControllerError::InvalidInput {
                message: "nonfinite sensor timestamp".into(),
            });
        }
        let lane = match sample {
            Sample::Imu(_) => &self.imu,
            Sample::Vio(_) => &self.vio,
            Sample::Gps(..) | Sample::Heading(..) | Sample::Course(..) => &self.outdoor,
        };
        let mut lane = lane.lock().map_err(|_| crate::ControllerError::Internal)?;
        if lane.samples.len() == lane.capacity {
            lane.samples.pop_front();
            lane.overflow_dropped += 1;
        }
        lane.samples.push_back(sample);
        Ok(())
    }

    /// Take every queued sample, oldest timestamp first.
    pub(crate) fn drain(&self) -> Vec<Sample> {
        let mut samples = Vec::new();
        for lane in [&self.imu, &self.vio, &self.outdoor] {
            if let Ok(mut lane) = lane.lock() {
                samples.extend(lane.samples.drain(..));
            }
        }
        // Stable: samples with equal timestamps keep IMU, VIO, outdoor order.
        samples.sort_by(|a, b| a.timestamp().total_cmp(&b.timestamp()));
        samples
    }

    pub(crate) fn reject(&self, reason: String) {
        if let Ok(mut r) = self.rejections.lock() {
            r.count += 1;
            r.last = Some(reason);
        }
    }

    /// Forget queued GPS, heading and course fixes.
    pub(crate) fn clear_outdoor(&self) {
        if let Ok(mut lane) = self.outdoor.lock() {
            lane.samples.clear();
        }
    }

    /// Forget every queued sample.
    pub(crate) fn clear(&self) {
        for lane in [&self.imu, &self.vio, &self.outdoor] {
            if let Ok(mut lane) = lane.lock() {
                lane.samples.clear();
            }
        }
    }

    pub(crate) fn stats(&self) -> SensorQueueStats {
        let mut stats = SensorQueueStats::default();
        for (lane, queued) in [
            (&self.imu, &mut stats.imu_queued),
            (&self.vio, &mut stats.vio_queued),
            (&self.outdoor, &mut stats.outdoor_queued),
        ] {
            if let Ok(lane) = lane.lock() {
                *queued = lane.samples.len() as u32;
                stats.overflow_dropped += lane.overflow_dropped;
            }
        }
        if let Ok(r) = self.rejections.lock() {
            stats.rejected = r.count;
            stats.last_rejection = r.last.clone();
        }
        stats
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn imu(t: f64) -> Sample {
        Sample::Imu(ImuReading {
            timestamp: t,
            acceleration_forward: 0.,
            acceleration_left: 0.,
            acceleration_up: 0.,
            gyro_roll: 0.,
            gyro_pitch: 0.,
            gyro_yaw: 0.,
        })
    }

    #[test]
    fn drain_orders_across_sensors_and_empties_the_queues() {
        let inbox = SensorInbox::default();
        inbox.push(imu(0.3)).unwrap();
        inbox
            .push(Sample::Gps(
                GpsFix {
                    timestamp: 0.1,
                    latitude: 0.,
                    longitude: 0.,
                    accuracy: 1.,
                },
                0.1,
            ))
            .unwrap();
        inbox.push(imu(0.2)).unwrap();
        let times: Vec<f64> = inbox.drain().iter().map(Sample::timestamp).collect();
        assert_eq!(times, vec![0.1, 0.2, 0.3]);
        assert!(inbox.drain().is_empty());
    }

    #[test]
    fn a_full_queue_drops_its_oldest_sample() {
        let inbox = SensorInbox::default();
        for i in 0..IMU_CAPACITY + 3 {
            inbox.push(imu(i as f64)).unwrap();
        }
        let stats = inbox.stats();
        assert_eq!(stats.imu_queued as usize, IMU_CAPACITY);
        assert_eq!(stats.overflow_dropped, 3);
        assert_eq!(inbox.drain()[0].timestamp(), 3.0);
    }
}
