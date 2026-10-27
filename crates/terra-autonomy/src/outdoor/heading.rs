//! True heading carried by local VIO yaw and corrected by compass and GPS course.
//!
//! The estimated state is one offset `c`, in degrees, with
//! `true_heading = c - local_yaw` (true heading is clockwise, local yaw is
//! counter-clockwise). VIO yaw changes therefore carry heading between
//! corrections, and a correction only has to estimate a slowly varying offset.
use std::collections::VecDeque;
use terra_navigation::Pose;

/// Offset variance growth while only VIO yaw carries heading, degrees² per second.
pub const DRIFT_VARIANCE_PER_S: f64 = 0.25;
/// Largest gap between a correction and the pose it is paired with, seconds.
const PAIRING_S: f64 = 0.35;
/// Window over which GPS course is compared with local travel direction, seconds.
const COURSE_WINDOW_S: f64 = 1.;
const COURSE_MIN_SPEED: f64 = 0.5;
const COURSE_MAX_TURN_DEG: f64 = 10.;
const COURSE_MIN_ACCURACY_DEG: f64 = 2.;
/// Consecutive gated corrections after which the estimate restarts from the latest one.
const MAX_REJECTS: u32 = 10;
const POSE_HISTORY_S: f64 = 5.;
/// Larger pose gaps or yaw steps mean the local frame cannot carry heading.
const POSE_GAP_S: f64 = 2.;
const MAX_YAW_RATE: f64 = 3.;

/// Direction of travel over ground from the platform location service.
#[derive(Clone, Copy, Debug)]
pub struct CourseFix {
    pub timestamp: f64,
    /// True-north clockwise course in degrees.
    pub course: f64,
    /// One-sigma course uncertainty in degrees.
    pub accuracy: f64,
    /// Ground speed in metres per second.
    pub speed: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source {
    Compass,
    GpsCourse,
}
impl Source {
    pub fn name(self) -> &'static str {
        match self {
            Source::Compass => "compass",
            Source::GpsCourse => "gps_course",
        }
    }
}

#[derive(Clone, Copy)]
struct Estimate {
    offset: f64,
    variance: f64,
    time: f64,
    source: Source,
}

#[derive(Default)]
pub struct HeadingFusion {
    poses: VecDeque<(f64, Pose)>,
    estimate: Option<Estimate>,
    rejects: u32,
}

/// Signed smallest difference `a - b` in degrees.
fn wrap(d: f64) -> f64 {
    (d + 180.).rem_euclid(360.) - 180.
}

impl HeadingFusion {
    pub fn reset(&mut self) {
        self.poses.clear();
        self.estimate = None;
        self.rejects = 0;
    }

    /// Record local pose; a tracking gap or impossible yaw step drops the estimate.
    pub fn observe_pose(&mut self, pose: Pose, t: f64) {
        if !pose.finite() || !t.is_finite() {
            return;
        }
        if let Some(&(last_t, last)) = self.poses.back() {
            if t <= last_t {
                return;
            }
            let dt = t - last_t;
            let turn = wrap((pose.yaw - last.yaw).to_degrees()).abs().to_radians();
            if dt > POSE_GAP_S || turn > MAX_YAW_RATE * dt + 0.2 {
                self.reset();
            } else if self
                .poses
                .len()
                .checked_sub(2)
                .and_then(|i| self.poses.get(i))
                .is_some_and(|(older, _)| t - older < 0.05)
            {
                // Keep the newest pose for pairing while thinning history to ~20 Hz.
                self.poses.pop_back();
            }
        }
        self.poses.push_back((t, pose));
        while self
            .poses
            .front()
            .is_some_and(|(f, _)| t - f > POSE_HISTORY_S)
        {
            self.poses.pop_front();
        }
    }

    fn latest_pose(&self) -> Option<(f64, Pose)> {
        self.poses.back().copied()
    }

    fn pose_near(&self, t: f64) -> Option<(f64, Pose)> {
        self.poses
            .iter()
            .copied()
            .min_by(|a, b| (a.0 - t).abs().total_cmp(&(b.0 - t).abs()))
            .filter(|(pt, _)| (pt - t).abs() <= 0.25)
    }

    /// Offset variance propagated to `now`.
    fn variance_at(e: &Estimate, now: f64) -> f64 {
        e.variance + DRIFT_VARIANCE_PER_S * (now - e.time).max(0.)
    }

    /// A compass sample already rotated to rover forward, paired with the latest pose.
    /// Compass error is mostly a slowly varying bias, so it never claims better than its
    /// own accuracy and is ignored once the estimate is much better than it.
    pub fn compass(
        &mut self,
        true_heading: f64,
        accuracy: f64,
        now: f64,
    ) -> Result<(), &'static str> {
        let (pt, pose) = self
            .latest_pose()
            .ok_or("heading sample has no local pose")?;
        if (now - pt).abs() > PAIRING_S {
            return Err("heading sample has no fresh local pose");
        }
        let r = accuracy.powi(2);
        let measured = true_heading + pose.yaw.to_degrees();
        if self
            .estimate
            .is_some_and(|e| r > 4. * Self::variance_at(&e, now))
        {
            return Ok(());
        }
        self.correct(measured, r, now, Source::Compass, true);
        Ok(())
    }

    /// Pair GPS course with local travel direction over the same window, so the
    /// correction holds whether the rover drives forward, reverses, or crabs.
    pub fn course(&mut self, s: CourseFix, now: f64) -> Result<(), &'static str> {
        if !s.course.is_finite()
            || !(0.0..360.).contains(&s.course)
            || !s.accuracy.is_finite()
            || s.accuracy < 0.
            || !s.speed.is_finite()
            || !s.timestamp.is_finite()
            || s.timestamp > now
            || now - s.timestamp > 3.
        {
            return Err("invalid course sample");
        }
        if s.speed < COURSE_MIN_SPEED {
            return Ok(());
        }
        let (Some((t1, p1)), Some((t0, p0))) = (
            self.pose_near(s.timestamp),
            self.pose_near(s.timestamp - COURSE_WINDOW_S),
        ) else {
            return Ok(());
        };
        let dt = t1 - t0;
        let (dx, dy) = (p1.x - p0.x, p1.y - p0.y);
        let turn = wrap((p1.yaw - p0.yaw).to_degrees()).abs();
        if dt < 0.5 * COURSE_WINDOW_S
            || dx.hypot(dy) < 0.6 * COURSE_MIN_SPEED * dt
            || turn > COURSE_MAX_TURN_DEG
        {
            return Ok(());
        }
        let measured = s.course + dy.atan2(dx).to_degrees();
        let r = s.accuracy.max(COURSE_MIN_ACCURACY_DEG).powi(2);
        self.correct(measured, r, now, Source::GpsCourse, false);
        Ok(())
    }

    fn correct(&mut self, measured: f64, r: f64, now: f64, source: Source, floor: bool) {
        let fresh = Estimate {
            offset: measured.rem_euclid(360.),
            variance: r,
            time: now,
            source,
        };
        let Some(e) = self.estimate else {
            self.estimate = Some(fresh);
            self.rejects = 0;
            return;
        };
        let p = Self::variance_at(&e, now);
        let innovation = wrap(measured - e.offset);
        if innovation.powi(2) > 9. * (p + r) {
            self.rejects += 1;
            if self.rejects >= MAX_REJECTS {
                self.estimate = Some(fresh);
                self.rejects = 0;
            }
            return;
        }
        self.rejects = 0;
        let gain = p / (p + r);
        let posterior = p * r / (p + r);
        self.estimate = Some(Estimate {
            offset: (e.offset + gain * innovation).rem_euclid(360.),
            variance: if floor {
                p.min(posterior.max(r))
            } else {
                posterior
            },
            time: now,
            source: if floor && p <= r { e.source } else { source },
        });
    }

    /// Local yaw, in radians, that points along true bearing `bearing` degrees.
    pub fn local_yaw(&self, bearing: f64) -> Option<f64> {
        self.estimate.map(|e| (e.offset - bearing).to_radians())
    }

    /// True bearing, in degrees clockwise from north, of local yaw `yaw` radians.
    pub fn bearing_of_local_yaw(&self, yaw: f64) -> Option<f64> {
        self.estimate
            .map(|e| (e.offset - yaw.to_degrees()).rem_euclid(360.))
    }

    /// Fused rover true heading, one-sigma accuracy and pose age at `now`.
    pub fn heading(&self, now: f64) -> Option<(f64, f64, f64)> {
        let e = self.estimate?;
        let (pt, pose) = self.latest_pose()?;
        Some((
            (e.offset - pose.yaw.to_degrees()).rem_euclid(360.),
            Self::variance_at(&e, now).sqrt(),
            now - pt,
        ))
    }

    pub fn source(&self) -> Option<Source> {
        self.estimate.map(|e| e.source)
    }

    pub fn correction_age(&self, now: f64) -> Option<f64> {
        self.estimate.map(|e| now - e.time)
    }
}
