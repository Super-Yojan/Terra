//! Tilt and flip hold (Terra issue #112).
//!
//! A rover that climbs something steep or flips over must stop driving at
//! every level, L0 included. This monitor reads the rover-body attitude from
//! tracked VIO samples and latches when the body's up axis leans more than
//! [`TiltConfig::limit_degrees`] away from its reference for longer than
//! [`TiltConfig::debounce_seconds`]. The latch holds until the operator resets
//! the stop, and a reset is refused while the rover is still tilted.
//!
//! The reference is the body up axis of the first tracked sample after a
//! controller start. Measuring against it, not against world up, keeps a wrong
//! mount roll (a landscape phone with roll 0) from reading as a permanent
//! 90° tilt. The absolute angle to world up is reported for diagnosis, and
//! past [`TiltConfig::flipped_degrees`] it latches on its own, so a controller
//! started on a rover that is already upside down still holds.
use serde_json::{Value, json};
use terra_types::{Quaternion, Vector3};

/// Limits for the tilt hold.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TiltConfig {
    /// Lean from the reference up axis that counts as tilted, degrees.
    pub limit_degrees: f64,
    /// How long the lean must last before the hold latches, seconds.
    pub debounce_seconds: f64,
    /// Angle from world up that counts as upside down whatever the
    /// reference, degrees. Catches a controller restarted on a flipped rover.
    pub flipped_degrees: f64,
}

impl Default for TiltConfig {
    fn default() -> Self {
        Self {
            limit_degrees: 30.,
            debounce_seconds: 0.25,
            flipped_degrees: 120.,
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct TiltMonitor {
    pub config: TiltConfig,
    reference_up: Option<Vector3>,
    /// Latest lean from the reference and from world up, degrees, with its time.
    latest: Option<(f64, f64, f64)>,
    /// When the lean first went past the limit, while it stays past it.
    over_since: Option<f64>,
    latched_at: Option<f64>,
    peak_degrees: f64,
}

impl TiltMonitor {
    /// Record one tracked body-to-world orientation (Z-up world).
    pub fn observe(&mut self, orientation: Quaternion, timestamp: f64) {
        let Some(q) = orientation.normalized() else {
            return;
        };
        let up = q.rotate(Vector3 {
            x: 0.,
            y: 0.,
            z: 1.,
        });
        let reference = *self.reference_up.get_or_insert(up);
        let lean = angle_degrees(up, reference);
        let absolute = up.z.clamp(-1., 1.).acos().to_degrees();
        self.latest = Some((lean, absolute, timestamp));
        if lean > self.config.limit_degrees || absolute > self.config.flipped_degrees {
            self.over_since.get_or_insert(timestamp);
        } else {
            self.over_since = None;
        }
        if self.latched_at.is_some() {
            self.peak_degrees = self.peak_degrees.max(lean.max(absolute));
        }
    }

    /// Latch when the lean has stayed past the limit for the debounce time.
    /// The last known lean is kept when tracking drops, so a rover that tips
    /// past the limit and then loses tracking still latches.
    /// Returns whether the hold is latched.
    pub fn update(&mut self, now: f64) -> bool {
        if self.latched_at.is_none()
            && let Some(since) = self.over_since
            && now - since >= self.config.debounce_seconds
        {
            self.latched_at = Some(now);
            self.peak_degrees = self.latest.map_or(0., |l| l.0.max(l.1));
        }
        self.latched_at.is_some()
    }

    pub fn latched(&self) -> bool {
        self.latched_at.is_some()
    }

    /// Whether the last known lean is past the limit.
    pub fn still_tilted(&self) -> bool {
        self.over_since.is_some()
    }

    /// Operator reset: clear the latch if the rover is upright again.
    /// Returns false (and keeps the latch) while it is still tilted.
    pub fn clear(&mut self) -> bool {
        if self.still_tilted() {
            return false;
        }
        self.latched_at = None;
        self.peak_degrees = 0.;
        true
    }

    /// Forget everything, including the reference (controller restart).
    pub fn reset(&mut self) {
        *self = Self {
            config: self.config,
            ..Self::default()
        };
    }

    /// Status for `autonomy/status` → `runtime.tilt`.
    pub fn status(&self, now: f64) -> Value {
        json!({
            "latched": self.latched(),
            "tilt_deg": self.latest.map(|l| round1(l.0)),
            "absolute_tilt_deg": self.latest.map(|l| round1(l.1)),
            "sample_age_s": self.latest.map(|l| now - l.2),
            "peak_deg": self.latched().then(|| round1(self.peak_degrees)),
            "limit_deg": self.config.limit_degrees,
            "debounce_s": self.config.debounce_seconds,
            "flipped_deg": self.config.flipped_degrees,
            "reference_set": self.reference_up.is_some(),
        })
    }
}

fn angle_degrees(a: Vector3, b: Vector3) -> f64 {
    (a.x * b.x + a.y * b.y + a.z * b.z)
        .clamp(-1., 1.)
        .acos()
        .to_degrees()
}

fn round1(v: f64) -> f64 {
    (v * 10.).round() / 10.
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Body-to-world rotation about rover left (+Y) by `degrees` (pitch).
    fn pitched(degrees: f64) -> Quaternion {
        let h = degrees.to_radians() / 2.;
        Quaternion {
            x: 0.,
            y: h.sin(),
            z: 0.,
            w: h.cos(),
        }
    }

    #[test]
    fn short_bump_does_not_latch() {
        let mut m = TiltMonitor::default();
        m.observe(pitched(0.), 0.);
        m.observe(pitched(40.), 1.0);
        assert!(!m.update(1.1));
        m.observe(pitched(5.), 1.2);
        assert!(!m.update(1.5));
    }

    #[test]
    fn sustained_tilt_latches_and_reset_waits_for_upright() {
        let mut m = TiltMonitor::default();
        m.observe(pitched(0.), 0.);
        m.observe(pitched(45.), 1.0);
        assert!(!m.update(1.2));
        m.observe(pitched(60.), 1.25);
        assert!(m.update(1.25));
        assert!(!m.clear(), "reset refused while still tilted");
        m.observe(pitched(180.), 2.0);
        assert!(m.update(2.0));
        assert_eq!(m.status(2.0)["peak_deg"], 180.0);
        m.observe(pitched(2.), 5.0);
        assert!(
            m.update(5.0),
            "upright again, but the latch waits for the operator"
        );
        assert!(m.clear());
        assert!(!m.update(5.1));
    }

    #[test]
    fn lean_is_measured_from_the_first_pose_not_world_up() {
        // A landscape phone with mount roll 0 reads ~90° from world up.
        let mut m = TiltMonitor::default();
        let rolled = Quaternion {
            x: (std::f64::consts::FRAC_PI_4).sin(),
            y: 0.,
            z: 0.,
            w: (std::f64::consts::FRAC_PI_4).cos(),
        };
        m.observe(rolled, 0.);
        m.observe(rolled, 1.);
        assert!(!m.update(2.));
        assert_eq!(m.status(2.)["tilt_deg"], 0.0);
        assert_eq!(m.status(2.)["absolute_tilt_deg"], 90.0);
    }

    #[test]
    fn started_upside_down_still_latches() {
        let mut m = TiltMonitor::default();
        m.observe(pitched(170.), 0.);
        m.observe(pitched(170.), 0.3);
        assert!(m.update(0.3));
    }

    #[test]
    fn last_tilted_sample_still_latches_after_tracking_drops() {
        let mut m = TiltMonitor::default();
        m.observe(pitched(0.), 0.);
        m.observe(pitched(50.), 1.);
        // No more tracked samples.
        assert!(m.update(1.5));
    }
}
