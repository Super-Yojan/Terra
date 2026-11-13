//! Tilt and flip hold (Terra issue #112).
//!
//! A rover that climbs something steep or flips over must stop driving at
//! every level, L0 included. This monitor tracks world up as seen from the
//! rover body (the gravity direction the phone sees) and latches when it leans
//! more than [`LIMIT_DEGREES`] away from its reference for longer
//! than [`DEBOUNCE_SECONDS`]. The latch holds until the operator
//! resets the stop, and a reset is refused while the rover is still tilted.
//!
//! World up in the body frame does not change when the rover turns, so
//! heading never reads as lean. Both phone apps apply the mount calibration
//! before `push_vio`, so body +Z is rover up and an upright rover sees world
//! up at body +Z.
//!
//! Sources: every tracked VIO orientation sets the estimate. Between them,
//! and for up to [`GYRO_HOLD_SECONDS`] after tracking drops, body gyro rates
//! carry it forward, so a fast flip that loses ARKit tracking still latches.
//!
//! The reference is the first estimate after a controller start, which
//! absorbs a few degrees of mount or floor slope. An estimate already past the
//! limit from body +Z is never taken as the reference: the nominal mount (body
//! +Z) is used instead, so a controller restarted on a rover lying on its side
//! stays held. Past [`FLIPPED_DEGREES`] from body +Z the hold
//! latches whatever the reference.
use serde_json::{Value, json};
use terra_types::{Quaternion, Vector3};

/// How long gyro rates alone keep the estimate moving after the last tracked
/// VIO sample, seconds. Later the last estimate is kept as it is.
pub const GYRO_HOLD_SECONDS: f64 = 5.;
/// Longest gyro step integrated; longer gaps skip propagation.
const MAX_GYRO_STEP: f64 = 0.1;
const BODY_UP: Vector3 = Vector3 {
    x: 0.,
    y: 0.,
    z: 1.,
};

// Placeholders until the hand-tilt test on hardware sets them (#112).
/// Lean from the reference that counts as tilted, degrees.
pub const LIMIT_DEGREES: f64 = 30.;
/// How long the lean must last before the hold latches, seconds.
pub const DEBOUNCE_SECONDS: f64 = 0.25;
/// Angle between world up and body +Z that counts as upside down whatever
/// the reference, degrees.
pub const FLIPPED_DEGREES: f64 = 120.;

#[derive(Clone, Debug, Default)]
pub struct TiltMonitor {
    /// World up in the body frame, and when it was last set or propagated.
    up: Option<(Vector3, f64)>,
    last_tracked: Option<f64>,
    /// Reference world-up in the body frame, and whether it is the nominal mount.
    reference: Option<(Vector3, bool)>,
    /// Latest lean from the reference and from body +Z, degrees, with its time.
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
        self.last_tracked = Some(timestamp);
        self.set(q.conjugate().rotate(BODY_UP), timestamp);
    }

    /// Carry the estimate forward with body angular rates (rad/s about body
    /// +X forward, +Y left, +Z up). Ignored before the first tracked sample,
    /// for out-of-order or long gaps, and once tracking has been gone for
    /// [`GYRO_HOLD_SECONDS`].
    pub fn observe_gyro(&mut self, rate: Vector3, timestamp: f64) {
        let Some((up, at)) = self.up else {
            return;
        };
        let dt = timestamp - at;
        if dt.is_nan() || dt <= 0. || ![rate.x, rate.y, rate.z].iter().all(|v| v.is_finite()) {
            return;
        }
        if dt > MAX_GYRO_STEP
            || self
                .last_tracked
                .is_none_or(|t| timestamp - t > GYRO_HOLD_SECONDS)
        {
            self.up = Some((up, timestamp));
            return;
        }
        // A world-fixed vector seen from a rotating body: d/dt = -ω × v.
        let cross = Vector3 {
            x: rate.y * up.z - rate.z * up.y,
            y: rate.z * up.x - rate.x * up.z,
            z: rate.x * up.y - rate.y * up.x,
        };
        let next = Vector3 {
            x: up.x - cross.x * dt,
            y: up.y - cross.y * dt,
            z: up.z - cross.z * dt,
        };
        let norm = (next.x * next.x + next.y * next.y + next.z * next.z).sqrt();
        if norm.is_finite() && norm > 0. {
            self.set(
                Vector3 {
                    x: next.x / norm,
                    y: next.y / norm,
                    z: next.z / norm,
                },
                timestamp,
            );
        }
    }

    fn set(&mut self, up: Vector3, timestamp: f64) {
        self.up = Some((up, timestamp));
        let absolute = angle_degrees(up, BODY_UP);
        let (reference, _) = *self.reference.get_or_insert(if absolute <= LIMIT_DEGREES {
            (up, false)
        } else {
            (BODY_UP, true)
        });
        let lean = angle_degrees(up, reference);
        self.latest = Some((lean, absolute, timestamp));
        if lean > LIMIT_DEGREES || absolute > FLIPPED_DEGREES {
            self.over_since.get_or_insert(timestamp);
        } else {
            self.over_since = None;
        }
        if self.latched_at.is_some() {
            self.peak_degrees = self.peak_degrees.max(lean.max(absolute));
        }
    }

    /// Latch when the lean has stayed past the limit for the debounce time.
    /// The last known lean is kept when no new estimate arrives, so a rover
    /// that tips past the limit and then loses every source still latches.
    /// Returns whether the hold is latched.
    pub fn update(&mut self, now: f64) -> bool {
        if self.latched_at.is_none()
            && let Some(since) = self.over_since
            && now - since >= DEBOUNCE_SECONDS
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

    /// Controller restart: forget the latch, estimate and reference. A rover
    /// that is still tilted re-latches from its first estimate (see the
    /// module notes on the reference).
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Status for `autonomy/status` → `runtime.tilt`.
    pub fn status(&self, now: f64) -> Value {
        json!({
            "latched": self.latched(),
            "still_tilted": self.still_tilted(),
            "tilt_deg": self.latest.map(|l| round1(l.0)),
            "absolute_tilt_deg": self.latest.map(|l| round1(l.1)),
            "sample_age_s": self.latest.map(|l| now - l.2),
            "tracked_age_s": self.last_tracked.map(|t| now - t),
            "peak_deg": self.latched().then(|| round1(self.peak_degrees)),
            "limit_deg": LIMIT_DEGREES,
            "debounce_s": DEBOUNCE_SECONDS,
            "flipped_deg": FLIPPED_DEGREES,
            "reference": self.reference.map(|(_, nominal)| if nominal { "nominal_mount" } else { "first_pose" }),
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

    /// Body-to-world rotation of `degrees` about a unit axis.
    fn about(axis: (f64, f64, f64), degrees: f64) -> Quaternion {
        let h = degrees.to_radians() / 2.;
        Quaternion {
            x: axis.0 * h.sin(),
            y: axis.1 * h.sin(),
            z: axis.2 * h.sin(),
            w: h.cos(),
        }
    }
    /// Pitch about rover left (+Y).
    fn pitched(degrees: f64) -> Quaternion {
        about((0., 1., 0.), degrees)
    }
    fn product(a: Quaternion, b: Quaternion) -> Quaternion {
        Quaternion {
            w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
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
    fn a_full_turn_on_the_floor_never_latches() {
        let mut m = TiltMonitor::default();
        // A few degrees of floor slope, then 360 degrees of heading.
        let slope = about((1., 0., 0.), 4.);
        for step in 0..=72 {
            let yaw = about((0., 0., 1.), step as f64 * 5.);
            let t = step as f64 * 0.1;
            m.observe(product(yaw, slope), t);
            assert!(!m.update(t), "latched at yaw {}", step * 5);
        }
        assert!(m.status(7.2)["tilt_deg"].as_f64().unwrap() < 0.1);
    }

    #[test]
    fn restart_on_its_side_stays_held() {
        let mut m = TiltMonitor::default();
        m.observe(pitched(0.), 0.);
        m.observe(pitched(90.), 1.);
        assert!(m.update(1.5));
        m.reset();
        // The first estimate after the restart is 90 degrees from body up, so
        // it is not taken as the reference.
        m.observe(pitched(90.), 2.);
        m.observe(pitched(90.), 2.3);
        assert!(m.update(2.3));
        assert_eq!(m.status(2.3)["reference"], "nominal_mount");
        assert!(!m.clear());
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

    #[test]
    fn gyro_carries_a_fast_flip_after_tracking_drops() {
        let mut m = TiltMonitor::default();
        m.observe(pitched(0.), 0.);
        m.observe(pitched(10.), 1.);
        // Tracking is gone; the body pitches back at 3 rad/s for 0.5 s.
        let mut t = 1.;
        while t < 1.5 {
            t += 0.01;
            m.observe_gyro(
                Vector3 {
                    x: 0.,
                    y: -3.,
                    z: 0.,
                },
                t,
            );
        }
        assert!(m.status(t)["tilt_deg"].as_f64().unwrap() > 60.);
        assert!(m.update(t));
    }

    #[test]
    fn gyro_turning_in_place_does_not_lean() {
        let mut m = TiltMonitor::default();
        m.observe(pitched(0.), 0.);
        let mut t = 0.;
        while t < 4. {
            t += 0.01;
            m.observe_gyro(
                Vector3 {
                    x: 0.,
                    y: 0.,
                    z: 2.,
                },
                t,
            );
            assert!(!m.update(t));
        }
    }

    #[test]
    fn gyro_stops_after_the_hold_time() {
        let mut m = TiltMonitor::default();
        m.observe(pitched(0.), 0.);
        let mut t = 0.;
        while t < GYRO_HOLD_SECONDS + 1. {
            t += 0.05;
            m.observe_gyro(
                Vector3 {
                    x: 0.,
                    y: 0.001,
                    z: 0.,
                },
                t,
            );
        }
        let held = m.status(t)["tilt_deg"].as_f64().unwrap();
        m.observe_gyro(
            Vector3 {
                x: 0.,
                y: 5.,
                z: 0.,
            },
            t + 0.05,
        );
        assert_eq!(m.status(t + 0.05)["tilt_deg"].as_f64().unwrap(), held);
    }
}
