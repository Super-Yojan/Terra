//! Rover geographic position: GPS fixes smoothed by VIO travel between them.
//!
//! The local (VIO) frame and the geographic frame drift apart: GPS jumps by a
//! few metres from fix to fix, and VIO slowly accumulates scale and heading
//! error. This tracker keeps one rover position in a local east/north plane.
//! VIO displacement, rotated by the fused heading, predicts it between fixes,
//! and each GPS fix corrects it by its reported accuracy (a 2-D Kalman filter
//! with an isotropic variance). Fixes that disagree by more than three sigma
//! are rejected, and three in a row re-anchor the estimate on the newest fix,
//! so a real frame jump is followed within a few seconds.
use terra_navigation::Pose;

const EARTH_RADIUS_M: f64 = 6_371_000.;
/// VIO scale error per metre travelled, as a fraction.
const VIO_SCALE_ERROR: f64 = 0.08;
/// Smallest GPS sigma trusted, in metres.
const MIN_GPS_SIGMA_M: f64 = 1.;
/// Innovations smaller than this are never rejected, in metres.
const MIN_REJECT_M: f64 = 3.;
/// Consecutive rejected fixes that re-anchor on the newest fix.
const MAX_REJECTS: u32 = 3;
/// A pose step longer than this, or a gap longer than `POSE_GAP_S`, is a frame jump.
const POSE_JUMP_M: f64 = 1.;
const POSE_GAP_S: f64 = 2.;
/// Re-centre the east/north plane beyond this distance, in metres.
const MAX_PLANE_M: f64 = 2_000.;

#[derive(Default)]
pub struct GeoAnchor {
    /// Reference latitude and longitude of the east/north plane, in degrees.
    origin: Option<(f64, f64)>,
    /// Rover east and north in metres, and its variance in square metres.
    rover: Option<(f64, f64, f64)>,
    last_pose: Option<(f64, Pose)>,
    rejects: u32,
    /// Times the estimate restarted from a single fix after it had one.
    pub reanchors: u64,
    /// Distance between the last fix and the estimate, in metres.
    pub last_innovation: Option<f64>,
}

impl GeoAnchor {
    pub fn reset(&mut self) {
        if self.rover.take().is_some() {
            self.reanchors += 1;
        }
        self.last_pose = None;
        self.rejects = 0;
        self.last_innovation = None;
    }

    fn geo_to_plane(&self, lat: f64, lon: f64) -> Option<(f64, f64)> {
        let (lat0, lon0) = self.origin?;
        let east = (lon - lon0).to_radians() * lat0.to_radians().cos() * EARTH_RADIUS_M;
        let north = (lat - lat0).to_radians() * EARTH_RADIUS_M;
        Some((east, north))
    }

    fn plane_to_geo(&self, east: f64, north: f64) -> Option<(f64, f64)> {
        let (lat0, lon0) = self.origin?;
        let lat = lat0 + (north / EARTH_RADIUS_M).to_degrees();
        let lon = lon0 + (east / (EARTH_RADIUS_M * lat0.to_radians().cos())).to_degrees();
        Some((lat, lon))
    }

    /// Carry the estimate by VIO travel. `east_yaw` is the local yaw of true east,
    /// with its one-sigma error in radians; without it the estimate is dropped.
    pub fn observe_pose(&mut self, pose: Pose, t: f64, east_yaw: Option<(f64, f64)>) {
        if !pose.finite() || !t.is_finite() {
            return;
        }
        let last = self.last_pose.replace((t, pose));
        let Some((last_t, last)) = last else {
            return;
        };
        if t <= last_t {
            self.last_pose = Some((last_t, last));
            return;
        }
        let (dx, dy) = (pose.x - last.x, pose.y - last.y);
        let step = dx.hypot(dy);
        if t - last_t > POSE_GAP_S || step > POSE_JUMP_M {
            self.reset();
            self.last_pose = Some((t, pose));
            return;
        }
        let Some((east, north, variance)) = self.rover else {
            return;
        };
        if step < 1e-9 {
            return;
        }
        let Some((yaw, sigma)) = east_yaw.filter(|(y, s)| y.is_finite() && s.is_finite()) else {
            // Travel without a heading cannot be placed; wait for the next fix.
            self.reset();
            self.last_pose = Some((t, pose));
            return;
        };
        let angle = dy.atan2(dx) - yaw;
        let error = step * (VIO_SCALE_ERROR + sigma.abs());
        self.rover = Some((
            east + step * angle.cos(),
            north + step * angle.sin(),
            variance + error * error,
        ));
    }

    /// Correct the estimate with a GPS fix of one-sigma `accuracy` metres.
    pub fn fix(&mut self, lat: f64, lon: f64, accuracy: f64) {
        if self
            .geo_to_plane(lat, lon)
            .is_none_or(|(e, n)| e.hypot(n) > MAX_PLANE_M)
        {
            self.origin = Some((lat, lon));
            if self.rover.take().is_some() {
                self.reanchors += 1;
            }
        }
        let Some((e, n)) = self.geo_to_plane(lat, lon) else {
            return;
        };
        let r = accuracy.max(MIN_GPS_SIGMA_M).powi(2);
        let Some((east, north, p)) = self.rover else {
            self.rover = Some((e, n, r));
            self.rejects = 0;
            self.last_innovation = None;
            return;
        };
        let (ve, vn) = (e - east, n - north);
        let innovation = ve.hypot(vn);
        self.last_innovation = Some(innovation);
        // 2-D chi-square at 99% is 9.21.
        if innovation > MIN_REJECT_M && innovation.powi(2) > 9.21 * (p + r) {
            self.rejects += 1;
            if self.rejects >= MAX_REJECTS {
                self.rover = Some((e, n, r));
                self.rejects = 0;
                self.reanchors += 1;
            }
            return;
        }
        self.rejects = 0;
        let gain = p / (p + r);
        self.rover = Some((east + gain * ve, north + gain * vn, p * r / (p + r)));
    }

    /// Smoothed rover latitude, longitude and one-sigma error in metres.
    pub fn position(&self) -> Option<(f64, f64, f64)> {
        let (east, north, p) = self.rover?;
        let (lat, lon) = self.plane_to_geo(east, north)?;
        Some((lat, lon, p.sqrt()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const LAT: f64 = 42.;
    fn north_of(m: f64) -> f64 {
        LAT + (m / EARTH_RADIUS_M).to_degrees()
    }
    fn pose(x: f64) -> Pose {
        Pose { x, y: 0., yaw: 0. }
    }
    /// Local +x points north when the local yaw of east is -90 degrees.
    const EAST: Option<(f64, f64)> = Some((-std::f64::consts::FRAC_PI_2, 0.02));

    fn north_m(a: &GeoAnchor) -> f64 {
        let (lat, _, _) = a.position().unwrap();
        (lat - LAT).to_radians() * EARTH_RADIUS_M
    }

    #[test]
    fn first_fix_is_the_position() {
        let mut a = GeoAnchor::default();
        a.fix(LAT, 7., 3.);
        let (lat, lon, sigma) = a.position().unwrap();
        assert_eq!((lat, lon), (LAT, 7.));
        assert_eq!(sigma, 3.);
    }

    #[test]
    fn vio_carries_the_estimate_and_gps_noise_is_averaged() {
        let mut a = GeoAnchor::default();
        a.observe_pose(pose(0.), 0., EAST);
        a.fix(LAT, 0., 3.);
        // Drive 20 m north at 1 m/s while GPS alternates ±2.5 m around the truth.
        for i in 1..=200 {
            let t = i as f64 * 0.1;
            a.observe_pose(pose(t), t, EAST);
            if i % 10 == 0 {
                let noise = if i % 20 == 0 { 2.5 } else { -2.5 };
                a.fix(north_of(t + noise), 0., 3.);
            }
        }
        assert!((north_m(&a) - 20.).abs() < 1.5, "{}", north_m(&a));
        assert!(a.position().unwrap().2 < 3.);
        assert_eq!(a.reanchors, 0);
    }

    #[test]
    fn a_single_outlier_is_rejected_and_a_persistent_one_reanchors() {
        let mut a = GeoAnchor::default();
        a.observe_pose(pose(0.), 0., EAST);
        for _ in 0..5 {
            a.fix(LAT, 0., 2.);
        }
        a.fix(north_of(30.), 0., 2.);
        assert!(north_m(&a).abs() < 0.5);
        a.fix(north_of(30.), 0., 2.);
        a.fix(north_of(30.), 0., 2.);
        assert!((north_m(&a) - 30.).abs() < 1e-6);
        assert_eq!(a.reanchors, 1);
    }

    #[test]
    fn a_local_frame_jump_or_missing_heading_waits_for_the_next_fix() {
        let mut a = GeoAnchor::default();
        a.observe_pose(pose(0.), 0., EAST);
        a.fix(LAT, 0., 2.);
        a.observe_pose(pose(5.), 0.1, EAST);
        assert!(a.position().is_none());
        a.fix(LAT, 0., 2.);
        a.observe_pose(pose(5.5), 0.2, None);
        assert!(a.position().is_none());
        assert_eq!(a.reanchors, 2);
    }
}
