//! GPS support for estimation: where the rover's world frame sits on Earth.
//!
//! VIO gives a smooth pose in a world frame that starts wherever the phone
//! was switched on. GPS gives noisy positions on Earth. [`AnchorFit`] lines
//! the two up, and the result is a [`GpsAnchor`]: the latitude/longitude of
//! the world origin and the compass heading of world `+X`. With it, a GPS
//! goal can be turned into a world-frame goal.
//!
//! # The fit
//! Every sample is a pair taken at the same moment:
//! - `p`: the rover's world position from VIO (m);
//! - `q`: the GPS fix as metres east and north of the first fix.
//!
//! The two are related by a rotation and a shift, `q = R(θ)·p + d`, where
//! `θ` is the angle from east to world `+X` (counter-clockwise) and `d` is
//! the world origin in the east/north map. The least-squares `θ` and `d`
//! have a closed form (2D Procrustes). With weights `w`, weighted means
//! `p̄`, `q̄`, and the centred samples `p' = p − p̄`, `q' = q − q̄`:
//!
//! ```text
//! A = Σ w · (p'.x·q'.x + p'.y·q'.y)        (dot products)
//! B = Σ w · (p'.x·q'.y − p'.y·q'.x)        (cross products)
//! θ = atan2(B, A)
//! d = q̄ − R(θ)·p̄
//! ```
//!
//! Check: if `q'` is `p'` turned by `θ`, each dot product is `|p'|²·cos θ`
//! and each cross product is `|p'|²·sin θ`, so `atan2(B, A) = θ`.
//!
//! The sums are kept running, so nothing is stored per sample. They are
//! kept un-centred (`Σ w·p·q`), and centred when read:
//! `Σ w·p'·q' = Σ w·p·q − W·p̄·q̄`, with `W = Σ w`.
use std::f64::consts::FRAC_PI_2;

use crate::geometry::{Rotation2, Vec2, wrap_angle};
use crate::sense::GpsFix;

/// Mean Earth radius (m). Good to well under GPS accuracy over a rover's range.
const EARTH_RADIUS_M: f64 = 6_371_000.0;

/// Fixes worse than this (m, 1σ) are ignored.
const MAX_FIX_ACCURACY_M: f64 = 20.0;
/// Core Location sometimes reports a very small accuracy; never trust a fix
/// more than this (m), so one sample can't outweigh all the others.
const MIN_FIX_ACCURACY_M: f64 = 1.0;
/// Take a new sample only after the rover moved this far (m). A parked
/// rover would otherwise pile all the weight onto one spot.
const SAMPLE_SPACING_M: f64 = 1.0;
/// Each new sample scales the older ones by this, so VIO drift from long
/// ago fades (the last ~50 samples, about 50 m of driving, dominate).
const FORGET: f64 = 0.98;
/// No anchor before this many samples.
const MIN_SAMPLES: u32 = 8;
/// No anchor until the samples are spread at least this far (m, RMS distance
/// from their mean). The heading comes from the direction of travel, so a
/// rover that has not moved has no heading. Its error is about
/// `GPS accuracy / (spread · √samples)` radians.
const MIN_SPREAD_M: f64 = 10.0;

/// Where the world frame sits on Earth.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GpsAnchor {
    /// World origin, WGS84 degrees.
    pub latitude: f64,
    pub longitude: f64,
    /// Compass heading of world `+X`: rad, clockwise from true north.
    pub heading: f64,
    /// How well the GPS fixes agree with the fit (m, RMS). A goal converted
    /// with this anchor can be off by about this much.
    pub accuracy: f64,
}

impl GpsAnchor {
    /// A position on Earth (WGS84 degrees) → the world frame (m).
    pub fn to_world(&self, latitude: f64, longitude: f64) -> Vec2 {
        // 1. Metres east and north of the world origin.
        let offset = east_north((self.latitude, self.longitude), (latitude, longitude));
        let (east, north) = (offset.x, offset.y);

        // 2. Project onto the world axes. World +X points `heading` clockwise
        //    from north, and world +Y is 90° to its left:
        //      x̂ = (east:  sin h, north: cos h)
        //      ŷ = (east: −cos h, north: sin h)
        let (sin_h, cos_h) = self.heading.sin_cos();
        Vec2 {
            x: east * sin_h + north * cos_h,
            y: -east * cos_h + north * sin_h,
        }
    }

    /// A compass heading (rad, clockwise from north) → a world yaw (rad,
    /// counter-clockwise from world `+X`). The two turn in opposite
    /// directions, so the yaw is the anchor's heading minus the given one.
    pub fn to_world_yaw(&self, heading: f64) -> f64 {
        wrap_angle(self.heading - heading)
    }
}

/// Metres east (`x`) and north (`y`) from `origin` to `point`, both
/// `(latitude, longitude)` in degrees. Treats the ground as flat around
/// `origin`: one degree of latitude is the same length everywhere, and a
/// degree of longitude shrinks with `cos(latitude)`.
fn east_north(origin: (f64, f64), point: (f64, f64)) -> Vec2 {
    let d_latitude = (point.0 - origin.0).to_radians();
    let d_longitude = (point.1 - origin.1).to_radians();
    Vec2 {
        x: EARTH_RADIUS_M * origin.0.to_radians().cos() * d_longitude,
        y: EARTH_RADIUS_M * d_latitude,
    }
}

/// The inverse of [`east_north`]: the `(latitude, longitude)` that lies
/// `offset` metres east/north of `origin`.
fn from_east_north(origin: (f64, f64), offset: Vec2) -> (f64, f64) {
    let d_latitude = offset.y / EARTH_RADIUS_M;
    let d_longitude = offset.x / (EARTH_RADIUS_M * origin.0.to_radians().cos());
    (origin.0 + d_latitude.to_degrees(), origin.1 + d_longitude.to_degrees())
}

/// Running least-squares fit of the anchor (see the module docs).
#[derive(Default)]
pub struct AnchorFit {
    /// The first accepted fix `(latitude, longitude)`: origin of the
    /// east/north map that every `q` is measured in.
    reference: Option<(f64, f64)>,
    /// World position of the newest sample, for [`SAMPLE_SPACING_M`].
    last_sample: Option<Vec2>,
    /// Samples taken so far.
    samples: u32,

    // Weighted running sums. p = world position, q = east/north position.
    /// `W = Σ w`
    weight: f64,
    /// `Σ w·p`
    sum_p: Vec2Sum,
    /// `Σ w·q`
    sum_q: Vec2Sum,
    /// `Σ w·|p|²`
    sum_pp: f64,
    /// `Σ w·|q|²`
    sum_qq: f64,
    /// `Σ w·(p.x·q.x + p.y·q.y)`
    sum_dot: f64,
    /// `Σ w·(p.x·q.y − p.y·q.x)`
    sum_cross: f64,
}

/// A running sum of 2D vectors.
#[derive(Default)]
struct Vec2Sum {
    x: f64,
    y: f64,
}

impl AnchorFit {
    /// Add one pair: the rover's world position and the GPS fix taken there.
    ///
    /// The fix is the phone's position and `world` is the body origin. They
    /// are a few decimetres apart, far below GPS accuracy, so the difference
    /// is ignored.
    pub fn add(&mut self, world: Vec2, fix: &GpsFix) {
        if !(fix.accuracy.is_finite() && fix.accuracy <= MAX_FIX_ACCURACY_M) {
            return;
        }
        if let Some(last) = self.last_sample
            && world.distance(last) < SAMPLE_SPACING_M
        {
            return;
        }

        let reference = *self.reference.get_or_insert((fix.latitude, fix.longitude));
        let p = world;
        let q = east_north(reference, (fix.latitude, fix.longitude));
        // Inverse-variance weight: a 2 m fix counts four times a 4 m fix.
        let w = 1.0 / fix.accuracy.max(MIN_FIX_ACCURACY_M).powi(2);

        // Fade the older samples, then add this one.
        for sum in [
            &mut self.weight,
            &mut self.sum_p.x,
            &mut self.sum_p.y,
            &mut self.sum_q.x,
            &mut self.sum_q.y,
            &mut self.sum_pp,
            &mut self.sum_qq,
            &mut self.sum_dot,
            &mut self.sum_cross,
        ] {
            *sum *= FORGET;
        }
        self.weight += w;
        self.sum_p.x += w * p.x;
        self.sum_p.y += w * p.y;
        self.sum_q.x += w * q.x;
        self.sum_q.y += w * q.y;
        self.sum_pp += w * (p.x * p.x + p.y * p.y);
        self.sum_qq += w * (q.x * q.x + q.y * q.y);
        self.sum_dot += w * (p.x * q.x + p.y * q.y);
        self.sum_cross += w * (p.x * q.y - p.y * q.x);

        self.last_sample = Some(world);
        self.samples += 1;
    }

    /// The fitted anchor, or `None` while the rover has not driven far
    /// enough for the heading to be known.
    pub fn anchor(&self) -> Option<GpsAnchor> {
        let reference = self.reference?;
        if self.samples < MIN_SAMPLES {
            return None;
        }

        // Weighted means.
        let w = self.weight;
        let p_mean = Vec2 { x: self.sum_p.x / w, y: self.sum_p.y / w };
        let q_mean = Vec2 { x: self.sum_q.x / w, y: self.sum_q.y / w };

        // Spread of each point set about its mean: Σ w·|p'|² / W.
        let p_spread = self.sum_pp / w - (p_mean.x * p_mean.x + p_mean.y * p_mean.y);
        let q_spread = self.sum_qq / w - (q_mean.x * q_mean.x + q_mean.y * q_mean.y);
        if p_spread < MIN_SPREAD_M * MIN_SPREAD_M {
            return None;
        }

        // Centred sums A and B, then the rotation θ (east → world +X, CCW).
        let a = self.sum_dot - w * (p_mean.x * q_mean.x + p_mean.y * q_mean.y);
        let b = self.sum_cross - w * (p_mean.x * q_mean.y - p_mean.y * q_mean.x);
        let theta = b.atan2(a);

        // World origin in the east/north map: d = q̄ − R(θ)·p̄.
        let rotated_p_mean = Rotation2::new(theta).apply(p_mean);
        let origin = Vec2 {
            x: q_mean.x - rotated_p_mean.x,
            y: q_mean.y - rotated_p_mean.y,
        };
        let (latitude, longitude) = from_east_north(reference, origin);

        // Mean squared misfit, Σ w·|q' − R(θ)·p'|² / W. Expanding the square:
        //   |q'|² + |p'|² − 2·(cos θ·dot + sin θ·cross)
        // and (cos θ, sin θ) = (A, B) / √(A² + B²), so the last term sums
        // to 2·√(A² + B²).
        let misfit = p_spread + q_spread - 2.0 * a.hypot(b) / w;

        Some(GpsAnchor {
            latitude,
            longitude,
            // θ is counter-clockwise from east; a compass heading is
            // clockwise from north; north is 90° counter-clockwise from east.
            heading: wrap_angle(FRAC_PI_2 - theta),
            accuracy: misfit.max(0.0).sqrt(),
        })
    }
}
