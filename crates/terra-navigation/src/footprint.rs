//! The rover body as an oriented rectangle, shared by every planner.
//!
//! A cell blocks the body when the rectangle at the body's heading overlaps it:
//! occupied cells (occupancy at least [`OCCUPIED`]) and off-map cells always,
//! unknown cells (occupancy below 0) unless they also lie under the body at the
//! starting pose, so the rover can drive off its own unobserved underbody.
use crate::{Pose, valid_map};
use serde::{Deserialize, Serialize};
use terra_mapping::MapSnapshot;

/// Occupancy at or above this value is an obstacle.
pub const OCCUPIED: i8 = 65;

/// Rover body size and the safety margin kept around it.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Footprint {
    /// Body length along the heading, metres.
    pub length: f64,
    /// Body width across the heading, metres.
    pub width: f64,
    /// Clearance kept on every side of the body, metres.
    pub margin: f64,
}

impl Default for Footprint {
    /// Terra: 0.61 m square with 5 cm on every side.
    fn default() -> Self {
        Self {
            length: 0.61,
            width: 0.61,
            margin: 0.05,
        }
    }
}

impl Footprint {
    /// Finite, positive body up to 5 m, margin in `0..=1` m.
    pub fn valid(&self) -> bool {
        [self.length, self.width]
            .iter()
            .all(|v| v.is_finite() && *v > 0. && *v <= 5.)
            && self.margin.is_finite()
            && (0.0..=1.).contains(&self.margin)
    }
    /// Half length and half width including the margin and an extra `pad`.
    pub fn half_extents(&self, pad: f64) -> (f64, f64) {
        (
            self.length / 2. + self.margin + pad,
            self.width / 2. + self.margin + pad,
        )
    }
    /// Radius of the circle around the body and margin: room the body needs at
    /// any heading. Use only where the heading is genuinely unknown.
    pub fn circumradius(&self) -> f64 {
        self.length.hypot(self.width) / 2. + self.margin
    }
    /// Radius of the largest circle inside the body and margin.
    pub fn inradius(&self) -> f64 {
        self.length.min(self.width) / 2. + self.margin
    }
}

/// Why the body (grown by `pad` on every side) cannot stand at `pose`, or `None`
/// when it can. Unknown cells under the body at `start` are allowed.
/// Returns `outside_map`, `occupied` or `unknown`.
pub fn footprint_rejection_at(
    map: &MapSnapshot,
    footprint: &Footprint,
    pose: Pose,
    pad: f64,
    start: Option<Pose>,
) -> Option<&'static str> {
    rejection(
        map,
        footprint,
        pose,
        pad,
        start.map_or(Unknown::Refused, Unknown::UnderBody),
    )
}

/// Where unknown cells may be crossed.
#[derive(Clone, Copy)]
pub(crate) enum Unknown {
    Refused,
    /// Under the body at this pose: the rover's own unobserved underbody.
    UnderBody(Pose),
    /// Within the disc the body sweeps spinning in place at this pose.
    SpinDisc(Pose),
}

pub(crate) fn rejection(
    map: &MapSnapshot,
    footprint: &Footprint,
    pose: Pose,
    pad: f64,
    unknown: Unknown,
) -> Option<&'static str> {
    if !valid_map(map) || !pose.finite() {
        return Some("outside_map");
    }
    let (a, b) = footprint.half_extents(pad);
    let mut reason = None;
    for_each_overlapping_cell(map, pose.x, pose.y, pose.yaw, a, b, |cell| {
        let Some(i) = cell else {
            reason = Some("outside_map");
            return false;
        };
        let occupancy = map.occupancy[i];
        if occupancy >= OCCUPIED {
            reason = Some("occupied");
            return false;
        }
        let allowed = match unknown {
            Unknown::Refused => false,
            Unknown::UnderBody(s) => cell_overlaps(map, i, s, footprint.half_extents(pad)),
            Unknown::SpinDisc(s) => {
                let width = map.width as usize;
                let cx = map.origin_x + ((i % width) as f64 + 0.5) * map.resolution;
                let cy = map.origin_y + ((i / width) as f64 + 0.5) * map.resolution;
                (cx - s.x).hypot(cy - s.y)
                    <= footprint.circumradius()
                        + pad
                        + map.resolution * std::f64::consts::FRAC_1_SQRT_2
            }
        };
        if occupancy < 0 && !allowed {
            reason = Some("unknown");
            return false;
        }
        true
    });
    reason
}

/// Whether the body can stand at `pose`; see [`footprint_rejection_at`].
pub fn footprint_fits(
    map: &MapSnapshot,
    footprint: &Footprint,
    pose: Pose,
    start: Option<Pose>,
) -> bool {
    footprint_rejection_at(map, footprint, pose, 0., start).is_none()
}

/// Whether map cell `i` overlaps the rectangle with half extents `(a, b)` at `pose`.
fn cell_overlaps(map: &MapSnapshot, i: usize, pose: Pose, (a, b): (f64, f64)) -> bool {
    let width = map.width as usize;
    let cx = map.origin_x + ((i % width) as f64 + 0.5) * map.resolution;
    let cy = map.origin_y + ((i / width) as f64 + 0.5) * map.resolution;
    overlaps(
        cx - pose.x,
        cy - pose.y,
        pose.yaw.sin_cos(),
        a,
        b,
        map.resolution / 2.,
    )
}

/// Separating-axis test of a cell square (half size `half`, centre offset
/// `(dx, dy)` from the body centre) against the body rectangle.
fn overlaps(dx: f64, dy: f64, (sin, cos): (f64, f64), a: f64, b: f64, half: f64) -> bool {
    let along = (dx * cos + dy * sin).abs();
    let across = (-dx * sin + dy * cos).abs();
    let spread = half * (cos.abs() + sin.abs());
    let x_reach = a * cos.abs() + b * sin.abs();
    let y_reach = a * sin.abs() + b * cos.abs();
    along <= a + spread
        && across <= b + spread
        && dx.abs() <= x_reach + half
        && dy.abs() <= y_reach + half
}

/// Visit every cell the rectangle (centre, yaw, half extents `a` × `b`) overlaps,
/// with `None` for cells off the map. Stops early, returning false, when `visit` does.
pub(crate) fn for_each_overlapping_cell(
    map: &MapSnapshot,
    x: f64,
    y: f64,
    yaw: f64,
    a: f64,
    b: f64,
    mut visit: impl FnMut(Option<usize>) -> bool,
) -> bool {
    let (sin, cos) = yaw.sin_cos();
    let res = map.resolution;
    let reach_x = a * cos.abs() + b * sin.abs();
    let reach_y = a * sin.abs() + b * cos.abs();
    let column = |v: f64| ((v - map.origin_x) / res).floor() as i64;
    let row = |v: f64| ((v - map.origin_y) / res).floor() as i64;
    for r in row(y - reach_y)..=row(y + reach_y) {
        for c in column(x - reach_x)..=column(x + reach_x) {
            let cx = map.origin_x + (c as f64 + 0.5) * res;
            let cy = map.origin_y + (r as f64 + 0.5) * res;
            if !overlaps(cx - x, cy - y, (sin, cos), a, b, res / 2.) {
                continue;
            }
            let inside = c >= 0 && r >= 0 && c < map.width as i64 && r < map.height as i64;
            let id = inside.then(|| r as usize * map.width as usize + c as usize);
            if !visit(id) {
                return false;
            }
        }
    }
    true
}

/// Distance from the body (with margin) at `pose` to the nearest occupied cell,
/// or `None` when no occupied cell is within `reach` metres. Zero when touching.
/// Each cell is treated as the disc around its square, so the result can read up
/// to half a cell diagonal low: it never overstates clearance.
pub fn rectangle_clearance(
    map: &MapSnapshot,
    footprint: &Footprint,
    pose: Pose,
    reach: f64,
) -> Option<f64> {
    if !valid_map(map) || !pose.finite() || !footprint.valid() {
        return None;
    }
    let (a, b) = footprint.half_extents(0.);
    let (sin, cos) = pose.yaw.sin_cos();
    let res = map.resolution;
    let cell_radius = res * std::f64::consts::FRAC_1_SQRT_2;
    let window = a.hypot(b) + reach + res;
    let column = |v: f64| ((v - map.origin_x) / res).floor() as i64;
    let row = |v: f64| ((v - map.origin_y) / res).floor() as i64;
    let mut best: Option<f64> = None;
    for r in row(pose.y - window).max(0)..=row(pose.y + window).min(map.height as i64 - 1) {
        for c in column(pose.x - window).max(0)..=column(pose.x + window).min(map.width as i64 - 1)
        {
            if map.occupancy[r as usize * map.width as usize + c as usize] < OCCUPIED {
                continue;
            }
            let dx = map.origin_x + (c as f64 + 0.5) * res - pose.x;
            let dy = map.origin_y + (r as f64 + 0.5) * res - pose.y;
            let along = (dx * cos + dy * sin).abs() - a;
            let across = (-dx * sin + dy * cos).abs() - b;
            let outside = along.max(0.).hypot(across.max(0.));
            let distance = (outside - cell_radius).max(0.);
            if distance <= reach && best.is_none_or(|d| distance < d) {
                best = Some(distance);
            }
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;

    fn map_with(cells: &[(usize, usize, i8)]) -> MapSnapshot {
        let mut map = MapSnapshot {
            width: 40,
            height: 40,
            resolution: 0.1,
            origin_x: -2.,
            origin_y: -2.,
            occupancy: vec![0; 1600],
        };
        for &(c, r, v) in cells {
            map.occupancy[r * 40 + c] = v;
        }
        map
    }

    #[test]
    fn a_square_body_needs_more_room_turned_than_square_on() {
        // Obstacle 0.4 m to the right of centre: clear of the 0.71 m square
        // (half 0.355 m) square-on, but hit by its corner at 45 degrees.
        let map = map_with(&[(24, 20, 100), (24, 19, 100)]);
        let body = Footprint::default();
        let square = Pose {
            x: 0.,
            y: 0.,
            yaw: 0.,
        };
        let turned = Pose {
            x: 0.,
            y: 0.,
            yaw: std::f64::consts::FRAC_PI_4,
        };
        assert!(footprint_fits(&map, &body, square, None));
        assert_eq!(
            footprint_rejection_at(&map, &body, turned, 0., None),
            Some("occupied")
        );
    }

    #[test]
    fn unknown_cells_are_allowed_only_under_the_starting_body() {
        let map = map_with(&[(20, 20, -1)]);
        let body = Footprint::default();
        let here = Pose {
            x: 0.,
            y: 0.,
            yaw: 0.,
        };
        assert_eq!(
            footprint_rejection_at(&map, &body, here, 0., None),
            Some("unknown")
        );
        assert!(footprint_fits(&map, &body, here, Some(here)));
        let elsewhere = Pose {
            x: 1.5,
            y: 1.5,
            yaw: 0.,
        };
        assert_eq!(
            footprint_rejection_at(&map, &body, here, 0., Some(elsewhere)),
            Some("unknown")
        );
    }

    #[test]
    fn leaving_the_map_is_rejected() {
        let map = map_with(&[]);
        let near_edge = Pose {
            x: -1.8,
            y: 0.,
            yaw: 0.,
        };
        assert_eq!(
            footprint_rejection_at(&map, &Footprint::default(), near_edge, 0., None),
            Some("outside_map")
        );
    }

    #[test]
    fn clearance_never_overstates_the_gap() {
        // Wall cell whose near edge is 0.6 m ahead of centre: 0.6 - 0.355 = 0.245 m.
        let map = map_with(&[(26, 20, 100)]);
        let clearance = rectangle_clearance(
            &map,
            &Footprint::default(),
            Pose {
                x: 0.,
                y: 0.,
                yaw: 0.,
            },
            2.,
        )
        .unwrap();
        assert!(
            clearance <= 0.245 + 1e-9 && clearance > 0.245 - 0.08,
            "{clearance}"
        );
    }
}
