//! The rover body as an oriented rectangle, shared by every planner.
//!
//! A cell blocks the body when the rectangle at the body's heading overlaps it:
//! occupied cells (occupancy at least [`OCCUPIED`]) and off-map cells always,
//! unknown cells (occupancy below 0) unless they also lie under the body at the
//! starting pose, so the rover can drive off its own unobserved underbody.
use crate::{Pose, valid_map};
use serde::{Deserialize, Serialize};
use std::f64::consts::SQRT_2;
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
    /// The camera's blind zone around the start: unknown cells whose centres lie
    /// within this distance of the starting position are tolerated, because the
    /// phone cannot see the ground right around (or under) the rover. Occupied
    /// cells there still block.
    pub fn blind_zone_radius(&self) -> f64 {
        self.circumradius() + BLIND_ZONE_MARGIN
    }
}

/// How far beyond the circle around the body the start blind zone reaches, metres.
pub const BLIND_ZONE_MARGIN: f64 = 0.25;

/// Whether cell `i`'s centre lies in the blind zone around `start`.
pub(crate) fn in_blind_zone(
    map: &MapSnapshot,
    footprint: &Footprint,
    i: usize,
    start: Pose,
) -> bool {
    let width = map.width as usize;
    let cx = map.origin_x + ((i % width) as f64 + 0.5) * map.resolution;
    let cy = map.origin_y + ((i / width) as f64 + 0.5) * map.resolution;
    (cx - start.x).hypot(cy - start.y) <= footprint.blind_zone_radius()
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
    /// Anywhere: only observed obstacles and the map edge block.
    Anywhere,
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
            Unknown::Anywhere => true,
            Unknown::UnderBody(s) => {
                cell_overlaps(map, i, s, footprint.half_extents(pad))
                    || in_blind_zone(map, footprint, i, s)
            }
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

/// Headings tried when the heading is free: a rectangle repeats every 180 degrees.
const FIT_HEADINGS: usize = 8;

/// Whether the body fits at `(x, y)` at one of 8 headings (22.5 degree steps).
/// Unknown cells under the body at `start` are allowed; with `None`, every
/// unknown cell blocks.
pub fn footprint_fits_some_heading(
    map: &MapSnapshot,
    footprint: &Footprint,
    x: f64,
    y: f64,
    start: Option<Pose>,
) -> bool {
    let unknown = start.map_or(Unknown::Refused, Unknown::UnderBody);
    fits_some_heading(map, footprint, x, y, unknown)
}

pub(crate) fn fits_some_heading(
    map: &MapSnapshot,
    footprint: &Footprint,
    x: f64,
    y: f64,
    unknown: Unknown,
) -> bool {
    (0..FIT_HEADINGS).any(|k| {
        let yaw = k as f64 * std::f64::consts::PI / FIT_HEADINGS as f64;
        rejection(map, footprint, Pose { x, y, yaw }, 0., unknown).is_none()
    })
}

/// Whether reversing straight back from `pose` by `distance` keeps the body off
/// every blocking cell (occupied, unknown or off-map), except cells it already
/// touches ahead of its centre, which reversing moves away from. Used to back
/// away from an obstacle the body already overlaps, for example map noise in
/// front of the rover.
pub fn backoff_clear(map: &MapSnapshot, footprint: &Footprint, pose: Pose, distance: f64) -> bool {
    if !valid_map(map)
        || !pose.finite()
        || !footprint.valid()
        || !distance.is_finite()
        || distance < 0.
    {
        return false;
    }
    let (a, b) = footprint.half_extents(0.);
    let (sin, cos) = pose.yaw.sin_cos();
    let width = map.width as usize;
    let mut touching = std::collections::HashSet::new();
    for_each_overlapping_cell(map, pose.x, pose.y, pose.yaw, a, b, |i| {
        if let Some(i) = i {
            let cx = map.origin_x + ((i % width) as f64 + 0.5) * map.resolution;
            let cy = map.origin_y + ((i / width) as f64 + 0.5) * map.resolution;
            // Only cells ahead of the centre: reversing leaves them behind.
            if (cx - pose.x) * cos + (cy - pose.y) * sin > 0. {
                touching.insert(i);
            }
        }
        true
    });
    // Half-cell steps; growing the body by a quarter cell covers what lies between.
    let (a, b) = footprint.half_extents(map.resolution / 4.);
    let steps = (distance / (map.resolution / 2.)).ceil().max(1.) as usize;
    (1..=steps).all(|k| {
        let back = distance * k as f64 / steps as f64;
        let (x, y) = (pose.x - back * cos, pose.y - back * sin);
        for_each_overlapping_cell(map, x, y, pose.yaw, a, b, |i| match i {
            None => false,
            Some(i) => {
                let v = map.occupancy[i];
                (0..OCCUPIED).contains(&v)
                    || touching.contains(&i)
                    || v < 0 && in_blind_zone(map, footprint, i, pose)
            }
        })
    })
}

/// Per-cell answer to "does the body fit here at some heading", cached, with a
/// distance field so most cells need no rectangle test: far from every blocking
/// cell the body fits at any heading, and within its inner radius of one it
/// fits at none.
pub struct FitGrid<'a> {
    map: &'a MapSnapshot,
    footprint: Footprint,
    start: Option<Pose>,
    /// Distance (cells, 8-connected) to the nearest blocking cell or map edge.
    distance: Vec<f64>,
    /// 0 = not yet known, 1 = fits, 2 = does not fit.
    cache: Vec<u8>,
}

impl<'a> FitGrid<'a> {
    /// Unknown cells under the body at `start` count as free; with `None`, all
    /// unknown cells block.
    pub fn new(map: &'a MapSnapshot, footprint: &Footprint, start: Option<Pose>) -> Self {
        let n = map.occupancy.len();
        let mut tolerated = vec![false; n];
        if let Some(s) = start.filter(|_| valid_map(map)) {
            let (a, b) = footprint.half_extents(0.);
            for_each_overlapping_cell(map, s.x, s.y, s.yaw, a, b, |i| {
                if let Some(i) = i {
                    tolerated[i] = true;
                }
                true
            });
            for (i, t) in tolerated.iter_mut().enumerate() {
                *t |= in_blind_zone(map, footprint, i, s);
            }
        }
        let distance = if valid_map(map) {
            distance_field(map, |i, v| v >= OCCUPIED || v < 0 && !tolerated[i])
        } else {
            Vec::new()
        };
        Self {
            map,
            footprint: *footprint,
            start,
            distance,
            cache: vec![0; n],
        }
    }

    /// Whether the body fits centred on cell `(column, row)` at some heading.
    pub fn fits(&mut self, column: usize, row: usize) -> bool {
        let width = self.map.width as usize;
        if self.distance.is_empty() || column >= width || row >= self.map.height as usize {
            return false;
        }
        let i = row * width + column;
        if self.cache[i] != 0 {
            return self.cache[i] == 1;
        }
        let res = self.map.resolution;
        let d = self.distance[i];
        let half_diagonal = std::f64::consts::FRAC_1_SQRT_2;
        // Straight-line distance between cell centres lies in [d / 1.0824, d].
        let fits = if (d / 1.082_392_2 - 2. * half_diagonal) * res > self.footprint.circumradius() {
            true
        } else if (d + half_diagonal) * res < self.footprint.inradius() {
            false
        } else {
            let x = self.map.origin_x + (column as f64 + 0.5) * res;
            let y = self.map.origin_y + (row as f64 + 0.5) * res;
            footprint_fits_some_heading(self.map, &self.footprint, x, y, self.start)
        };
        self.cache[i] = if fits { 1 } else { 2 };
        fits
    }
}

/// 8-connected distance, in cells, from each cell to the nearest cell for which
/// `blocked(index, occupancy)` holds, or to the map edge. It is at least the
/// straight-line distance between cell centres.
pub(crate) fn distance_field(map: &MapSnapshot, blocked: impl Fn(usize, i8) -> bool) -> Vec<f64> {
    // Two-pass chamfer transform: one sweep down-right and one up-left with the
    // 3x3 mask (1 and sqrt 2) gives the exact 8-connected distance.
    let (w, h) = (map.width as usize, map.height as usize);
    let mut field: Vec<f64> = (0..w * h)
        .map(|i| {
            let (c, r) = (i % w, i / w);
            if blocked(i, map.occupancy[i]) {
                0.
            } else {
                (c + 1).min(r + 1).min(w - c).min(h - r) as f64
            }
        })
        .collect();
    let relax = |field: &mut Vec<f64>, c: usize, r: usize, mask: &[(i64, i64, f64)]| {
        let mut best = field[r * w + c];
        for &(dc, dr, cost) in mask {
            let (nc, nr) = (c as i64 + dc, r as i64 + dr);
            if nc >= 0 && nr >= 0 && (nc as usize) < w && (nr as usize) < h {
                best = best.min(field[nr as usize * w + nc as usize] + cost);
            }
        }
        field[r * w + c] = best;
    };
    let forward = [(-1, 0, 1.), (-1, -1, SQRT_2), (0, -1, 1.), (1, -1, SQRT_2)];
    let backward = [(1, 0, 1.), (1, 1, SQRT_2), (0, 1, 1.), (-1, 1, SQRT_2)];
    for r in 0..h {
        for c in 0..w {
            relax(&mut field, c, r, &forward);
        }
    }
    for r in (0..h).rev() {
        for c in (0..w).rev() {
            relax(&mut field, c, r, &backward);
        }
    }
    field
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
    fn backing_off_leaves_cells_ahead_but_not_cells_behind() {
        let body = Footprint::default();
        let here = Pose {
            x: 0.,
            y: 0.,
            yaw: 0.,
        };
        // Touching ahead (x 0.3..0.4): reversing moves away from it.
        assert!(backoff_clear(&map_with(&[(23, 20, 100)]), &body, here, 0.3));
        // Touching behind (x -0.4..-0.3): reversing goes deeper.
        assert!(!backoff_clear(
            &map_with(&[(16, 20, 100)]),
            &body,
            here,
            0.3
        ));
        // 0.65 m behind: unknown ground there is the camera's blind zone and may
        // be entered, but an occupied cell blocks a 0.3 m back-off.
        assert!(backoff_clear(&map_with(&[(13, 20, -1)]), &body, here, 0.3));
        assert!(!backoff_clear(
            &map_with(&[(13, 20, 100)]),
            &body,
            here,
            0.3
        ));
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
