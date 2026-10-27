//! Wall, window and door cells from ARKit's classified scene mesh. LiDAR
//! passes through clear glass, so a ray to whatever stands behind a pane
//! would otherwise clear the pane's cells. The mesh often still holds faces
//! on the glass, its frame or its reflections, and ARKit labels them.
use terra_types::Vector3;

/// Faces a cell needs before it counts as a barrier, so a stray face does
/// not wall off open floor.
pub const BARRIER_MIN_FACES: u32 = 3;
/// Height slack around a cell's face heights when testing a ray, in metres.
pub const BARRIER_HEIGHT_MARGIN: f64 = 0.05;
/// Largest accepted surface side, in cells.
const MAX_SIDE: u32 = 400;

/// Barrier height span per cell of a square world-aligned grid.
///
/// Built fresh from each batch of mesh samples, so it never outlives the mesh
/// that produced it by more than one update.
#[derive(Clone, Debug, PartialEq)]
pub struct BarrierSurface {
    origin_x: f64,
    origin_y: f64,
    resolution: f64,
    side: u32,
    /// Row-major lowest and highest face height in world metres; NaN where
    /// the cell holds fewer than [`BARRIER_MIN_FACES`] faces.
    spans: Vec<[f32; 2]>,
}
impl BarrierSurface {
    /// Bin wall, window and door face centres (world metres) into
    /// `resolution` cells within `half_extent` of `(centre_x, centre_y)`.
    /// Nonfinite samples and samples outside the square are ignored.
    ///
    /// Returns `None` for invalid geometry or when no cell holds
    /// [`BARRIER_MIN_FACES`] faces.
    pub fn build(
        samples: &[Vector3],
        centre_x: f64,
        centre_y: f64,
        half_extent: f64,
        resolution: f64,
    ) -> Option<Self> {
        if ![centre_x, centre_y, half_extent, resolution]
            .iter()
            .all(|v| v.is_finite())
            || !(0.01..=10.0).contains(&resolution)
            || half_extent <= 0.0
        {
            return None;
        }
        let side = (2.0 * half_extent / resolution).ceil();
        if side > MAX_SIDE as f64 {
            return None;
        }
        let side = side as u32;
        let origin_x = centre_x - side as f64 * resolution / 2.0;
        let origin_y = centre_y - side as f64 * resolution / 2.0;
        let mut counts = vec![0_u32; (side * side) as usize];
        let mut spans = vec![[f32::INFINITY, f32::NEG_INFINITY]; counts.len()];
        for p in samples {
            if !p.finite() {
                continue;
            }
            let col = ((p.x - origin_x) / resolution).floor();
            let row = ((p.y - origin_y) / resolution).floor();
            if col < 0.0 || row < 0.0 || col >= side as f64 || row >= side as f64 {
                continue;
            }
            let i = row as usize * side as usize + col as usize;
            counts[i] += 1;
            spans[i][0] = spans[i][0].min(p.z as f32);
            spans[i][1] = spans[i][1].max(p.z as f32);
        }
        let mut any = false;
        for (span, count) in spans.iter_mut().zip(&counts) {
            if *count < BARRIER_MIN_FACES {
                *span = [f32::NAN; 2];
            } else {
                any = true;
            }
        }
        any.then_some(Self {
            origin_x,
            origin_y,
            resolution,
            side,
            spans,
        })
    }
    fn span_at(&self, x: f64, y: f64) -> Option<(i64, i64, [f32; 2])> {
        let col = ((x - self.origin_x) / self.resolution).floor();
        let row = ((y - self.origin_y) / self.resolution).floor();
        if !(col >= 0.0 && row >= 0.0 && col < self.side as f64 && row < self.side as f64) {
            return None;
        }
        let span = self.spans[row as usize * self.side as usize + col as usize];
        Some((col as i64, row as i64, span))
    }
    /// First point where the segment `from`→`to` passes through a barrier
    /// cell at a height within that cell's faces (plus
    /// [`BARRIER_HEIGHT_MARGIN`]), or `None` when it passes none.
    ///
    /// The cell holding `from` is skipped, so a camera beside a wall still
    /// sees past it, and so are the last `resolution` metres before `to`, so
    /// a return off the barrier itself keeps its own label.
    pub fn crossing(&self, from: Vector3, to: Vector3) -> Option<Vector3> {
        let length = (to.x - from.x).hypot(to.y - from.y);
        let stop = length - self.resolution;
        if !stop.is_finite() || stop <= 0.0 {
            return None;
        }
        let start = self.span_at(from.x, from.y).map(|(c, r, _)| (c, r));
        let step = self.resolution / 4.0;
        let mut s = step;
        while s < stop {
            let t = s / length;
            let p = Vector3 {
                x: from.x + t * (to.x - from.x),
                y: from.y + t * (to.y - from.y),
                z: from.z + t * (to.z - from.z),
            };
            if let Some((col, row, [low, high])) = self.span_at(p.x, p.y)
                && Some((col, row)) != start
                && p.z >= low as f64 - BARRIER_HEIGHT_MARGIN
                && p.z <= high as f64 + BARRIER_HEIGHT_MARGIN
            {
                return Some(p);
            }
            s += step;
        }
        None
    }
    /// World centres of the barrier cells, at the middle of their height span.
    pub fn cell_centres(&self) -> Vec<Vector3> {
        let side = self.side as usize;
        (0..side * side)
            .filter_map(|i| {
                let [low, high] = self.spans[i];
                low.is_finite().then(|| Vector3 {
                    x: self.origin_x + ((i % side) as f64 + 0.5) * self.resolution,
                    y: self.origin_y + ((i / side) as f64 + 0.5) * self.resolution,
                    z: (low as f64 + high as f64) / 2.0,
                })
            })
            .collect()
    }
    /// Number of barrier cells.
    pub fn cells(&self) -> usize {
        self.spans.iter().filter(|s| s[0].is_finite()).count()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    /// A pane in the plane x = `x`, `n` faces per 0.1 m cell, from z 0 to 1 m.
    fn pane(x: f64, y0: f64, y1: f64, n: usize) -> Vec<Vector3> {
        let mut out = Vec::new();
        let mut y = y0 + 0.05;
        while y < y1 {
            for k in 0..n {
                out.push(Vector3 {
                    x,
                    y,
                    z: k as f64 / (n - 1).max(1) as f64,
                });
            }
            y += 0.1;
        }
        out
    }
    fn at(x: f64, y: f64, z: f64) -> Vector3 {
        Vector3 { x, y, z }
    }
    #[test]
    fn ray_through_a_pane_stops_at_it() {
        let s = BarrierSurface::build(&pane(2.05, -1.0, 1.0, 5), 0.0, 0.0, 6.0, 0.1).unwrap();
        assert_eq!(s.cells(), 20);
        let hit = s.crossing(at(0.0, 0.0, 0.5), at(4.0, 0.0, 0.5)).unwrap();
        assert!((2.0..2.1).contains(&hit.x), "{hit:?}");
    }
    #[test]
    fn return_off_the_pane_and_rays_beside_it_pass() {
        let s = BarrierSurface::build(&pane(2.05, -1.0, 1.0, 5), 0.0, 0.0, 6.0, 0.1).unwrap();
        assert!(s.crossing(at(0.0, 0.0, 0.5), at(2.05, 0.0, 0.5)).is_none());
        assert!(s.crossing(at(0.0, 0.0, 0.5), at(4.0, 3.0, 0.5)).is_none());
    }
    #[test]
    fn ray_above_a_low_barrier_passes() {
        let s = BarrierSurface::build(&pane(2.05, -1.0, 1.0, 5), 0.0, 0.0, 6.0, 0.1).unwrap();
        assert!(s.crossing(at(0.0, 0.0, 1.5), at(4.0, 0.0, 1.5)).is_none());
    }
    #[test]
    fn camera_cell_and_sparse_faces_do_not_block() {
        let s = BarrierSurface::build(&pane(0.05, -1.0, 1.0, 5), 0.0, 0.0, 6.0, 0.1).unwrap();
        assert!(
            s.crossing(at(0.02, 0.02, 0.5), at(4.0, 0.02, 0.5))
                .is_none()
        );
        assert!(BarrierSurface::build(&pane(2.05, -1.0, 1.0, 2), 0.0, 0.0, 6.0, 0.1).is_none());
    }
    #[test]
    fn rejects_invalid_geometry() {
        let samples = pane(2.05, -1.0, 1.0, 5);
        assert!(BarrierSurface::build(&samples, f64::NAN, 0.0, 6.0, 0.1).is_none());
        assert!(BarrierSurface::build(&samples, 0.0, 0.0, 0.0, 0.1).is_none());
        assert!(BarrierSurface::build(&samples, 0.0, 0.0, 100.0, 0.1).is_none());
    }
}
