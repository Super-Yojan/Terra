//! Floor heights from ARKit's classified scene mesh, used as a second opinion
//! on obstacle returns. Geometry stays the source of truth: the surface only
//! turns a would-be obstacle that sits on reported floor into a floor return.
use terra_types::Vector3;

/// Cells farther than this from the surface's median height are not trusted
/// as floor, in metres. A box top mislabelled as floor fails this test.
pub const FLOOR_OUTLIER: f64 = 0.08;
/// Floor samples must lie at least this far below the camera, in metres.
pub const FLOOR_BELOW_CAMERA: f64 = 0.10;
/// Largest accepted surface side, in cells.
const MAX_SIDE: u32 = 400;

/// Trusted floor height per cell of a square world-aligned grid.
///
/// Built fresh from each batch of floor samples, so it never outlives the
/// mesh that produced it by more than one update.
#[derive(Clone, Debug, PartialEq)]
pub struct FloorSurface {
    origin_x: f64,
    origin_y: f64,
    resolution: f64,
    side: u32,
    /// Row-major heights in world metres; NaN where no trusted floor was seen.
    heights: Vec<f32>,
}
impl FloorSurface {
    /// Bin floor samples (world metres) into `resolution` cells within
    /// `half_extent` of `(centre_x, centre_y)`. Each cell keeps its median
    /// sample height; cells more than [`FLOOR_OUTLIER`] from the median of all
    /// cells are dropped. Samples that are nonfinite, outside the square, or
    /// less than [`FLOOR_BELOW_CAMERA`] below `camera_z` are ignored.
    ///
    /// Returns `None` for invalid geometry or when no cell survives.
    pub fn build(
        samples: &[Vector3],
        centre_x: f64,
        centre_y: f64,
        camera_z: f64,
        half_extent: f64,
        resolution: f64,
    ) -> Option<Self> {
        if ![centre_x, centre_y, camera_z, half_extent, resolution]
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
        let mut cells: Vec<Vec<f32>> = vec![Vec::new(); (side * side) as usize];
        let ceiling = camera_z - FLOOR_BELOW_CAMERA;
        for p in samples {
            if !p.finite() || p.z > ceiling {
                continue;
            }
            let col = ((p.x - origin_x) / resolution).floor();
            let row = ((p.y - origin_y) / resolution).floor();
            if col < 0.0 || row < 0.0 || col >= side as f64 || row >= side as f64 {
                continue;
            }
            cells[row as usize * side as usize + col as usize].push(p.z as f32);
        }
        let mut heights: Vec<f32> = cells
            .iter_mut()
            .map(|c| median(c).unwrap_or(f32::NAN))
            .collect();
        let mut seen: Vec<f32> = heights.iter().copied().filter(|h| h.is_finite()).collect();
        let centre = median(&mut seen)? as f64;
        for h in &mut heights {
            if (*h as f64 - centre).abs() > FLOOR_OUTLIER {
                *h = f32::NAN;
            }
        }
        Some(Self {
            origin_x,
            origin_y,
            resolution,
            side,
            heights,
        })
    }
    /// Trusted floor height of the cell containing world `(x, y)`.
    pub fn height_at(&self, x: f64, y: f64) -> Option<f64> {
        let col = ((x - self.origin_x) / self.resolution).floor();
        let row = ((y - self.origin_y) / self.resolution).floor();
        if !(col >= 0.0 && row >= 0.0 && col < self.side as f64 && row < self.side as f64) {
            return None;
        }
        let h = self.heights[row as usize * self.side as usize + col as usize];
        h.is_finite().then_some(h as f64)
    }
    /// Median trusted floor height within `radius` of `(x, y)` and the area
    /// those cells cover in square metres.
    pub fn estimate(&self, x: f64, y: f64, radius: f64) -> Option<(f64, f64)> {
        if !x.is_finite() || !y.is_finite() || !radius.is_finite() || radius <= 0.0 {
            return None;
        }
        let mut near = Vec::new();
        for row in 0..self.side as usize {
            let cy = self.origin_y + (row as f64 + 0.5) * self.resolution;
            for col in 0..self.side as usize {
                let cx = self.origin_x + (col as f64 + 0.5) * self.resolution;
                let h = self.heights[row * self.side as usize + col];
                if h.is_finite() && (cx - x).hypot(cy - y) <= radius {
                    near.push(h);
                }
            }
        }
        let area = near.len() as f64 * self.resolution * self.resolution;
        median(&mut near).map(|h| (h as f64, area))
    }
    /// World centres of the trusted cells, at their floor height.
    pub fn trusted_centres(&self) -> Vec<Vector3> {
        let side = self.side as usize;
        (0..side * side)
            .filter_map(|i| {
                let h = self.heights[i];
                h.is_finite().then(|| Vector3 {
                    x: self.origin_x + ((i % side) as f64 + 0.5) * self.resolution,
                    y: self.origin_y + ((i / side) as f64 + 0.5) * self.resolution,
                    z: h as f64,
                })
            })
            .collect()
    }
    /// Number of cells holding trusted floor.
    pub fn trusted_cells(&self) -> usize {
        self.heights.iter().filter(|h| h.is_finite()).count()
    }
}
fn median(values: &mut [f32]) -> Option<f32> {
    if values.is_empty() {
        return None;
    }
    let mid = values.len() / 2;
    let (_, m, _) = values.select_nth_unstable_by(mid, f32::total_cmp);
    Some(*m)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn patch(x0: f64, y0: f64, n: usize, z: f64) -> Vec<Vector3> {
        (0..n * n)
            .map(|i| Vector3 {
                x: x0 + (i % n) as f64 * 0.1 + 0.05,
                y: y0 + (i / n) as f64 * 0.1 + 0.05,
                z,
            })
            .collect()
    }
    #[test]
    fn keeps_floor_and_drops_raised_patches() {
        let mut samples = patch(-1.0, -1.0, 20, 0.0);
        samples.extend(patch(2.0, 2.0, 3, 0.3));
        let s = FloorSurface::build(&samples, 0.0, 0.0, 0.5, 6.0, 0.1).unwrap();
        assert_eq!(s.height_at(0.05, 0.05), Some(0.0));
        assert_eq!(s.height_at(2.15, 2.15), None);
        assert_eq!(s.height_at(5.0, 5.0), None);
        assert_eq!(s.trusted_cells(), 400);
        let centres = s.trusted_centres();
        assert_eq!(centres.len(), 400);
        assert!(
            centres
                .iter()
                .all(|c| c.z == 0.0 && (-1.0..1.0).contains(&c.x))
        );
    }
    #[test]
    fn ignores_samples_near_or_above_the_camera() {
        let samples = patch(0.0, 0.0, 4, 0.45);
        assert!(FloorSurface::build(&samples, 0.0, 0.0, 0.5, 6.0, 0.1).is_none());
    }
    #[test]
    fn estimate_reports_median_height_and_area() {
        let samples = patch(-1.0, -1.0, 20, -0.2);
        let s = FloorSurface::build(&samples, 0.0, 0.0, 0.0, 6.0, 0.1).unwrap();
        let (h, area) = s.estimate(0.0, 0.0, 0.5).unwrap();
        assert!((h + 0.2).abs() < 1e-6);
        assert!(area > 0.7 && area < 0.85, "{area}");
        assert!(s.estimate(10.0, 10.0, 0.5).is_none());
    }
    #[test]
    fn rejects_invalid_geometry() {
        let samples = patch(0.0, 0.0, 2, 0.0);
        assert!(FloorSurface::build(&samples, f64::NAN, 0.0, 1.0, 6.0, 0.1).is_none());
        assert!(FloorSurface::build(&samples, 0.0, 0.0, 1.0, 0.0, 0.1).is_none());
        assert!(FloorSurface::build(&samples, 0.0, 0.0, 1.0, 100.0, 0.1).is_none());
    }
}
