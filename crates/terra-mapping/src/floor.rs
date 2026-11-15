//! Floor heights from ARKit's classified scene mesh, used as a second opinion
//! on obstacle returns. Geometry stays the source of truth: the surface only
//! turns a would-be obstacle that sits on reported floor into a floor return.
use crate::square_grid::SquareGrid;
use terra_types::Vector3;

/// Cells farther than this from the surface's median height are not trusted
/// as floor, in metres. A box top mislabelled as floor fails this test.
const FLOOR_OUTLIER: f64 = 0.08;
/// Floor samples must lie at least this far below the camera, in metres.
const FLOOR_BELOW_CAMERA: f64 = 0.10;

/// Trusted floor height per cell of a square world-aligned grid.
///
/// Built fresh from each batch of floor samples, so it never outlives the
/// mesh that produced it by more than one update.
#[derive(Clone, Debug, PartialEq)]
pub struct FloorSurface {
    grid: SquareGrid,
    /// Row-major heights in world metres; NaN where no trusted floor was seen.
    heights: Vec<f32>,
}
impl FloorSurface {
    /// Bin floor samples (world metres) into `resolution` cells within
    /// `half_extent` of `(centre_x, centre_y)`. Each cell keeps its median
    /// sample height; cells more than 8 cm from the median of all cells are
    /// dropped. Samples that are nonfinite, outside the square, or less than
    /// 10 cm below `camera_z` are ignored.
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
        if !camera_z.is_finite() {
            return None;
        }
        let grid = SquareGrid::around(centre_x, centre_y, half_extent, resolution)?;
        let mut cells: Vec<Vec<f32>> = vec![Vec::new(); grid.len()];
        let ceiling = camera_z - FLOOR_BELOW_CAMERA;
        for p in samples {
            if !p.finite() || p.z > ceiling {
                continue;
            }
            if let Some(i) = grid.index(p.x, p.y) {
                cells[i].push(p.z as f32);
            }
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
        Some(Self { grid, heights })
    }
    /// Trusted floor height of the cell containing world `(x, y)`.
    pub fn height_at(&self, x: f64, y: f64) -> Option<f64> {
        let h = self.heights[self.grid.index(x, y)?];
        h.is_finite().then_some(h as f64)
    }
    /// Median trusted floor height within `radius` of `(x, y)` and the area
    /// those cells cover in square metres.
    pub fn estimate(&self, x: f64, y: f64, radius: f64) -> Option<(f64, f64)> {
        if !x.is_finite() || !y.is_finite() || !radius.is_finite() || radius <= 0.0 {
            return None;
        }
        let mut near: Vec<f32> = self
            .heights
            .iter()
            .enumerate()
            .filter(|(i, h)| {
                let (cx, cy) = self.grid.centre(*i);
                h.is_finite() && (cx - x).hypot(cy - y) <= radius
            })
            .map(|(_, h)| *h)
            .collect();
        let resolution = self.grid.resolution;
        let area = near.len() as f64 * resolution * resolution;
        median(&mut near).map(|h| (h as f64, area))
    }
    /// World centres of the trusted cells, at their floor height.
    pub fn trusted_centres(&self) -> Vec<Vector3> {
        self.heights
            .iter()
            .enumerate()
            .filter(|(_, h)| h.is_finite())
            .map(|(i, h)| {
                let (x, y) = self.grid.centre(i);
                Vector3 { x, y, z: *h as f64 }
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
