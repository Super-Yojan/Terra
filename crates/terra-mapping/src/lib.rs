//! Rolling, world-aligned XY occupancy grid. World +Z is up.
use terra_types::{Quaternion, Vector3};

#[derive(Clone, Copy, Debug)]
pub struct MapConfig {
    pub width: u32,
    pub height: u32,
    pub resolution: f64,
    pub min_obstacle_height: f64,
    pub max_obstacle_height: f64,
    pub max_range: f64,
    pub pixel_stride: u32,
}
impl Default for MapConfig {
    fn default() -> Self {
        Self {
            width: 200,
            height: 200,
            resolution: 0.1,
            min_obstacle_height: 0.15,
            max_obstacle_height: 2.0,
            max_range: 15.0,
            pixel_stride: 4,
        }
    }
}
#[derive(Clone, Copy, Debug)]
pub struct CameraIntrinsics {
    pub width: u32,
    pub height: u32,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
}
/// Camera optical axes: X right, Y down, Z forward. Pose rotates these into world axes.
#[derive(Clone, Copy, Debug)]
pub struct CameraPose {
    pub position: Vector3,
    pub orientation: Quaternion,
}
#[derive(Clone, Debug)]
pub struct MapSnapshot {
    pub width: u32,
    pub height: u32,
    pub resolution: f64,
    pub origin_x: f64,
    pub origin_y: f64,
    /// Row-major; rows increase in world +Y. -1 unknown, 0..100 occupancy probability.
    pub occupancy: Vec<i8>,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MapError {
    InvalidConfig,
    InvalidPose,
    InvalidFrame,
    InvalidTime,
}
impl std::fmt::Display for MapError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for MapError {}
pub struct LocalOccupancyMap {
    config: MapConfig,
    origin_x: f64,
    origin_y: f64,
    evidence: Vec<f32>,
    observed: Vec<bool>,
    last_timestamp: Option<f64>,
}
impl LocalOccupancyMap {
    pub fn new(config: MapConfig) -> Result<Self, MapError> {
        if config.width == 0
            || config.height == 0
            || u64::from(config.width) * u64::from(config.height) > 1_000_000
            || !config.resolution.is_finite()
            || config.resolution < 0.01
            || config.resolution > 10.0
            || !config.min_obstacle_height.is_finite()
            || !config.max_obstacle_height.is_finite()
            || config.min_obstacle_height >= config.max_obstacle_height
            || !config.max_range.is_finite()
            || config.max_range <= 0.0
            || config.max_range > 100.0
            || config.pixel_stride == 0
        {
            return Err(MapError::InvalidConfig);
        }
        let n = (config.width * config.height) as usize;
        Ok(Self {
            config,
            origin_x: -(config.width as f64) * config.resolution / 2.0,
            origin_y: -(config.height as f64) * config.resolution / 2.0,
            evidence: vec![0.0; n],
            observed: vec![false; n],
            last_timestamp: None,
        })
    }
    pub fn clear(&mut self) {
        self.evidence.fill(0.0);
        self.observed.fill(false);
        self.last_timestamp = None;
    }
    /// Shift by whole cells, preserving overlapping world cells. Newly exposed cells are unknown.
    pub fn recenter(&mut self, x: f64, y: f64) -> Result<(), MapError> {
        if !x.is_finite() || !y.is_finite() || x.abs() > 1e9 || y.abs() > 1e9 {
            return Err(MapError::InvalidPose);
        }
        let r = self.config.resolution;
        let dx = ((x - self.config.width as f64 * r / 2.0 - self.origin_x) / r).round() as i64;
        let dy = ((y - self.config.height as f64 * r / 2.0 - self.origin_y) / r).round() as i64;
        if dx == 0 && dy == 0 {
            return Ok(());
        }
        let mut evidence = vec![0.0; self.evidence.len()];
        let mut observed = vec![false; self.observed.len()];
        for row in 0..self.config.height as i64 {
            for col in 0..self.config.width as i64 {
                if let Some(old) = self.index(col + dx, row + dy) {
                    let new = (row * self.config.width as i64 + col) as usize;
                    evidence[new] = self.evidence[old];
                    observed[new] = self.observed[old];
                }
            }
        }
        self.origin_x += dx as f64 * r;
        self.origin_y += dy as f64 * r;
        self.evidence = evidence;
        self.observed = observed;
        Ok(())
    }
    fn index(&self, x: i64, y: i64) -> Option<usize> {
        (x >= 0 && y >= 0 && x < self.config.width as i64 && y < self.config.height as i64)
            .then_some((y * self.config.width as i64 + x) as usize)
    }
    fn cell(&self, p: Vector3) -> (i64, i64) {
        (
            ((p.x - self.origin_x) / self.config.resolution).floor() as i64,
            ((p.y - self.origin_y) / self.config.resolution).floor() as i64,
        )
    }
    /// Axial depth in metres, top-left row-major. Invalid/no-return values never clear space.
    /// Pose must correspond to exposure time. Heights are relative to the caller's world ground plane.
    pub fn integrate_depth(
        &mut self,
        timestamp: f64,
        intrinsics: CameraIntrinsics,
        pose: CameraPose,
        depth: &[f32],
    ) -> Result<(), MapError> {
        if !timestamp.is_finite()
            || timestamp < 0.0
            || self.last_timestamp.is_some_and(|t| timestamp <= t)
        {
            return Err(MapError::InvalidTime);
        }
        let k = intrinsics;
        if k.width == 0
            || k.height == 0
            || u64::from(k.width) * u64::from(k.height) > 4_194_304
            || depth.len() != k.width as usize * k.height as usize
            || ![k.fx, k.fy, k.cx, k.cy].iter().all(|v| v.is_finite())
            || k.cx.abs() > 1e6
            || k.cy.abs() > 1e6
            || k.fx < 1e-6
            || k.fy < 1e-6
        {
            return Err(MapError::InvalidFrame);
        }
        if !pose.position.finite()
            || pose.position.x.abs() > 1e9
            || pose.position.y.abs() > 1e9
            || pose.position.z.abs() > 1e9
        {
            return Err(MapError::InvalidPose);
        }
        let q = pose.orientation.normalized().ok_or(MapError::InvalidPose)?;
        let start = self.cell(pose.position);
        if self.index(start.0, start.1).is_none() {
            return Err(MapError::InvalidPose);
        }
        // Deduplicate per frame so dense pixels cannot instantly saturate a cell.
        // Obstacle evidence wins over free evidence from another ray in the same frame.
        let mut updates = vec![0_i8; self.evidence.len()];
        for v in (0..k.height).step_by(self.config.pixel_stride as usize) {
            for u in (0..k.width).step_by(self.config.pixel_stride as usize) {
                let z = depth[(v * k.width + u) as usize] as f64;
                if !z.is_finite() || z <= 0.0 {
                    continue;
                }
                let optical = Vector3 {
                    x: (u as f64 - k.cx) * z / k.fx,
                    y: (v as f64 - k.cy) * z / k.fy,
                    z,
                };
                let range = optical.x.hypot(optical.y).hypot(optical.z);
                if range > self.config.max_range {
                    continue;
                }
                let endpoint = pose.position.plus(q.rotate(optical));
                // High overhead objects provide no ground-level free-space evidence.
                if endpoint.z > self.config.max_obstacle_height {
                    continue;
                }
                let hit = endpoint.z >= self.config.min_obstacle_height;
                let end = self.cell(endpoint);
                // Bound traversal by the local grid: never iterate to a distant off-grid endpoint.
                let (mut x, mut y) = start;
                let dx = (end.0 - x).abs();
                let dy = -(end.1 - y).abs();
                let sx = if x < end.0 { 1 } else { -1 };
                let sy = if y < end.1 { 1 } else { -1 };
                let mut err = dx + dy;
                while let Some(i) = self.index(x, y) {
                    if (x, y) == end {
                        if hit {
                            updates[i] = 1;
                        } else if updates[i] == 0 {
                            updates[i] = -1;
                        }
                        break;
                    }
                    if updates[i] == 0 {
                        updates[i] = -1;
                    }
                    let e2 = 2 * err;
                    if e2 >= dy {
                        err += dy;
                        x += sx;
                    }
                    if e2 <= dx {
                        err += dx;
                        y += sy;
                    }
                }
            }
        }
        for (i, update) in updates.into_iter().enumerate() {
            if update != 0 {
                self.observed[i] = true;
                self.evidence[i] =
                    (self.evidence[i] + if update > 0 { 0.85 } else { -0.4 }).clamp(-4.0, 4.0);
            }
        }
        self.last_timestamp = Some(timestamp);
        Ok(())
    }
    pub fn snapshot(&self) -> MapSnapshot {
        MapSnapshot {
            width: self.config.width,
            height: self.config.height,
            resolution: self.config.resolution,
            origin_x: self.origin_x,
            origin_y: self.origin_y,
            occupancy: self
                .evidence
                .iter()
                .zip(&self.observed)
                .map(|(l, seen)| {
                    if *seen {
                        (100.0 / (1.0 + (-l).exp())).round() as i8
                    } else {
                        -1
                    }
                })
                .collect(),
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    fn setup() -> (LocalOccupancyMap, CameraIntrinsics, CameraPose) {
        let map = LocalOccupancyMap::new(MapConfig {
            width: 20,
            height: 20,
            resolution: 0.5,
            pixel_stride: 1,
            ..Default::default()
        })
        .unwrap();
        let k = CameraIntrinsics {
            width: 1,
            height: 1,
            fx: 1.0,
            fy: 1.0,
            cx: 0.0,
            cy: 0.0,
        };
        // Optical forward +Z -> world +X.
        let pose = CameraPose {
            position: Vector3 {
                x: 0.25,
                y: 0.25,
                z: 0.5,
            },
            orientation: Quaternion::from_rotation_vector(Vector3 {
                y: std::f64::consts::FRAC_PI_2,
                ..Vector3::ZERO
            }),
        };
        (map, k, pose)
    }
    #[test]
    fn wall_marks_ray_free_endpoint_occupied_and_space_behind_unknown() {
        let (mut map, k, p) = setup();
        for t in 0..4 {
            map.integrate_depth(t as f64, k, p, &[2.0]).unwrap();
        }
        let s = map.snapshot();
        assert!(s.occupancy[10 * 20 + 12] < 50);
        assert!(s.occupancy[10 * 20 + 14] > 90);
        assert_eq!(s.occupancy[10 * 20 + 15], -1);
    }
    #[test]
    fn rolling_preserves_world_evidence_and_exposes_unknown_cells() {
        let (mut map, k, p) = setup();
        map.integrate_depth(0.0, k, p, &[2.0]).unwrap();
        let before = map.snapshot().occupancy[10 * 20 + 14];
        map.recenter(1.0, 0.0).unwrap();
        assert_eq!(map.snapshot().occupancy[10 * 20 + 12], before);
        assert_eq!(map.snapshot().occupancy[10 * 20 + 19], -1);
        map.recenter(100.0, 100.0).unwrap();
        assert!(map.snapshot().occupancy.iter().all(|x| *x == -1));
    }
    #[test]
    fn invalid_depth_and_inputs_do_not_clear_unknown_space() {
        let (mut map, k, p) = setup();
        map.integrate_depth(0.0, k, p, &[f32::NAN]).unwrap();
        assert!(map.snapshot().occupancy.iter().all(|x| *x == -1));
        assert_eq!(
            map.integrate_depth(0.0, k, p, &[2.0]),
            Err(MapError::InvalidTime)
        );
        assert_eq!(
            map.integrate_depth(1.0, k, p, &[]),
            Err(MapError::InvalidFrame)
        );
        map.integrate_depth(1.0, k, p, &[100.0]).unwrap();
        assert!(map.snapshot().occupancy.iter().all(|x| *x == -1));
    }
    #[test]
    fn ground_is_free_and_overhead_return_is_ignored() {
        let (mut map, k, mut p) = setup();
        p.position.z = 0.0;
        map.integrate_depth(0.0, k, p, &[2.0]).unwrap();
        assert!(map.snapshot().occupancy[10 * 20 + 14] < 50);
        map.clear();
        p.position.z = 3.0;
        map.integrate_depth(1.0, k, p, &[2.0]).unwrap();
        assert!(map.snapshot().occupancy.iter().all(|x| *x == -1));
    }
}
