//! Rolling occupancy mapping from exposure-aligned camera depth.
//!
//! Read [`LocalOccupancyMap::integrate_depth`] for sensor integration and
//! [`LocalOccupancyMap::recenter`] for moving the window without losing overlapping
//! world evidence. [`MapSnapshot`] stores row-major cells: -1 is unknown and
//! 0..=100 is occupancy probability. World +Z is up; optical camera +Z is forward.

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
        let resolution = self.config.resolution;
        let column_shift = ((x - self.config.width as f64 * resolution / 2.0 - self.origin_x)
            / resolution)
            .round() as i64;
        let row_shift = ((y - self.config.height as f64 * resolution / 2.0 - self.origin_y)
            / resolution)
            .round() as i64;
        if column_shift == 0 && row_shift == 0 {
            return Ok(());
        }
        let mut evidence = vec![0.0; self.evidence.len()];
        let mut observed = vec![false; self.observed.len()];
        for row in 0..self.config.height as i64 {
            for col in 0..self.config.width as i64 {
                if let Some(source_index) = self.index(col + column_shift, row + row_shift) {
                    let destination_index = (row * self.config.width as i64 + col) as usize;
                    evidence[destination_index] = self.evidence[source_index];
                    observed[destination_index] = self.observed[source_index];
                }
            }
        }
        self.origin_x += column_shift as f64 * resolution;
        self.origin_y += row_shift as f64 * resolution;
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
        let camera_intrinsics = intrinsics;
        if camera_intrinsics.width == 0
            || camera_intrinsics.height == 0
            || u64::from(camera_intrinsics.width) * u64::from(camera_intrinsics.height) > 4_194_304
            || depth.len() != camera_intrinsics.width as usize * camera_intrinsics.height as usize
            || ![
                camera_intrinsics.fx,
                camera_intrinsics.fy,
                camera_intrinsics.cx,
                camera_intrinsics.cy,
            ]
            .iter()
            .all(|v| v.is_finite())
            || camera_intrinsics.cx.abs() > 1e6
            || camera_intrinsics.cy.abs() > 1e6
            || camera_intrinsics.fx < 1e-6
            || camera_intrinsics.fy < 1e-6
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
        for v in (0..camera_intrinsics.height).step_by(self.config.pixel_stride as usize) {
            for u in (0..camera_intrinsics.width).step_by(self.config.pixel_stride as usize) {
                let z = depth[(v * camera_intrinsics.width + u) as usize] as f64;
                if !z.is_finite() || z <= 0.0 {
                    continue;
                }
                let optical = Vector3 {
                    x: (u as f64 - camera_intrinsics.cx) * z / camera_intrinsics.fx,
                    y: (v as f64 - camera_intrinsics.cy) * z / camera_intrinsics.fy,
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
    /// Copy observed evidence into a row-major probability grid.
    /// The origin is the lower-left cell boundary; cell centers add half a cell.
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
        let camera_intrinsics = CameraIntrinsics {
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
        (map, camera_intrinsics, pose)
    }
    #[test]
    fn wall_marks_ray_free_endpoint_occupied_and_space_behind_unknown() {
        let (mut map, camera_intrinsics, p) = setup();
        for t in 0..4 {
            map.integrate_depth(t as f64, camera_intrinsics, p, &[2.0])
                .unwrap();
        }
        let s = map.snapshot();
        assert!(s.occupancy[10 * 20 + 12] < 50);
        assert!(s.occupancy[10 * 20 + 14] > 90);
        assert_eq!(s.occupancy[10 * 20 + 15], -1);
    }
    #[test]
    fn rolling_preserves_world_evidence_and_exposes_unknown_cells() {
        let (mut map, camera_intrinsics, p) = setup();
        map.integrate_depth(0.0, camera_intrinsics, p, &[2.0])
            .unwrap();
        let before = map.snapshot().occupancy[10 * 20 + 14];
        map.recenter(1.0, 0.0).unwrap();
        assert_eq!(map.snapshot().occupancy[10 * 20 + 12], before);
        assert_eq!(map.snapshot().occupancy[10 * 20 + 19], -1);
        map.recenter(100.0, 100.0).unwrap();
        assert!(map.snapshot().occupancy.iter().all(|x| *x == -1));
    }
    #[test]
    fn invalid_depth_and_inputs_do_not_clear_unknown_space() {
        let (mut map, camera_intrinsics, p) = setup();
        map.integrate_depth(0.0, camera_intrinsics, p, &[f32::NAN])
            .unwrap();
        assert!(map.snapshot().occupancy.iter().all(|x| *x == -1));
        assert_eq!(
            map.integrate_depth(0.0, camera_intrinsics, p, &[2.0]),
            Err(MapError::InvalidTime)
        );
        assert_eq!(
            map.integrate_depth(1.0, camera_intrinsics, p, &[]),
            Err(MapError::InvalidFrame)
        );
        map.integrate_depth(1.0, camera_intrinsics, p, &[100.0])
            .unwrap();
        assert!(map.snapshot().occupancy.iter().all(|x| *x == -1));
    }
    #[test]
    fn ground_is_free_and_overhead_return_is_ignored() {
        let (mut map, camera_intrinsics, mut p) = setup();
        p.position.z = 0.0;
        map.integrate_depth(0.0, camera_intrinsics, p, &[2.0])
            .unwrap();
        assert!(map.snapshot().occupancy[10 * 20 + 14] < 50);
        map.clear();
        p.position.z = 3.0;
        map.integrate_depth(1.0, camera_intrinsics, p, &[2.0])
            .unwrap();
        assert!(map.snapshot().occupancy.iter().all(|x| *x == -1));
    }
}
