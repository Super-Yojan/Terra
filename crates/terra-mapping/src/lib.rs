//! Rolling occupancy mapping from exposure-aligned camera depth.
//!
//! Read [`LocalOccupancyMap::integrate_depth`] for sensor integration and
//! [`LocalOccupancyMap::recenter`] for moving the window without losing overlapping
//! world evidence. [`MapSnapshot`] stores row-major cells: -1 is unknown and
//! 0..=100 is occupancy probability. World +Z is up; optical camera +Z is forward.
//!
//! # Example
//!
//! ```
//! use terra_mapping::{LocalOccupancyMap, MapConfig};
//!
//! let mut map = LocalOccupancyMap::new(MapConfig::default())?;
//! assert!(map.snapshot().occupancy.iter().all(|&cell| cell == -1));
//! map.recenter(2.0, 1.0)?;
//! // Recentring preserves dimensions and leaves new cells unobserved.
//! assert_eq!(map.snapshot().width, 200);
//! # Ok::<(), terra_mapping::MapError>(())
//! ```

use terra_types::{Quaternion, Vector3};

/// Rolling grid dimensions, depth sampling, and obstacle classification limits.
#[derive(Clone, Copy, Debug)]
pub struct MapConfig {
    /// Grid columns; defaults to 200.
    pub width: u32,
    /// Grid rows; defaults to 200.
    pub height: u32,
    /// Cell side length in metres; defaults to 0.1.
    pub resolution: f64,
    /// Minimum height above supporting ground classified as an obstacle, in metres; defaults to 0.15.
    pub min_obstacle_height: f64,
    /// Maximum accepted endpoint height above supporting ground, in metres; defaults to 2.0.
    pub max_obstacle_height: f64,
    /// Maximum Euclidean camera-to-return range in metres; defaults to 15.
    pub max_range: f64,
    /// Pixel step on each image axis; defaults to 4 and must be nonzero.
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
/// Pinhole calibration for the exact depth image supplied to integration.
#[derive(Clone, Copy, Debug)]
pub struct CameraIntrinsics {
    /// Image width in pixels.
    pub width: u32,
    /// Image height in pixels.
    pub height: u32,
    /// Horizontal focal length in pixels.
    pub fx: f64,
    /// Vertical focal length in pixels.
    pub fy: f64,
    /// Principal point column in pixels, measured from the top-left image origin.
    pub cx: f64,
    /// Principal point row in pixels, measured from the top-left image origin.
    pub cy: f64,
}
/// Camera optical axes: X right, Y down, Z forward. Pose rotates these into world axes.
#[derive(Clone, Copy, Debug)]
pub struct CameraPose {
    /// Camera optical origin in world metres at image exposure time.
    pub position: Vector3,
    /// Optical-to-world rotation, normalized during integration.
    pub orientation: Quaternion,
}
/// Supporting terrain plane in the same fixed world frame as the depth pose.
/// Signed height is `(normal · point - offset) / |normal|`, in metres.
/// This affects height classification only; occupancy cells retain world x/y.
#[derive(Clone, Copy, Debug)]
pub struct GroundPlane {
    /// Upward plane normal; need not be unit length, but must be finite and nonzero.
    pub normal: Vector3,
    /// Plane equation offset, scaled consistently with `normal`.
    pub offset: f64,
}
impl Default for GroundPlane {
    fn default() -> Self {
        Self {
            normal: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
            offset: 0.0,
        }
    }
}
/// Horizontal slice of the depth view, used when no ground estimate exists.
/// Rays within `half_angle` of world horizontal are kept and every return in
/// the slice is an obstacle, so no floor is needed to classify it.
#[derive(Clone, Copy, Debug)]
pub struct HorizonBand {
    /// Half-height of the slice around world horizontal, in radians, within (0, π/2).
    pub half_angle: f64,
    /// Returns farther than this are ignored, in metres. Set it near where the
    /// slice's lower edge meets the floor so floor returns never become obstacles.
    pub max_range: f64,
}
/// Owned occupancy grid suitable for consumers that do not need mapping internals.
#[derive(Clone, Debug)]
pub struct MapSnapshot {
    /// Number of columns.
    pub width: u32,
    /// Number of rows.
    pub height: u32,
    /// Square cell side length in metres.
    pub resolution: f64,
    /// World x coordinate of the lower-left cell boundary, in metres.
    pub origin_x: f64,
    /// World y coordinate of the lower-left cell boundary, in metres.
    pub origin_y: f64,
    /// Row-major; rows increase in world +Y. -1 unknown, 0..100 occupancy probability.
    pub occupancy: Vec<i8>,
}
/// Invalid mapping input; rejected frames do not update evidence or the last timestamp.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MapError {
    /// Grid geometry, sampling stride, height interval, or range is unsupported.
    InvalidConfig,
    /// Camera pose is nonfinite, out of range, outside the grid, or not normalizable.
    InvalidPose,
    /// Depth storage or camera calibration does not describe a supported image.
    InvalidFrame,
    /// Exposure time is nonfinite, negative, or not strictly increasing.
    InvalidTime,
    /// Ground plane is nonfinite, degenerate, or tilted more than 60 degrees from world up.
    InvalidGroundPlane,
    /// Horizon band half-angle is outside (0, π/2) or its range is not positive and finite.
    InvalidBand,
}
impl std::fmt::Display for MapError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for MapError {}
/// Bounded occupancy evidence accumulated from depth rays in a rolling world grid.
/// Each accepted frame updates a cell at most once, with occupied evidence winning
/// over free evidence when rays disagree within that frame.
pub struct LocalOccupancyMap {
    config: MapConfig,
    origin_x: f64,
    origin_y: f64,
    evidence: Vec<f32>,
    observed: Vec<bool>,
    last_timestamp: Option<f64>,
}
impl LocalOccupancyMap {
    /// Allocate an unknown grid centered on world `(0, 0)`.
    ///
    /// # Errors
    ///
    /// Returns [`MapError::InvalidConfig`] for zero dimensions, more than 1,000,000
    /// cells, resolution outside `0.01..=10.0` m, nonfinite or unordered obstacle
    /// heights, range outside `(0, 100]` m, or zero pixel stride.
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
    /// Erase observations and timestamp history while retaining geometry and origin.
    pub fn clear(&mut self) {
        self.evidence.fill(0.0);
        self.observed.fill(false);
        self.last_timestamp = None;
    }
    /// Shift by whole cells, preserving overlapping world cells. Newly exposed cells are unknown.
    ///
    /// The requested center `(x, y)` is in world metres; shifts round to the
    /// nearest cell. A shift beyond the window discards all old evidence.
    /// Timestamp history is retained.
    ///
    /// # Errors
    ///
    /// Returns [`MapError::InvalidPose`] for nonfinite coordinates or magnitude above 1e9 m.
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
    ///
    /// Pose must correspond to exposure time. Heights are relative to the caller's world ground plane.
    /// Each sampled valid return clears its ray up to the endpoint. Endpoints in
    /// the configured obstacle-height interval add occupied evidence; lower
    /// returns add free evidence. Returns above the height limit or range limit
    /// are ignored. Frames with no usable returns still advance timestamp history.
    ///
    /// # Errors
    ///
    /// - [`MapError::InvalidTime`]: nonfinite, negative, or nonincreasing timestamp.
    /// - [`MapError::InvalidFrame`]: zero dimensions, more than 4,194,304 pixels,
    ///   mismatched storage, nonfinite calibration, focal length below `1e-6`,
    ///   or principal point magnitude above `1e6` pixels.
    /// - [`MapError::InvalidPose`]: invalid orientation or camera position; position
    ///   components must be finite, within ±1e9 m, and its x/y cell must lie in the grid.
    pub fn integrate_depth(
        &mut self,
        timestamp: f64,
        intrinsics: CameraIntrinsics,
        pose: CameraPose,
        depth: &[f32],
    ) -> Result<(), MapError> {
        self.integrate_depth_with_ground_plane(
            timestamp,
            intrinsics,
            pose,
            depth,
            GroundPlane::default(),
        )
    }
    /// Integrate depth using signed height above a local supporting terrain plane.
    ///
    /// All ray endpoints and occupancy cells stay in the fixed world frame. Only
    /// the obstacle-height interval is measured along the plane normal, allowing
    /// elevated or sloped ground to remain free while objects above it stay occupied.
    /// The caller must supply an exposure-aligned ground estimate; this method does
    /// not infer traversability from arbitrary depth surfaces.
    ///
    /// # Errors
    ///
    /// Returns the same errors as [`Self::integrate_depth`], plus
    /// [`MapError::InvalidGroundPlane`] for nonfinite/degenerate planes or normals
    /// tilted more than 60 degrees from world up. Rejection leaves the map unchanged.
    pub fn integrate_depth_with_ground_plane(
        &mut self,
        timestamp: f64,
        intrinsics: CameraIntrinsics,
        pose: CameraPose,
        depth: &[f32],
        ground: GroundPlane,
    ) -> Result<(), MapError> {
        let norm = ground
            .normal
            .x
            .hypot(ground.normal.y)
            .hypot(ground.normal.z);
        if !ground.normal.finite()
            || !norm.is_finite()
            || norm < 1e-12
            || ground.normal.z / norm < 0.5
            || !(ground.offset / norm).is_finite()
        {
            return Err(MapError::InvalidGroundPlane);
        }
        let normal = ground.normal.scaled(1.0 / norm);
        let ground_offset = ground.offset / norm;
        let (min_height, max_height) = (
            self.config.min_obstacle_height,
            self.config.max_obstacle_height,
        );
        self.integrate_rays(timestamp, intrinsics, pose, depth, |endpoint, _, _| {
            let height = normal.x * endpoint.x + normal.y * endpoint.y + normal.z * endpoint.z
                - ground_offset;
            // High overhead objects provide no ground-level free-space evidence.
            (height <= max_height).then_some(height >= min_height)
        })
    }
    /// Integrate depth without a ground estimate, keeping only rays within a
    /// horizontal band around the camera's level line.
    ///
    /// Every valid return in the band within `band.max_range` adds occupied
    /// evidence and clears its approach ray. Rays outside the band, and farther
    /// returns, are ignored. Obstacles entirely above or below the band are not seen.
    ///
    /// # Errors
    ///
    /// Returns the same errors as [`Self::integrate_depth`], plus
    /// [`MapError::InvalidBand`] for an invalid band. Rejection leaves the map unchanged.
    pub fn integrate_depth_in_horizon_band(
        &mut self,
        timestamp: f64,
        intrinsics: CameraIntrinsics,
        pose: CameraPose,
        depth: &[f32],
        band: HorizonBand,
    ) -> Result<(), MapError> {
        if !band.half_angle.is_finite()
            || band.half_angle <= 0.0
            || band.half_angle >= std::f64::consts::FRAC_PI_2
            || !band.max_range.is_finite()
            || band.max_range <= 0.0
        {
            return Err(MapError::InvalidBand);
        }
        let max_sin = band.half_angle.sin();
        self.integrate_rays(timestamp, intrinsics, pose, depth, |_, ray, range| {
            (range <= band.max_range && (ray.z / range).abs() <= max_sin).then_some(true)
        })
    }
    /// Validate one frame and cast its sampled rays. `classify` receives the
    /// world endpoint, world ray, and range; it returns `None` to ignore the ray,
    /// or whether the endpoint is an obstacle.
    fn integrate_rays(
        &mut self,
        timestamp: f64,
        intrinsics: CameraIntrinsics,
        pose: CameraPose,
        depth: &[f32],
        classify: impl Fn(Vector3, Vector3, f64) -> Option<bool>,
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
                let ray = q.rotate(optical);
                let endpoint = pose.position.plus(ray);
                let Some(hit) = classify(endpoint, ray, range) else {
                    continue;
                };
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
    ///
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
    /// Use a single-pixel camera looking along world +X from 0.5 m above ground.
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
    /// Repeated wall returns build occupied endpoint evidence, clear the approach
    /// ray, and leave occluded cells behind the wall unknown.
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
    /// A rolling shift preserves a wall at its world location and exposes unknown
    /// cells; moving beyond all overlap leaves an entirely unknown window.
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
    /// NaN and out-of-range returns provide no free evidence, duplicate timestamps
    /// are rejected, and depth storage must match the calibrated image dimensions.
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
    /// Ground-level returns clear their endpoints, while overhead returns must
    /// not clear the ground grid underneath them.
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

    /// An elevated 20-degree ramp is ground, while a return 30 cm above that
    /// same supporting surface remains an obstacle. World x/y must not be rotated.
    #[test]
    fn climbing_a_slope_does_not_turn_ground_into_an_obstacle() {
        let angle = 20_f64.to_radians();
        for (height_above_ground, occupied) in [(0.0, false), (0.3, true)] {
            let (mut map, intrinsics, _) = setup();
            // Camera starts 0.5 m normal to a ramp whose local elevation is 1 m.
            // Aim a ray 2 m along the ramp and down toward the selected endpoint.
            let normal = Vector3 {
                x: -angle.sin(),
                y: 0.0,
                z: angle.cos(),
            };
            let drop = 0.5 - height_above_ground;
            let ray_angle = angle - (drop / 2.0_f64).atan();
            let pose = CameraPose {
                position: Vector3 {
                    x: 0.25 + normal.x * 0.5,
                    y: 0.25,
                    z: 1.0 + normal.z * 0.5,
                },
                orientation: Quaternion::from_rotation_vector(Vector3 {
                    y: std::f64::consts::FRAC_PI_2 - ray_angle,
                    ..Vector3::ZERO
                }),
            };
            let depth = [(4.0 + drop * drop).sqrt() as f32];
            for t in 0..4 {
                map.integrate_depth_with_ground_plane(
                    t as f64,
                    intrinsics,
                    pose,
                    &depth,
                    GroundPlane {
                        normal,
                        offset: normal.x * 0.25 + normal.z * 1.0,
                    },
                )
                .unwrap();
            }
            // Both independently constructed endpoints fall in world cell (14, 10).
            let value = map.snapshot().occupancy[10 * 20 + 14];
            if occupied {
                assert!(value >= 65, "raised obstacle disappeared: {value}");
            } else {
                assert!((0..65).contains(&value), "ramp became an obstacle: {value}");
            }
        }
    }

    /// Invalid terrain estimates cannot erase evidence or consume a frame timestamp.
    #[test]
    fn invalid_ground_planes_do_not_change_the_map() {
        let (mut map, intrinsics, pose) = setup();
        for ground in [
            GroundPlane {
                normal: Vector3::ZERO,
                offset: 0.0,
            },
            GroundPlane {
                normal: Vector3 {
                    z: -1.0,
                    ..Vector3::ZERO
                },
                offset: 0.0,
            },
            GroundPlane {
                normal: Vector3 {
                    x: 1.0,
                    ..Vector3::ZERO
                },
                offset: 0.0,
            },
            GroundPlane {
                offset: f64::NAN,
                ..GroundPlane::default()
            },
        ] {
            assert_eq!(
                map.integrate_depth_with_ground_plane(0.0, intrinsics, pose, &[2.0], ground),
                Err(MapError::InvalidGroundPlane)
            );
            assert!(map.snapshot().occupancy.iter().all(|&v| v == -1));
        }
        map.integrate_depth(0.0, intrinsics, pose, &[2.0]).unwrap();
    }

    const BAND: HorizonBand = HorizonBand {
        half_angle: 10.0 * std::f64::consts::PI / 180.0,
        max_range: 3.4,
    };

    /// Without any ground estimate, a level return is an obstacle at any height
    /// and clears its approach ray, like a wall seen by a planar scanner.
    #[test]
    fn horizon_band_marks_level_return_occupied_without_ground() {
        let (mut map, intrinsics, pose) = setup();
        for t in 0..4 {
            map.integrate_depth_in_horizon_band(t as f64, intrinsics, pose, &[2.0], BAND)
                .unwrap();
        }
        let s = map.snapshot();
        assert!(s.occupancy[10 * 20 + 12] < 50);
        assert!(s.occupancy[10 * 20 + 14] > 90);
        assert_eq!(s.occupancy[10 * 20 + 15], -1);
    }

    /// Rays steeper than the band (the floor ahead) and returns past the band's
    /// range add no evidence at all.
    #[test]
    fn horizon_band_ignores_steep_rays_and_far_returns() {
        let (mut map, intrinsics, mut pose) = setup();
        pose.orientation = Quaternion::from_rotation_vector(Vector3 {
            y: std::f64::consts::FRAC_PI_2 + 0.5,
            ..Vector3::ZERO
        });
        map.integrate_depth_in_horizon_band(0.0, intrinsics, pose, &[1.0], BAND)
            .unwrap();
        assert!(map.snapshot().occupancy.iter().all(|&v| v == -1));
        let (mut map, intrinsics, pose) = setup();
        map.integrate_depth_in_horizon_band(0.0, intrinsics, pose, &[4.0], BAND)
            .unwrap();
        assert!(map.snapshot().occupancy.iter().all(|&v| v == -1));
    }

    /// Only the level rows of a tall image contribute; the top and bottom rows
    /// fall outside the band.
    #[test]
    fn horizon_band_keeps_only_rows_near_level() {
        let (mut map, _, mut pose) = setup();
        // Upright camera: optical right/down/forward -> world -Y/-Z/+X.
        pose.orientation = Quaternion {
            x: -0.5,
            y: 0.5,
            z: -0.5,
            w: 0.5,
        };
        // Three rows at about +45, 0 and -45 degrees from level.
        let intrinsics = CameraIntrinsics {
            width: 1,
            height: 3,
            fx: 1.0,
            fy: 1.0,
            cx: 0.0,
            cy: 1.0,
        };
        map.integrate_depth_in_horizon_band(0.0, intrinsics, pose, &[1.0, 2.0, 1.0], BAND)
            .unwrap();
        let s = map.snapshot();
        let observed = s.occupancy.iter().filter(|&&v| v >= 0).count();
        // Only the level row's ray (cells 10..=14 on row 10) was cast.
        assert_eq!(observed, 5, "{:?}", s.occupancy);
        assert!(s.occupancy[10 * 20 + 14] > 50);
    }

    /// Invalid bands are rejected before they can change the map or consume time.
    #[test]
    fn invalid_horizon_bands_do_not_change_the_map() {
        let (mut map, intrinsics, pose) = setup();
        for band in [
            HorizonBand {
                half_angle: 0.0,
                ..BAND
            },
            HorizonBand {
                half_angle: std::f64::consts::FRAC_PI_2,
                ..BAND
            },
            HorizonBand {
                half_angle: f64::NAN,
                ..BAND
            },
            HorizonBand {
                max_range: 0.0,
                ..BAND
            },
        ] {
            assert_eq!(
                map.integrate_depth_in_horizon_band(0.0, intrinsics, pose, &[2.0], band),
                Err(MapError::InvalidBand)
            );
            assert!(map.snapshot().occupancy.iter().all(|&v| v == -1));
        }
        map.integrate_depth_in_horizon_band(0.0, intrinsics, pose, &[2.0], BAND)
            .unwrap();
    }
}
