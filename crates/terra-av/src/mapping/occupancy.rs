//! World-frame occupancy grid with ARKit semantic floor veto.
//!
//! Geometry (LiDAR depth) stays the source of truth:
//! 1. Each 20 Hz LiDAR scan (`Mapper::update`) projects depth points into Terra
//!    world `W` (full 6-DoF via `frame.vio` when tracked, else `[mount]` body
//!    fallback) and classifies them by height above the ground reference.
//! 2. ARKit's classified `.floor` mesh faces (`Mapper::update_mesh`) are binned
//!    by face centroid into a trusted per-cell floor height map — no triangle
//!    rasterization needed. Per-cell median + global `8 cm` outlier rejection
//!    drops raised surfaces (like box tops) that ARKit mislabels as floor.
//! 3. **Floor veto**: if a LiDAR return would be classified as an obstacle,
//!    but lies less than `MIN_OBSTACLE_Z_M` above a trusted mesh floor cell,
//!    the floor vetoes it and treats it as free. Anything standing on top of
//!    the floor still exceeds `MIN_OBSTACLE_Z_M` and stays an obstacle.
use std::collections::{HashMap, HashSet};

use super::{Map, Mapper};
use crate::{
    estimation::{State, arkit_to_world},
    geometry::{Vec2, Vec3},
    sense::{Mount, SensorFrame},
};

/// Apple `ARMeshClassification.floor.rawValue`.
const CLASS_FLOOR: u8 = 2;

/// Minimum horizontal distance from the rover origin (m); ignores self-hits.
const MIN_RANGE_M: f64 = 0.15;
/// Maximum reliable LiDAR range (m).
const MAX_RANGE_M: f64 = 5.0;

/// Floor band relative to ground reference (m).
const MIN_FLOOR_Z_M: f64 = -0.10;
/// Returns at or above this height above ground (m) count as obstacles.
const MIN_OBSTACLE_Z_M: f64 = 0.08;
/// Returns above this height above ground (m) are ignored (overhangs / ceilings).
const MAX_OBSTACLE_Z_M: f64 = 1.0;

/// Trusted-floor outlier guard (m): cells farther than this from the overall
/// median floor height are dropped (rejects box tops mislabeled as `.floor`).
const FLOOR_OUTLIER_M: f64 = 0.08;
/// Floor samples must lie at least this far below the camera (m).
const FLOOR_BELOW_CAMERA_M: f64 = 0.10;

/// Sparse log-odds parameters.
const LOG_ODDS_HIT: f32 = 0.85;
const LOG_ODDS_MISS: f32 = 0.40;
const LOG_ODDS_MIN: f32 = -4.0;
const LOG_ODDS_MAX: f32 = 4.0;
const OCCUPIED_THRESHOLD: f32 = 0.5;

pub struct OccupancyGrid {
    pub mount: Mount,
    pub resolution: f64,

    /// Sparse grid of `(ix, iy)` world cells → clamped log-odds of occupancy.
    cells: HashMap<(i32, i32), f32>,
    /// Cells observed as floor/free (kept separate from `cells` so a newly
    /// placed obstacle on known floor is detected on the very first hit).
    observed_free: HashSet<(i32, i32)>,

    /// `.floor` face centroids (Terra world `W`) keyed by `ARMeshAnchor` UUID.
    chunk_floor_samples: HashMap<String, Vec<Vec3>>,
    /// Trusted floor height `z` (world metres) per cell after median + outlier filter.
    trusted_floor: HashMap<(i32, i32), f32>,
    /// Overall median trusted floor height (world metres), when available.
    floor_median_z: Option<f64>,
    /// Most recent camera height in world `W` (m), used to reject above-camera samples.
    last_camera_z: Option<f64>,

    /// Cached world-frame obstacle cell centres returned to the planners.
    map: Map,
}

impl OccupancyGrid {
    pub fn new(mount: Mount, resolution: f64) -> Self {
        assert!(
            resolution.is_finite() && resolution > 0.0,
            "occupancy grid resolution must be positive and finite"
        );
        Self {
            mount,
            resolution,
            cells: HashMap::new(),
            observed_free: HashSet::new(),
            chunk_floor_samples: HashMap::new(),
            trusted_floor: HashMap::new(),
            floor_median_z: None,
            last_camera_z: None,
            map: Map { resolution, ..Map::default() },
        }
    }

    /// World-frame point (m) → discrete grid cell `(ix, iy)`.
    fn world_to_cell(&self, p: Vec2) -> (i32, i32) {
        (
            (p.x / self.resolution).floor() as i32,
            (p.y / self.resolution).floor() as i32,
        )
    }

    /// Discrete grid cell `(ix, iy)` → cell centre in world metres.
    fn cell_to_world(&self, (ix, iy): (i32, i32)) -> Vec2 {
        Vec2 {
            x: (f64::from(ix) + 0.5) * self.resolution,
            y: (f64::from(iy) + 0.5) * self.resolution,
        }
    }

    /// Transform a LiDAR point from camera frame (`C`) into rover body frame (`B`).
    fn camera_to_body(&self, p_cam: Vec3) -> Vec3 {
        let r = self.mount.camera_to_body.apply(p_cam);
        Vec3 {
            x: r.x + self.mount.camera_offset.x,
            y: r.y + self.mount.camera_offset.y,
            z: r.z + self.mount.camera_offset.z,
        }
    }

    /// True when `world_z` sits less than `MIN_OBSTACLE_Z_M` above trusted mesh
    /// floor in `cell` (vetoing a would-be obstacle return).
    fn on_floor(&self, cell: (i32, i32), world_z: f64) -> bool {
        self.trusted_floor
            .get(&cell)
            .is_some_and(|&h| world_z - f64::from(h) < MIN_OBSTACLE_Z_M)
    }

    /// Rebuild `trusted_floor` from all active `.floor` chunk centroids:
    /// bin by cell → per-cell median → drop cells > `8 cm` from global median.
    fn rebuild_floor_surface(&mut self) {
        self.trusted_floor.clear();
        self.floor_median_z = None;

        let ceiling = self.last_camera_z.map(|z| z - FLOOR_BELOW_CAMERA_M);
        let mut bins: HashMap<(i32, i32), Vec<f32>> = HashMap::new();

        for samples in self.chunk_floor_samples.values() {
            for &p in samples {
                if !finite_vec3(p) {
                    continue;
                }
                if let Some(max_z) = ceiling
                    && p.z > max_z
                {
                    continue;
                }
                let cell = self.world_to_cell(Vec2 { x: p.x, y: p.y });
                bins.entry(cell).or_default().push(p.z as f32);
            }
        }

        let mut cell_medians: Vec<((i32, i32), f32)> = bins
            .into_iter()
            .filter_map(|(cell, mut zs)| median(&mut zs).map(|m| (cell, m)))
            .collect();

        let mut all_zs: Vec<f32> = cell_medians.iter().map(|&(_, z)| z).collect();
        let Some(global_median) = median(&mut all_zs).map(f64::from) else {
            return;
        };

        for (cell, z) in cell_medians.drain(..) {
            if (f64::from(z) - global_median).abs() <= FLOOR_OUTLIER_M {
                self.trusted_floor.insert(cell, z);
            }
        }
        if !self.trusted_floor.is_empty() {
            self.floor_median_z = Some(global_median);
        }
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

fn finite_vec3(v: Vec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}

impl Mapper for OccupancyGrid {
    fn update_mesh(
        &mut self,
        id: &str,
        vertices_world: &[Vec3],
        indices: &[u32],
        classes: &[u8],
    ) {
        if id.is_empty() {
            return;
        }

        let face_count = indices.len() / 3;
        let mut samples = Vec::new();
        for f in 0..face_count {
            if classes.get(f).copied() != Some(CLASS_FLOOR) {
                continue;
            }
            let (i0, i1, i2) = (
                indices[f * 3] as usize,
                indices[f * 3 + 1] as usize,
                indices[f * 3 + 2] as usize,
            );
            let (Some(&v0), Some(&v1), Some(&v2)) = (
                vertices_world.get(i0),
                vertices_world.get(i1),
                vertices_world.get(i2),
            ) else {
                continue;
            };
            if finite_vec3(v0) && finite_vec3(v1) && finite_vec3(v2) {
                samples.push(Vec3 {
                    x: (v0.x + v1.x + v2.x) / 3.0,
                    y: (v0.y + v1.y + v2.y) / 3.0,
                    z: (v0.z + v1.z + v2.z) / 3.0,
                });
            }
        }

        if samples.is_empty() {
            if self.chunk_floor_samples.remove(id).is_some() {
                self.rebuild_floor_surface();
            }
        } else {
            self.chunk_floor_samples.insert(id.to_owned(), samples);
            self.rebuild_floor_surface();
        }
    }

    fn update(&mut self, frame: &SensorFrame, state: &State) -> &Map {
        let tracked_vio = frame.vio.as_ref().filter(|v| v.tracked);
        if let Some(vio) = tracked_vio {
            self.last_camera_z = Some(arkit_to_world(vio.position).z);
        }

        if frame.depth.is_empty() {
            return &self.map;
        }

        // 1. Classify each LiDAR point into hit (obstacle) or free (floor / floor-vetoed).
        let mut hit_cells = HashSet::new();
        let mut free_cells = HashSet::new();

        if let Some(vio) = tracked_vio {
            // Full 6-DoF camera → Terra world `W` (gravity-aligned).
            let ground_z = self
                .floor_median_z
                .unwrap_or_else(|| arkit_to_world(vio.position).z - self.mount.camera_offset.z);

            for &p_cam in &frame.depth {
                if !finite_vec3(p_cam) {
                    continue;
                }
                let r = vio.rotation.apply(p_cam);
                let p_world = arkit_to_world(Vec3 {
                    x: r.x + vio.position.x,
                    y: r.y + vio.position.y,
                    z: r.z + vio.position.z,
                });
                let range = (p_world.x - state.pose.x).hypot(p_world.y - state.pose.y);
                if !(MIN_RANGE_M..=MAX_RANGE_M).contains(&range) {
                    continue;
                }
                let cell = self.world_to_cell(Vec2 { x: p_world.x, y: p_world.y });
                let height = p_world.z - ground_z;

                if (MIN_OBSTACLE_Z_M..=MAX_OBSTACLE_Z_M).contains(&height) {
                    // Floor veto: a would-be obstacle sitting right on trusted
                    // mesh floor is flipped to free.
                    if self.on_floor(cell, p_world.z) {
                        free_cells.insert(cell);
                    } else {
                        hit_cells.insert(cell);
                    }
                } else if (MIN_FLOOR_Z_M..MIN_OBSTACLE_Z_M).contains(&height) {
                    free_cells.insert(cell);
                }
            }
        } else {
            // Fallback when no 6-DoF VIO pose is attached: mount camera → body → 2D world.
            for &p_cam in &frame.depth {
                if !finite_vec3(p_cam) {
                    continue;
                }
                let p_body = self.camera_to_body(p_cam);
                let range = p_body.x.hypot(p_body.y);
                if !(MIN_RANGE_M..=MAX_RANGE_M).contains(&range) {
                    continue;
                }
                let p_world = state.pose.to_world(Vec2 { x: p_body.x, y: p_body.y });
                let cell = self.world_to_cell(p_world);

                if (MIN_OBSTACLE_Z_M..=MAX_OBSTACLE_Z_M).contains(&p_body.z) {
                    hit_cells.insert(cell);
                } else if (MIN_FLOOR_Z_M..MIN_OBSTACLE_Z_M).contains(&p_body.z) {
                    free_cells.insert(cell);
                }
            }
        }

        // 2. Apply clamped log-odds updates.
        // Observed free cells live in `observed_free` (separate from obstacle
        // log-odds) so a newly placed obstacle on known floor still triggers on
        // the very first hit instead of fighting negative log-odds.
        let mut obstacles_changed = false;
        let mut free_changed = false;
        for &cell in &hit_cells {
            let entry = self.cells.entry(cell).or_insert(0.0);
            let was_occ = *entry >= OCCUPIED_THRESHOLD;
            *entry = (*entry + LOG_ODDS_HIT).clamp(LOG_ODDS_MIN, LOG_ODDS_MAX);
            obstacles_changed |= was_occ != (*entry >= OCCUPIED_THRESHOLD);
        }
        for &cell in &free_cells {
            if !hit_cells.contains(&cell) {
                free_changed |= self.observed_free.insert(cell);
                if let Some(entry) = self.cells.get_mut(&cell) {
                    let was_occ = *entry >= OCCUPIED_THRESHOLD;
                    *entry = (*entry - LOG_ODDS_MISS).clamp(LOG_ODDS_MIN, LOG_ODDS_MAX);
                    obstacles_changed |= was_occ != (*entry >= OCCUPIED_THRESHOLD);
                }
            }
        }

        // 3. Rebuild world-frame obstacle / free centres when occupancy changed.
        if obstacles_changed || free_changed {
            let res = self.resolution;
            let to_world = |(ix, iy): (i32, i32)| Vec2 {
                x: (f64::from(ix) + 0.5) * res,
                y: (f64::from(iy) + 0.5) * res,
            };
            if obstacles_changed {
                self.map.obstacles.clear();
                self.map.obstacles.extend(
                    self.cells
                        .iter()
                        .filter(|&(_, &lo)| lo >= OCCUPIED_THRESHOLD)
                        .map(|(&cell, _)| to_world(cell)),
                );
                self.map.revision += 1;
            }
            self.map.free.clear();
            self.map.free.extend(
                self.observed_free
                    .iter()
                    .filter(|cell| {
                        self.cells
                            .get(cell)
                            .is_none_or(|&lo| lo < OCCUPIED_THRESHOLD)
                    })
                    .map(|&cell| to_world(cell)),
            );
        }

        &self.map
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Pose2, Rotation3};
    use crate::sense::VioPose;
    use crate::vehicle::Twist;

    fn default_mount() -> Mount {
        Mount {
            camera_to_body: Rotation3::from_euler(0.0, 0.0, 0.0),
            camera_offset: Vec3 { x: 0.0, y: 0.0, z: 0.25 },
        }
    }

    fn state_at(x: f64, y: f64, yaw: f64) -> State {
        State {
            t: 0.0,
            pose: Pose2 { x, y, yaw },
            twist: Twist::STOP,
        }
    }

    fn frame_with_depth(depth: Vec<Vec3>, vio: Option<VioPose>) -> SensorFrame {
        SensorFrame {
            t: 0.0,
            estop: false,
            vio,
            gyro: None,
            gps: None,
            depth,
        }
    }

    /// Make a small `.floor` triangle whose centroid lands at `(x, y, z)`.
    fn floor_tri_at(x: f64, y: f64, z: f64) -> [Vec3; 3] {
        [
            Vec3 { x: x - 0.02, y: y - 0.02, z },
            Vec3 { x: x + 0.02, y: y - 0.02, z },
            Vec3 { x, y: y + 0.04, z },
        ]
    }

    #[test]
    fn floor_veto_clears_return_on_trusted_mesh_floor_and_keeps_obstacle_above_it() {
        let mut grid = OccupancyGrid::new(default_mount(), 0.1);

        // Floor patch around (1.55, 0.05) at z = 0.05 (slightly above z = 0.0,
        // e.g. mild floor unevenness that would otherwise look like an obstacle
        // if ground_z were 0.0).
        let mut verts = Vec::new();
        let mut indices = Vec::new();
        let mut classes = Vec::new();
        for ix in 0..20 {
            let tri = floor_tri_at(ix as f64 * 0.1 + 0.05, 0.05, 0.0);
            let base = verts.len() as u32;
            verts.extend_from_slice(&tri);
            indices.extend_from_slice(&[base, base + 1, base + 2]);
            classes.push(CLASS_FLOOR);
        }
        // Cell (15, 0) at x = 1.55 has floor slightly raised at z = 0.05 (within 8 cm outlier).
        let tri = floor_tri_at(1.55, 0.05, 0.05);
        let base = verts.len() as u32;
        verts.extend_from_slice(&tri);
        indices.extend_from_slice(&[base, base + 1, base + 2]);
        classes.push(CLASS_FLOOR);
        grid.update_mesh("floor-1", &verts, &indices, &classes);

        // Camera at world (0, 0, 0.25) → ARKit (0, 0.25, 0).
        let make_vio = || VioPose {
            position: Vec3 { x: 0.0, y: 0.25, z: 0.0 },
            rotation: Rotation3::from_euler(0.0, 0.0, 0.0),
            tracked: true,
        };

        // 1. LiDAR point at world (1.55, 0.05, 0.09): above global ground_z (0.0)
        //    by 0.09 m (>= 0.08 m obstacle band!), BUT only 0.04 m above trusted
        //    mesh floor at (15, 0) (0.05 m) → VETOED to free!
        //    ARKit relative to cam (0, 0.25, 0): x_A = -0.05, y_A = 0.09 - 0.25 = -0.16, z_A = -1.55.
        let on_floor_pt = Vec3 { x: -0.05, y: -0.16, z: -1.55 };
        let map = grid.update(
            &frame_with_depth(vec![on_floor_pt], Some(make_vio())),
            &state_at(0.0, 0.0, 0.0),
        );
        assert!(
            map.obstacles.is_empty(),
            "return on trusted mesh floor should be vetoed, got {:?}",
            map.obstacles
        );

        // 2. Box standing ON that same cell at world z = 0.25 (0.20 m above mesh floor) → Obstacle!
        let box_pt = Vec3 { x: -0.05, y: 0.0, z: -1.55 };
        let map = grid.update(
            &frame_with_depth(vec![box_pt], Some(make_vio())),
            &state_at(0.0, 0.0, 0.0),
        );
        assert_eq!(map.obstacles.len(), 1);
        assert!((map.obstacles[0].x - 1.55).abs() < 0.05);
    }

    #[test]
    fn mislabelled_raised_box_top_is_rejected_by_floor_outlier_filter() {
        let mut grid = OccupancyGrid::new(default_mount(), 0.1);

        // 20 real floor faces at z = 0.0, plus 1 box top at z = 0.30 mislabelled as CLASS_FLOOR.
        let mut verts = Vec::new();
        let mut indices = Vec::new();
        let mut classes = Vec::new();
        for ix in 0..20 {
            let tri = floor_tri_at(ix as f64 * 0.1 + 0.05, 0.05, 0.0);
            let base = verts.len() as u32;
            verts.extend_from_slice(&tri);
            indices.extend_from_slice(&[base, base + 1, base + 2]);
            classes.push(CLASS_FLOOR);
        }
        let box_top = floor_tri_at(1.55, 0.55, 0.30);
        let base = verts.len() as u32;
        verts.extend_from_slice(&box_top);
        indices.extend_from_slice(&[base, base + 1, base + 2]);
        classes.push(CLASS_FLOOR);
        grid.update_mesh("floor-outlier", &verts, &indices, &classes);

        // The mislabelled box top at (15, 5) must NOT be in trusted_floor, so
        // a LiDAR hit on the box at world (1.55, 0.55, 0.30) stays an obstacle.
        let vio = VioPose {
            position: Vec3 { x: 0.0, y: 0.25, z: 0.0 },
            rotation: Rotation3::from_euler(0.0, 0.0, 0.0),
            tracked: true,
        };
        let box_pt = Vec3 { x: -0.55, y: 0.05, z: -1.55 };
        let map = grid.update(
            &frame_with_depth(vec![box_pt], Some(vio)),
            &state_at(0.0, 0.0, 0.0),
        );
        assert_eq!(map.obstacles.len(), 1);
        assert!((map.obstacles[0].x - 1.55).abs() < 0.05);
        assert!((map.obstacles[0].y - 0.55).abs() < 0.05);
    }

    #[test]
    fn obstacle_clears_after_repeated_floor_observations() {
        let mut grid = OccupancyGrid::new(default_mount(), 0.1);
        let vio = || VioPose {
            position: Vec3 { x: 0.0, y: 0.25, z: 0.0 },
            rotation: Rotation3::from_euler(0.0, 0.0, 0.0),
            tracked: true,
        };

        // 1 hit at world (1.05, 0.05, 0.25) -> log-odds = +0.85 (>= 0.5, occupied)
        let hit = Vec3 { x: -0.05, y: 0.0, z: -1.05 };
        let map = grid.update(
            &frame_with_depth(vec![hit], Some(vio())),
            &state_at(0.0, 0.0, 0.0),
        );
        assert_eq!(map.obstacles.len(), 1);

        // 2 floor returns at world (1.05, 0.05, 0.0) -> 0.85 - 2*0.40 = 0.05 (< 0.5, cleared)
        let floor = Vec3 { x: -0.05, y: -0.25, z: -1.05 };
        grid.update(
            &frame_with_depth(vec![floor], Some(vio())),
            &state_at(0.0, 0.0, 0.0),
        );
        let map = grid.update(
            &frame_with_depth(vec![floor], Some(vio())),
            &state_at(0.0, 0.0, 0.0),
        );
        assert!(map.obstacles.is_empty());
    }
}
