//! Two-layer world-frame occupancy grid combining:
//!
//! 1. **Static semantic mesh layer (`ARMeshAnchor` via [`Mapper::update_mesh`]):**
//!    - `.floor`-classified triangles build a 2D world ground-elevation map
//!      `floor_z(ix, iy)` (handling flat floors, ramps, and multi-level landings).
//!    - Adjacent floor cells whose elevation jumps faster than a traversable ramp
//!      (curbs, steps, stairs) are flagged as step obstacles.
//!    - Non-floor, non-ceiling triangles whose height above `floor_z(ix, iy)` falls
//!      in `[MIN_OBSTACLE_Z_M, MAX_OBSTACLE_Z_M]` are rasterized per-anchor so
//!      chunk refinements and removals automatically clean up stale obstacles.
//!    - Floor directly under the rover continuously auto-calibrates the camera's
//!      height above ground (`calibrated_camera_height`).
//!
//! 2. **Dynamic fast LiDAR layer (20 Hz `SensorFrame.depth` via [`Mapper::update`]):**
//!    - When a tracked 6-DoF ARKit `VioPose` is present, depth points are
//!      projected straight into Terra world `W` (`T_{W←A} · T_{A←C} · p_cam`),
//!      which is already gravity-aligned and immune to mount tilt and ramp pitch,
//!      and classified by `p_world.z - floor_z(ix, iy)`.
//!    - When `frame.vio` is absent, falls back to `[mount]` body-frame projection.
//!
//! `Map.obstacles` is the union of the static mesh layer, step obstacles, and
//! dynamic LiDAR log-odds layer.
use std::collections::{HashMap, HashSet};

use super::{Map, Mapper};
use crate::{
    estimation::{State, arkit_to_world},
    geometry::{Vec2, Vec3},
    sense::{Mount, SensorFrame},
};

/// Apple `ARMeshClassification` raw values.
const CLASS_NONE: u8 = 0;
const CLASS_FLOOR: u8 = 2;
const CLASS_CEILING: u8 = 3;

/// Minimum horizontal distance from the rover origin (m); ignores self-hits.
const MIN_RANGE_M: f64 = 0.15;
/// Maximum reliable LiDAR range (m).
const MAX_RANGE_M: f64 = 5.0;

/// Floor band relative to local `floor_z` (m).
const MIN_FLOOR_Z_M: f64 = -0.10;
/// Points at or above this height above `floor_z` (m) count as obstacles.
const MIN_OBSTACLE_Z_M: f64 = 0.08;
/// Points above this height above `floor_z` (m) are ignored (overhangs / ceilings).
const MAX_OBSTACLE_Z_M: f64 = 1.0;

/// Nearly-horizontal `.none` triangles (`|n_z| >= 0.85`, i.e. within ~32° of flat)
/// sitting near `floor_z` are treated as mislabeled floor rather than obstacles.
const FLOOR_NORMAL_Z_MIN: f64 = 0.85;

/// Ramp vs. curb/stair threshold between 4-connected floor cells:
/// slopes up to ~19° (`0.35`) or steps below `6 cm` stay traversable.
const MAX_TRAVERSABLE_SLOPE: f64 = 0.35;
const MIN_STEP_OBSTACLE_M: f64 = 0.06;

/// Sanity bounds and EMA smoothing factor for live camera-above-floor calibration.
const MIN_CAMERA_HEIGHT_M: f64 = 0.05;
const MAX_CAMERA_HEIGHT_M: f64 = 2.0;
const CAMERA_HEIGHT_EMA_ALPHA: f64 = 0.15;

/// Guard against degenerate huge triangles stalling rasterization.
const MAX_RASTER_SPAN_CELLS: i32 = 64;
const MAX_EDGE_STEPS: usize = 200;

/// Sparse log-odds parameters for the dynamic LiDAR layer.
const LOG_ODDS_HIT: f32 = 0.85;
const LOG_ODDS_MISS: f32 = 0.40;
const LOG_ODDS_MIN: f32 = -4.0;
const LOG_ODDS_MAX: f32 = 4.0;
const OCCUPIED_THRESHOLD: f32 = 0.5;

/// Per-`ARMeshAnchor` contribution so updates/removals are O(changed chunk).
#[derive(Default)]
struct MeshChunkCells {
    /// Mean floor elevation `z` (m, Terra world `W`) per cell from `.floor` faces.
    floor: HashMap<(i32, i32), f64>,
    /// Occupied cells contributed by this chunk's obstacle faces.
    obstacles: HashSet<(i32, i32)>,
}

pub struct OccupancyGrid {
    pub mount: Mount,
    pub resolution: f64,

    // --- Dynamic fast LiDAR layer ---
    /// Sparse grid of `(ix, iy)` world cells → clamped log-odds of occupancy.
    cells: HashMap<(i32, i32), f32>,

    // --- Static semantic mesh layer (ARMeshAnchor) ---
    /// Per-anchor contributions keyed by anchor UUID.
    mesh_chunks: HashMap<String, MeshChunkCells>,
    /// Aggregated floor elevation `(sum_z, chunk_count)` per cell.
    floor_elevation: HashMap<(i32, i32), (f64, u32)>,
    /// Refcount of mesh chunks marking each cell as an obstacle.
    mesh_obstacle_counts: HashMap<(i32, i32), u32>,
    /// Floor cells bordering a steep elevation jump (curb / stair).
    step_obstacles: HashSet<(i32, i32)>,

    // --- Auto-calibrated ground reference ---
    /// Live camera height above the semantic `.floor` directly under the rover (m).
    calibrated_camera_height: Option<f64>,
    /// Most recent world-frame floor elevation under the rover (m).
    last_rover_floor_z: Option<f64>,

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
            mesh_chunks: HashMap::new(),
            floor_elevation: HashMap::new(),
            mesh_obstacle_counts: HashMap::new(),
            step_obstacles: HashSet::new(),
            calibrated_camera_height: None,
            last_rover_floor_z: None,
            map: Map { resolution, ..Map::default() },
        }
    }

    /// Auto-calibrated camera height above ground (m), if `.floor` has been
    /// observed under the rover; otherwise falls back to `mount.camera_offset.z`.
    pub fn camera_height(&self) -> f64 {
        self.calibrated_camera_height
            .unwrap_or(self.mount.camera_offset.z)
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

    /// Transform a LiDAR point from camera frame (`C`) into rover body frame (`B`):
    /// `p_B = R_{B←C} · p_C + camera_offset`.
    fn camera_to_body(&self, p_cam: Vec3) -> Vec3 {
        let r = self.mount.camera_to_body.apply(p_cam);
        Vec3 {
            x: r.x + self.mount.camera_offset.x,
            y: r.y + self.mount.camera_offset.y,
            z: r.z + self.mount.camera_offset.z,
        }
    }

    /// Exact mean floor elevation at `cell`, if any `.floor` mesh triangle covers it.
    fn exact_floor_z(&self, cell: (i32, i32)) -> Option<f64> {
        self.floor_elevation
            .get(&cell)
            .filter(|(_, n)| *n > 0)
            .map(|&(sum, n)| sum / f64::from(n))
    }

    /// Local floor elevation at `cell`, searching the 3×3 neighborhood if the
    /// exact cell has no `.floor` sample yet.
    fn local_floor_z(&self, (ix, iy): (i32, i32)) -> Option<f64> {
        if let Some(z) = self.exact_floor_z((ix, iy)) {
            return Some(z);
        }
        let mut sum = 0.0;
        let mut count = 0u32;
        for dx in -1..=1 {
            for dy in -1..=1 {
                if dx == 0 && dy == 0 {
                    continue;
                }
                if let Some(z) = self.exact_floor_z((ix + dx, iy + dy)) {
                    sum += z;
                    count += 1;
                }
            }
        }
        (count > 0).then(|| sum / f64::from(count))
    }

    /// Maximum elevation difference between 4-connected floor cells that is
    /// still treated as a traversable ramp (above this = curb / stair).
    fn max_traversable_step(&self) -> f64 {
        (MAX_TRAVERSABLE_SLOPE * self.resolution).max(MIN_STEP_OBSTACLE_M)
    }

    /// Re-evaluate whether `cell` borders a steep floor discontinuity.
    fn refresh_step_cell(&mut self, cell: (i32, i32)) -> bool {
        let was_step = self.step_obstacles.contains(&cell);
        let is_step = match self.exact_floor_z(cell) {
            Some(z) => {
                let max_step = self.max_traversable_step();
                let (ix, iy) = cell;
                [(ix - 1, iy), (ix + 1, iy), (ix, iy - 1), (ix, iy + 1)]
                    .into_iter()
                    .filter_map(|n| self.exact_floor_z(n))
                    .any(|nz| (z - nz).abs() > max_step)
            }
            None => false,
        };
        if is_step {
            self.step_obstacles.insert(cell);
        } else {
            self.step_obstacles.remove(&cell);
        }
        was_step != is_step
    }

    /// Whether `cell` is currently occupied in ANY layer (mesh, step, or LiDAR).
    fn is_occupied(&self, cell: (i32, i32)) -> bool {
        self.mesh_obstacle_counts.get(&cell).copied().unwrap_or(0) > 0
            || self.step_obstacles.contains(&cell)
            || self
                .cells
                .get(&cell)
                .is_some_and(|&lo| lo >= OCCUPIED_THRESHOLD)
    }

    /// Rebuild `self.map.obstacles` as the union of all occupied cells and bump `revision`.
    fn rebuild_obstacles(&mut self) {
        let mut occupied = HashSet::new();
        for (&cell, &count) in &self.mesh_obstacle_counts {
            if count > 0 {
                occupied.insert(cell);
            }
        }
        for &cell in &self.step_obstacles {
            occupied.insert(cell);
        }
        for (&cell, &log_odds) in &self.cells {
            if log_odds >= OCCUPIED_THRESHOLD {
                occupied.insert(cell);
            }
        }
        let res = self.resolution;
        self.map.obstacles.clear();
        self.map.obstacles.extend(occupied.into_iter().map(|(ix, iy)| Vec2 {
            x: (f64::from(ix) + 0.5) * res,
            y: (f64::from(iy) + 0.5) * res,
        }));
        self.map.revision += 1;
    }

    /// Rasterize a world-frame triangle `(v0, v1, v2)` onto 2D grid cells, calling
    /// `visit(cell, z)` for every covered cell along its edges (so vertical walls
    /// with zero 2D area are never missed) and interior.
    fn rasterize_triangle(&self, v0: Vec3, v1: Vec3, v2: Vec3, mut visit: impl FnMut((i32, i32), f64)) {
        // 1. Sample all 3 edges at half-cell spacing (catches vertical walls).
        let step_m = self.resolution * 0.5;
        for (a, b) in [(v0, v1), (v1, v2), (v2, v0)] {
            let len_xy = (b.x - a.x).hypot(b.y - a.y);
            let steps = ((len_xy / step_m).ceil() as usize).clamp(1, MAX_EDGE_STEPS);
            for i in 0..=steps {
                let t = i as f64 / steps as f64;
                let x = a.x + t * (b.x - a.x);
                let y = a.y + t * (b.y - a.y);
                let z = a.z + t * (b.z - a.z);
                visit(self.world_to_cell(Vec2 { x, y }), z);
            }
        }

        // 2. Rasterize interior cell centres via 2D barycentric coordinates
        //    (gives exact interpolated elevation on wide/sloped floor faces).
        let area2 = (v1.x - v0.x) * (v2.y - v0.y) - (v1.y - v0.y) * (v2.x - v0.x);
        if area2.abs() < 1e-8 {
            return;
        }
        let min_x = v0.x.min(v1.x).min(v2.x);
        let max_x = v0.x.max(v1.x).max(v2.x);
        let min_y = v0.y.min(v1.y).min(v2.y);
        let max_y = v0.y.max(v1.y).max(v2.y);
        let (min_ix, min_iy) = self.world_to_cell(Vec2 { x: min_x, y: min_y });
        let (max_ix, max_iy) = self.world_to_cell(Vec2 { x: max_x, y: max_y });
        if max_ix - min_ix > MAX_RASTER_SPAN_CELLS || max_iy - min_iy > MAX_RASTER_SPAN_CELLS {
            return;
        }

        let inv_area2 = 1.0 / area2;
        for ix in min_ix..=max_ix {
            for iy in min_iy..=max_iy {
                let c = self.cell_to_world((ix, iy));
                let w0 = ((v1.x - c.x) * (v2.y - c.y) - (v1.y - c.y) * (v2.x - c.x)) * inv_area2;
                let w1 = ((v2.x - c.x) * (v0.y - c.y) - (v2.y - c.y) * (v0.x - c.x)) * inv_area2;
                let w2 = 1.0 - w0 - w1;
                if w0 >= 0.0 && w1 >= 0.0 && w2 >= 0.0 {
                    let z = w0 * v0.z + w1 * v1.z + w2 * v2.z;
                    visit((ix, iy), z);
                }
            }
        }
    }
}

/// Cross product `(v1 - v0) × (v2 - v0)` and its `|n_z| / |n|` cosine.
fn vertical_normal_cos(v0: Vec3, v1: Vec3, v2: Vec3) -> f64 {
    let e1 = Vec3 { x: v1.x - v0.x, y: v1.y - v0.y, z: v1.z - v0.z };
    let e2 = Vec3 { x: v2.x - v0.x, y: v2.y - v0.y, z: v2.z - v0.z };
    let nx = e1.y * e2.z - e1.z * e2.y;
    let ny = e1.z * e2.x - e1.x * e2.z;
    let nz = e1.x * e2.y - e1.y * e2.x;
    let norm = (nx * nx + ny * ny + nz * nz).sqrt();
    if norm < 1e-9 { 0.0 } else { (nz / norm).abs() }
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

        let mut affected_floor = HashSet::new();
        let mut obstacles_changed = false;

        // 1. Remove this chunk's previous contributions.
        if let Some(old) = self.mesh_chunks.remove(id) {
            for (cell, z) in old.floor {
                affected_floor.insert(cell);
                if let Some((sum, count)) = self.floor_elevation.get_mut(&cell) {
                    if *count <= 1 {
                        self.floor_elevation.remove(&cell);
                    } else {
                        *sum -= z;
                        *count -= 1;
                    }
                }
            }
            for cell in old.obstacles {
                let was_occupied = self.is_occupied(cell);
                if let Some(count) = self.mesh_obstacle_counts.get_mut(&cell) {
                    if *count <= 1 {
                        self.mesh_obstacle_counts.remove(&cell);
                    } else {
                        *count -= 1;
                    }
                }
                obstacles_changed |= was_occupied != self.is_occupied(cell);
            }
        }

        // 2. If non-empty, rasterize the updated chunk in two passes:
        //    Pass 1 extracts `.floor` elevations; Pass 2 classifies obstacles
        //    against the updated local floor elevation map.
        let face_count = indices.len() / 3;
        if face_count > 0 && !vertices_world.is_empty() {
            let mut floor_acc: HashMap<(i32, i32), (f64, u32)> = HashMap::new();

            // Pass 1: `.floor` triangles → local ground elevation.
            for f in 0..face_count {
                let class = classes.get(f).copied().unwrap_or(CLASS_NONE);
                if class != CLASS_FLOOR {
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
                if !(finite_vec3(v0) && finite_vec3(v1) && finite_vec3(v2)) {
                    continue;
                }
                self.rasterize_triangle(v0, v1, v2, |cell, z| {
                    let entry = floor_acc.entry(cell).or_insert((0.0, 0));
                    entry.0 += z;
                    entry.1 += 1;
                });
            }

            let mut chunk_floor = HashMap::with_capacity(floor_acc.len());
            for (cell, (sum, n)) in floor_acc {
                let mean_z = sum / f64::from(n);
                chunk_floor.insert(cell, mean_z);
                affected_floor.insert(cell);
                let entry = self.floor_elevation.entry(cell).or_insert((0.0, 0));
                entry.0 += mean_z;
                entry.1 += 1;
            }

            // Pass 2: non-floor, non-ceiling triangles → static obstacles.
            let fallback_floor_z = self.last_rover_floor_z.unwrap_or(0.0);
            let mut chunk_obstacles = HashSet::new();
            for f in 0..face_count {
                let class = classes.get(f).copied().unwrap_or(CLASS_NONE);
                if class == CLASS_FLOOR || class == CLASS_CEILING {
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
                if !(finite_vec3(v0) && finite_vec3(v1) && finite_vec3(v2)) {
                    continue;
                }

                let min_z = v0.z.min(v1.z).min(v2.z);
                let max_z = v0.z.max(v1.z).max(v2.z);
                let mean_z = (v0.z + v1.z + v2.z) / 3.0;
                let flat_unclassified =
                    class == CLASS_NONE && vertical_normal_cos(v0, v1, v2) >= FLOOR_NORMAL_Z_MIN;

                self.rasterize_triangle(v0, v1, v2, |cell, _| {
                    let floor_z = self.local_floor_z(cell).unwrap_or(fallback_floor_z);
                    // Flat `.none` surfaces right on the floor (rugs, floor edges) are floor.
                    if flat_unclassified && mean_z < floor_z + MIN_OBSTACLE_Z_M {
                        return;
                    }
                    // Triangle overlaps the rover's vertical clearance band above local floor.
                    if max_z >= floor_z + MIN_OBSTACLE_Z_M && min_z <= floor_z + MAX_OBSTACLE_Z_M {
                        chunk_obstacles.insert(cell);
                    }
                });
            }

            for &cell in &chunk_obstacles {
                let was_occupied = self.is_occupied(cell);
                *self.mesh_obstacle_counts.entry(cell).or_insert(0) += 1;
                obstacles_changed |= was_occupied != self.is_occupied(cell);
            }

            if !chunk_floor.is_empty() || !chunk_obstacles.is_empty() {
                self.mesh_chunks.insert(
                    id.to_owned(),
                    MeshChunkCells {
                        floor: chunk_floor,
                        obstacles: chunk_obstacles,
                    },
                );
            }
        }

        // 3. Recompute step/curb obstacles for changed floor cells and their 4-neighbors.
        if !affected_floor.is_empty() {
            let mut to_check = HashSet::with_capacity(affected_floor.len() * 5);
            for (ix, iy) in affected_floor {
                to_check.insert((ix, iy));
                to_check.insert((ix - 1, iy));
                to_check.insert((ix + 1, iy));
                to_check.insert((ix, iy - 1));
                to_check.insert((ix, iy + 1));
            }
            for cell in to_check {
                let was_occupied = self.is_occupied(cell);
                self.refresh_step_cell(cell);
                obstacles_changed |= was_occupied != self.is_occupied(cell);
            }
        }

        if obstacles_changed {
            self.rebuild_obstacles();
        }
    }

    fn update(&mut self, frame: &SensorFrame, state: &State) -> &Map {
        // 1. Live camera-height auto-calibration from semantic `.floor` under the rover.
        let tracked_vio = frame.vio.as_ref().filter(|v| v.tracked);
        let rover_cell = self.world_to_cell(state.pose.position());
        let cam_world_z = tracked_vio.map(|v| arkit_to_world(v.position).z);

        if let Some(cam_z) = cam_world_z {
            if let Some(floor_z) = self.local_floor_z(rover_cell) {
                self.last_rover_floor_z = Some(floor_z);
                let observed = cam_z - floor_z;
                if (MIN_CAMERA_HEIGHT_M..=MAX_CAMERA_HEIGHT_M).contains(&observed) {
                    self.calibrated_camera_height = Some(match self.calibrated_camera_height {
                        Some(prev) => {
                            prev * (1.0 - CAMERA_HEIGHT_EMA_ALPHA)
                                + observed * CAMERA_HEIGHT_EMA_ALPHA
                        }
                        None => observed,
                    });
                }
            } else {
                // No semantic floor under the rover yet: derive ground elevation
                // from the calibrated (or configured) camera height.
                self.last_rover_floor_z = Some(cam_z - self.camera_height());
            }
        }

        if frame.depth.is_empty() {
            return &self.map;
        }

        // 2. Classify each LiDAR point into hit (obstacle) or free (floor) cells.
        //    Deduplicate per frame so a dense cluster in one cell counts once per scan,
        //    and an obstacle hit overrides floor hits in the same cell.
        let mut hit_cells = HashSet::new();
        let mut free_cells = HashSet::new();

        if let Some(vio) = tracked_vio {
            // Full 6-DoF camera → Terra world `W` (gravity-aligned, ramp-aware).
            let fallback_floor_z = self
                .last_rover_floor_z
                .unwrap_or_else(|| arkit_to_world(vio.position).z - self.camera_height());
            for &p_cam in &frame.depth {
                if !finite_vec3(p_cam) {
                    continue;
                }
                let r_a = vio.rotation.apply(p_cam);
                let p_world = arkit_to_world(Vec3 {
                    x: r_a.x + vio.position.x,
                    y: r_a.y + vio.position.y,
                    z: r_a.z + vio.position.z,
                });
                let range = (p_world.x - state.pose.x).hypot(p_world.y - state.pose.y);
                if !(MIN_RANGE_M..=MAX_RANGE_M).contains(&range) {
                    continue;
                }
                let cell = self.world_to_cell(Vec2 { x: p_world.x, y: p_world.y });
                let floor_z = self.local_floor_z(cell).unwrap_or(fallback_floor_z);
                let height = p_world.z - floor_z;

                if (MIN_OBSTACLE_Z_M..=MAX_OBSTACLE_Z_M).contains(&height) {
                    hit_cells.insert(cell);
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

        // 3. Apply clamped log-odds updates to the dynamic LiDAR layer.
        let mut obstacles_changed = false;
        for &cell in &hit_cells {
            let was_occupied = self.is_occupied(cell);
            let entry = self.cells.entry(cell).or_insert(0.0);
            *entry = (*entry + LOG_ODDS_HIT).clamp(LOG_ODDS_MIN, LOG_ODDS_MAX);
            obstacles_changed |= was_occupied != self.is_occupied(cell);
        }
        for &cell in &free_cells {
            if !hit_cells.contains(&cell) && self.cells.contains_key(&cell) {
                let was_occupied = self.is_occupied(cell);
                if let Some(entry) = self.cells.get_mut(&cell) {
                    *entry = (*entry - LOG_ODDS_MISS).clamp(LOG_ODDS_MIN, LOG_ODDS_MAX);
                }
                obstacles_changed |= was_occupied != self.is_occupied(cell);
            }
        }

        if obstacles_changed {
            self.rebuild_obstacles();
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
        // Identity camera→body for body-frame fallback tests; phone 0.25 m above floor.
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

    #[test]
    fn vertical_wall_mesh_chunk_rasterizes_and_clears_on_removal() {
        let mut grid = OccupancyGrid::new(default_mount(), 0.1);
        // Vertical wall along x = 1.05, y in [0.0, 0.5], z in [0.0, 1.5].
        let verts = [
            Vec3 { x: 1.05, y: 0.0, z: 0.0 },
            Vec3 { x: 1.05, y: 0.5, z: 0.0 },
            Vec3 { x: 1.05, y: 0.5, z: 1.5 },
        ];
        grid.update_mesh("wall-1", &verts, &[0, 1, 2], &[1]); // CLASS_WALL = 1

        let map = grid.update(&frame_with_depth(vec![], None), &state_at(0.0, 0.0, 0.0));
        assert!(map.revision > 0);
        assert!(
            map.obstacles.len() >= 5,
            "expected vertical wall edges rasterized across ~5 cells, got {}",
            map.obstacles.len()
        );
        assert!(map.obstacles.iter().all(|p| (p.x - 1.05).abs() < 0.1));

        // Removing the chunk clears all of its obstacles.
        grid.update_mesh("wall-1", &[], &[], &[]);
        let map = grid.update(&frame_with_depth(vec![], None), &state_at(0.0, 0.0, 0.0));
        assert!(map.obstacles.is_empty());
    }

    #[test]
    fn floor_ceiling_and_flat_unclassified_rug_are_ignored_while_box_is_kept() {
        let mut grid = OccupancyGrid::new(default_mount(), 0.1);
        let verts = [
            // Floor triangle at z = 0.0 (CLASS_FLOOR)
            Vec3 { x: 0.0, y: 0.0, z: 0.0 },
            Vec3 { x: 2.0, y: 0.0, z: 0.0 },
            Vec3 { x: 2.0, y: 2.0, z: 0.0 },
            // Ceiling triangle at z = 2.4 (CLASS_CEILING)
            Vec3 { x: 0.0, y: 0.0, z: 2.4 },
            Vec3 { x: 2.0, y: 0.0, z: 2.4 },
            Vec3 { x: 2.0, y: 2.0, z: 2.4 },
            // Flat unclassified rug at z = 0.02 (CLASS_NONE, horizontal)
            Vec3 { x: 0.5, y: 0.5, z: 0.02 },
            Vec3 { x: 0.8, y: 0.5, z: 0.02 },
            Vec3 { x: 0.8, y: 0.8, z: 0.02 },
            // Small unclassified box face at z = 0.25 (CLASS_NONE)
            Vec3 { x: 1.5, y: 0.2, z: 0.0 },
            Vec3 { x: 1.5, y: 0.4, z: 0.0 },
            Vec3 { x: 1.5, y: 0.4, z: 0.25 },
        ];
        let indices = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11];
        let classes = [CLASS_FLOOR, CLASS_CEILING, CLASS_NONE, CLASS_NONE];
        grid.update_mesh("room-1", &verts, &indices, &classes);

        let map = grid.update(&frame_with_depth(vec![], None), &state_at(0.0, 0.0, 0.0));
        assert!(!map.obstacles.is_empty());
        // Only the box around x ≈ 1.5 should be occupied (not the floor, ceiling, or rug).
        assert!(map.obstacles.iter().all(|p| p.x > 1.3 && p.x < 1.7));
    }

    #[test]
    fn ramp_floor_stays_free_while_obstacle_on_ramp_is_detected() {
        let mut grid = OccupancyGrid::new(default_mount(), 0.1);
        // Gentle ramp rising from z = 0.0 at x = 0.0 to z = 0.40 at x = 2.0 (slope 0.20 = 11.3°).
        let verts = [
            Vec3 { x: 0.0, y: -0.5, z: 0.0 },
            Vec3 { x: 2.0, y: -0.5, z: 0.4 },
            Vec3 { x: 2.0, y: 0.5, z: 0.4 },
            Vec3 { x: 0.0, y: 0.5, z: 0.0 },
        ];
        grid.update_mesh("ramp", &verts, &[0, 1, 2, 0, 2, 3], &[CLASS_FLOOR, CLASS_FLOOR]);

        // Rover at (0, 0) with camera at world (0, 0, 0.25) -> ARKit (0, 0.25, 0).
        // With identity ARKit rotation, camera point p_cam = (x_A, y_A, z_A) maps to
        // world (-z_A, -x_A, y_A).
        let vio = VioPose {
            position: Vec3 { x: 0.0, y: 0.25, z: 0.0 },
            rotation: Rotation3::from_euler(0.0, 0.0, 0.0),
            tracked: true,
        };

        // 1. LiDAR hit ON the ramp surface at x_W = 1.5, y_W = 0.0, z_W = 0.30.
        //    In camera frame (relative to cam at z_W = 0.25): z_A = -1.5, x_A = 0.0, y_A = +0.05.
        //    Even though z_W = 0.30 is way above 0.08 m, it sits right on the ramp floor!
        let ramp_surface_pt = Vec3 { x: 0.0, y: 0.05, z: -1.5 };
        let map = grid.update(
            &frame_with_depth(vec![ramp_surface_pt], Some(VioPose {
                position: vio.position,
                rotation: Rotation3::from_euler(0.0, 0.0, 0.0),
                tracked: true,
            })),
            &state_at(0.0, 0.0, 0.0),
        );
        assert!(
            map.obstacles.is_empty(),
            "ramp surface should be classified as floor, got {:?}",
            map.obstacles
        );

        // 2. Box sitting ON the ramp at x_W = 1.5, y_W = 0.0, z_W = 0.50 (20 cm above ramp).
        //    Relative to cam at z_W = 0.25: y_A = +0.25, z_A = -1.5.
        let box_on_ramp_pt = Vec3 { x: 0.0, y: 0.25, z: -1.5 };
        let map = grid.update(
            &frame_with_depth(vec![box_on_ramp_pt], Some(vio)),
            &state_at(0.0, 0.0, 0.0),
        );
        assert_eq!(map.obstacles.len(), 1);
        assert!((map.obstacles[0].x - 1.55).abs() < 0.1);
    }

    #[test]
    fn steep_floor_step_marks_curb_as_obstacle() {
        let mut grid = OccupancyGrid::new(default_mount(), 0.1);
        // Lower floor at z = 0.0 (x in [0, 1]) and upper landing at z = 0.20 (x in [1, 2]).
        let verts = [
            Vec3 { x: 0.0, y: -0.2, z: 0.0 },
            Vec3 { x: 0.99, y: -0.2, z: 0.0 },
            Vec3 { x: 0.99, y: 0.2, z: 0.0 },
            Vec3 { x: 1.01, y: -0.2, z: 0.20 },
            Vec3 { x: 2.0, y: -0.2, z: 0.20 },
            Vec3 { x: 2.0, y: 0.2, z: 0.20 },
        ];
        grid.update_mesh("stairs", &verts, &[0, 1, 2, 3, 4, 5], &[CLASS_FLOOR, CLASS_FLOOR]);

        let map = grid.update(&frame_with_depth(vec![], None), &state_at(0.0, 0.0, 0.0));
        assert!(
            !map.obstacles.is_empty(),
            "20 cm vertical step between adjacent floor cells should be marked as an obstacle"
        );
        assert!(map.obstacles.iter().all(|p| p.x > 0.85 && p.x < 1.15));
    }

    #[test]
    fn camera_height_auto_calibrates_from_semantic_floor() {
        // Deliberately misconfigure mount height as 0.10 m when true height is 0.35 m.
        let wrong_mount = Mount {
            camera_to_body: Rotation3::from_euler(0.0, 0.0, 0.0),
            camera_offset: Vec3 { x: 0.0, y: 0.0, z: 0.10 },
        };
        let mut grid = OccupancyGrid::new(wrong_mount, 0.1);

        // Floor patch under the rover at z = 0.0.
        let floor = [
            Vec3 { x: -0.5, y: -0.5, z: 0.0 },
            Vec3 { x: 0.5, y: -0.5, z: 0.0 },
            Vec3 { x: 0.5, y: 0.5, z: 0.0 },
            Vec3 { x: -0.5, y: 0.5, z: 0.0 },
        ];
        grid.update_mesh("under-rover", &floor, &[0, 1, 2, 0, 2, 3], &[CLASS_FLOOR, CLASS_FLOOR]);

        // Camera is actually at world z = 0.35 (ARKit y = 0.35).
        let vio = VioPose {
            position: Vec3 { x: 0.0, y: 0.35, z: 0.0 },
            rotation: Rotation3::from_euler(0.0, 0.0, 0.0),
            tracked: true,
        };
        // LiDAR hit on the floor OUTSIDE the mesh patch at x_W = 2.0, z_W = 0.0
        // (relative to cam at z_W = 0.35: y_A = -0.35, z_A = -2.0).
        let floor_pt_outside_mesh = Vec3 { x: 0.0, y: -0.35, z: -2.0 };
        let obstacles_empty = grid
            .update(
                &frame_with_depth(vec![floor_pt_outside_mesh], Some(vio)),
                &state_at(0.0, 0.0, 0.0),
            )
            .obstacles
            .is_empty();

        assert!((grid.camera_height() - 0.35).abs() < 1e-6);
        assert!(
            obstacles_empty,
            "auto-calibrated height should classify distant flat floor as free"
        );
    }
}
