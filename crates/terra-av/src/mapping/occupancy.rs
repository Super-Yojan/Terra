//! LiDAR depth points (camera frame) → body frame via the mount → world via
//! the state, then accumulated into a sparse world-frame log-odds grid.
//! Points near the floor clear cells (free space); points above it mark obstacles.
use std::collections::{HashMap, HashSet};

use super::{Map, Mapper};
use crate::{
    estimation::State,
    geometry::{Vec2, Vec3},
    sense::{Mount, SensorFrame},
};

/// Minimum horizontal distance from the rover origin (m); ignores self-hits.
const MIN_RANGE_M: f64 = 0.15;
/// Maximum reliable LiDAR range (m).
const MAX_RANGE_M: f64 = 5.0;
/// Floor band in body frame (z = 0 is the ground).
const MIN_FLOOR_Z_M: f64 = -0.10;
/// Points at or above this height (m) count as obstacles; below it count as floor.
const MIN_OBSTACLE_Z_M: f64 = 0.08;
/// Points above this height (m) are ignored (overhangs / ceilings).
const MAX_OBSTACLE_Z_M: f64 = 1.0;

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
            map: Map {
                obstacles: Vec::new(),
            },
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
}

impl Mapper for OccupancyGrid {
    fn update(&mut self, frame: &SensorFrame, state: &State) -> &Map {
        if frame.depth.is_empty() {
            return &self.map;
        }

        // Deduplicate per frame so a dense cluster in one cell counts once per scan,
        // and an obstacle hit in a cell overrides floor hits in the same cell.
        let mut hit_cells = HashSet::new();
        let mut free_cells = HashSet::new();

        for &p_cam in &frame.depth {
            if !(p_cam.x.is_finite() && p_cam.y.is_finite() && p_cam.z.is_finite()) {
                continue;
            }

            // 1. Camera → Body (+X forward, +Y left, +Z up, ground at z = 0).
            let p_body = self.camera_to_body(p_cam);
            let range = p_body.x.hypot(p_body.y);
            if !(MIN_RANGE_M..=MAX_RANGE_M).contains(&range) {
                continue;
            }

            // 2. Classify by height in body frame, then project into Terra world.
            let p_world = state.pose.to_world(Vec2 {
                x: p_body.x,
                y: p_body.y,
            });
            let cell = self.world_to_cell(p_world);

            if (MIN_OBSTACLE_Z_M..=MAX_OBSTACLE_Z_M).contains(&p_body.z) {
                hit_cells.insert(cell);
            } else if (MIN_FLOOR_Z_M..MIN_OBSTACLE_Z_M).contains(&p_body.z) {
                free_cells.insert(cell);
            }
        }

        // 3. Apply clamped log-odds updates.
        for cell in &hit_cells {
            let entry = self.cells.entry(*cell).or_insert(0.0);
            *entry = (*entry + LOG_ODDS_HIT).clamp(LOG_ODDS_MIN, LOG_ODDS_MAX);
        }
        for cell in free_cells {
            if hit_cells.contains(&cell) {
                continue;
            }
            if let Some(entry) = self.cells.get_mut(&cell) {
                *entry = (*entry - LOG_ODDS_MISS).clamp(LOG_ODDS_MIN, LOG_ODDS_MAX);
            }
        }

        // 4. Rebuild world-frame obstacle list from cells above threshold.
        self.map.obstacles.clear();
        for (&cell, &log_odds) in &self.cells {
            if log_odds >= OCCUPIED_THRESHOLD {
                self.map.obstacles.push(self.cell_to_world(cell));
            }
        }

        &self.map
    }
}
