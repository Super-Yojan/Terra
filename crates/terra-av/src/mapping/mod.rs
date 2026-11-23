//! Stage 3, Perception and mapping: what is around me.
//!
//! ```text
//! SensorFrame (depth, camera frame) + State (pose) ─► Mapper ─► Map (world-frame obstacles)
//! ```
//!
//! A `Mapper` turns each frame's depth points into world-frame obstacles,
//! using the estimated pose, and keeps whatever memory it needs between
//! ticks. The planners read the resulting `Map`.
//!
//! - [`NoMap`]: open space, for teleop and controller tests.
//! - [`OccupancyGrid`]: accumulates LiDAR points into a world-frame grid.
mod none;
mod occupancy;
pub use none::NoMap;
pub use occupancy::OccupancyGrid;

use crate::estimation::State;
use crate::geometry::{Vec2, Vec3};
use crate::sense::SensorFrame;

/// What the planner sees: occupied and observed-free points in the world frame.
#[derive(Default)]
pub struct Map {
    /// The centre of every occupied grid cell.
    pub obstacles: Vec<Vec2>,
    /// The centre of every observed-free grid cell (used by frontier exploration).
    pub free: Vec<Vec2>,
    /// Side of one grid cell (m). Zero for a mapper without a grid.
    pub resolution: f64,
    /// Goes up by one every time `obstacles` changes, so a planner can tell
    /// "same map as last time" without comparing the points.
    pub revision: u64,
}

pub trait Mapper: Send {
    fn update(&mut self, frame: &SensorFrame, state: &State) -> &Map;

    /// Ingest one incremental ARKit `ARMeshAnchor` chunk (vertices in Terra
    /// world frame `W`, Z-up; flat triangle `indices`; per-face
    /// `ARMeshClassification` bytes). Empty `indices` removes the chunk.
    fn update_mesh(
        &mut self,
        _id: &str,
        _vertices_world: &[Vec3],
        _indices: &[u32],
        _classes: &[u8],
    ) {
    }
}
