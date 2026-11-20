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
use crate::geometry::Vec2;
use crate::sense::SensorFrame;

/// What the planner sees: occupied points in the world frame.
pub struct Map {
    pub obstacles: Vec<Vec2>,
}

pub trait Mapper: Send {
    fn update(&mut self, frame: &SensorFrame, state: &State) -> &Map;
}
