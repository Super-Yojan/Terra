//! Stage 3, Perception and mapping: what is around me.
mod none;
mod occupancy;
pub use none::NoMap;
pub use occupancy::OccupancyGrid;

use crate::estimation::State;
use crate::geometry::Vec2;
use crate::sense::SensorFrame;

/// What the planner sees: occupied points in the world frame.
pub struct Map { pub obstacles: Vec<Vec2> }

pub trait Mapper: Send {
    fn update(&mut self, frame: &SensorFrame, state: &State) -> &Map;
}
