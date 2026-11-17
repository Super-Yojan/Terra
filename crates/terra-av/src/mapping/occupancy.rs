//! Obstacle returns accumulated into a world-frame grid.
use super::{Map, Mapper};
use crate::{estimation::State, sense::SensorFrame};

pub struct OccupancyGrid;

impl Mapper for OccupancyGrid {
    fn update(&mut self, frame: &SensorFrame, state: &State) -> &Map { todo!() }
}
