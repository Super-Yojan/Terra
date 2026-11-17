//! LiDAR depth points (camera frame) → body frame via the mount → world via
//! the state, then accumulated into a world-frame grid. Points near the
//! floor are free space; points above it are obstacles.
use super::{Map, Mapper};
use crate::{estimation::State, sense::{Mount, SensorFrame}};

pub struct OccupancyGrid { pub mount: Mount, pub resolution: f64 }

impl Mapper for OccupancyGrid {
    fn update(&mut self, frame: &SensorFrame, state: &State) -> &Map { todo!() }
}
