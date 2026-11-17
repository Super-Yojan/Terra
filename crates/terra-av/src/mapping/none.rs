//! Open space: no obstacles. For teleop and controller tests.
use super::{Map, Mapper};
use crate::{estimation::State, sense::SensorFrame};

pub struct NoMap;

impl Mapper for NoMap {
    fn update(&mut self, frame: &SensorFrame, state: &State) -> &Map { todo!() }
}
