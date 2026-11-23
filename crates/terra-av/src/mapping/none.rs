//! Open space: no obstacles. For teleop and controller tests.
use super::{Map, Mapper};
use crate::{estimation::State, sense::SensorFrame};

#[derive(Default)]
pub struct NoMap {
    /// Always empty, and its `revision` never changes.
    empty: Map,
}

impl Mapper for NoMap {
    fn update(&mut self, _frame: &SensorFrame, _state: &State) -> &Map {
        &self.empty
    }
}
