//! The path is the goal itself: no obstacle search. For open ground and the
//! roadmap demos, where the local planner does all the work.
use super::PathPlanner;
use crate::planning::{Goal, Path};
use crate::{estimation::State, mapping::Map, vehicle::VehicleModel};

pub struct StraightLine;

impl PathPlanner for StraightLine {
    fn plan(&mut self, state: &State, map: &Map, goal: &Goal, vehicle: &dyn VehicleModel) -> Path { todo!() }
}
