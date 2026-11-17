//! Pass a stick's twist through, within the vehicle's limits.
use super::{Goal, Planner};
use crate::{estimation::State, mapping::Map, vehicle::{Twist, VehicleModel}};

pub struct Teleop;

impl Planner for Teleop {
    fn plan(&mut self, state: &State, map: &Map, goal: &Goal, vehicle: &dyn VehicleModel) -> Twist { todo!() }
}
