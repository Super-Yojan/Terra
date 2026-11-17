//! Pass a stick's twist through, within the vehicle's limits.
use super::LocalPlanner;
use crate::planning::{Goal, Path};
use crate::{estimation::State, mapping::Map, vehicle::{Twist, VehicleModel}};

pub struct Teleop;

impl LocalPlanner for Teleop {
    fn follow(&mut self, state: &State, map: &Map, goal: &Goal, path: &Path, vehicle: &dyn VehicleModel) -> Twist { todo!() }
}
