//! Drive-to-pose controller (roadmap 4): steer onto a goal position and heading.
use super::{Goal, Planner};
use crate::{estimation::State, mapping::Map, vehicle::{Twist, VehicleModel}};

pub struct DrivePose { pub k_rho: f64, pub k_alpha: f64, pub k_beta: f64 }

impl Planner for DrivePose {
    fn plan(&mut self, state: &State, map: &Map, goal: &Goal, vehicle: &dyn VehicleModel) -> Twist { todo!() }
}
