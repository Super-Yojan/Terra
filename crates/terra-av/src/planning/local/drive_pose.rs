//! Drive-to-pose controller (roadmap 4): steer onto the next path pose,
//! position and heading.
use super::LocalPlanner;
use crate::planning::{Goal, Path};
use crate::{estimation::State, mapping::Map, vehicle::{Twist, VehicleModel}};

pub struct DrivePose { pub k_rho: f64, pub k_alpha: f64, pub k_beta: f64 }

impl LocalPlanner for DrivePose {
    fn follow(&mut self, state: &State, map: &Map, goal: &Goal, path: &Path, vehicle: &dyn VehicleModel) -> Twist { todo!() }
}
