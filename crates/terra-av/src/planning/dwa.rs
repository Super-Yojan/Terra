//! Dynamic Window Approach (roadmap 5): sample reachable twists, simulate
//! each with the vehicle model, score heading, clearance and speed.
use super::{Goal, Planner};
use crate::{estimation::State, mapping::Map, vehicle::{Twist, VehicleModel}};

pub struct Dwa { pub horizon: f64, pub samples: usize, pub weights: DwaWeights }
pub struct DwaWeights { pub heading: f64, pub clearance: f64, pub speed: f64 }

impl Planner for Dwa {
    fn plan(&mut self, state: &State, map: &Map, goal: &Goal, vehicle: &dyn VehicleModel) -> Twist { todo!() }
}
