//! Dynamic Window Approach (roadmap 5): sample reachable twists, simulate
//! each with the vehicle model, score progress along the path, clearance
//! and speed.
use super::LocalPlanner;
use crate::planning::{Goal, Path};
use crate::{estimation::State, mapping::Map, vehicle::{Twist, VehicleModel}};

pub struct Dwa { pub horizon: f64, pub samples: usize, pub weights: DwaWeights }
pub struct DwaWeights { pub heading: f64, pub clearance: f64, pub speed: f64 }

impl LocalPlanner for Dwa {
    fn follow(&mut self, state: &State, map: &Map, goal: &Goal, path: &Path, vehicle: &dyn VehicleModel) -> Twist { todo!() }
}
