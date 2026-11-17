//! Hybrid A*: A* over (x, y, yaw) that expands drivable arcs from the
//! vehicle model, so the route respects the minimum turning radius.
use super::PathPlanner;
use crate::planning::{Goal, Path};
use crate::{estimation::State, mapping::Map, vehicle::VehicleModel};

pub struct HybridAStar {
    /// Grid cell size for visited-state pruning (m).
    pub resolution: f64,
    /// Number of yaw bins.
    pub yaw_bins: usize,
    /// Length of one expanded arc (m).
    pub step: f64,
    /// >1 trades optimality for speed.
    pub heuristic_weight: f64,
}

impl PathPlanner for HybridAStar {
    fn plan(&mut self, state: &State, map: &Map, goal: &Goal, vehicle: &dyn VehicleModel) -> Path { todo!() }
}
