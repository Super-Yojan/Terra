//! Global planning: find a route from the current pose to the goal over the
//! whole map. Re-plans when the goal or the map changes.
mod straight_line;
mod hybrid_astar;
pub use straight_line::StraightLine;
pub use hybrid_astar::HybridAStar;

use super::{Goal, Path};
use crate::{estimation::State, mapping::Map, vehicle::VehicleModel};

pub trait PathPlanner {
    fn plan(&mut self, state: &State, map: &Map, goal: &Goal, vehicle: &dyn VehicleModel) -> Path;
}
