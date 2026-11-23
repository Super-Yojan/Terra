//! Global planning: find a route from the current pose to the goal over the
//! whole map. Re-plans when the goal or the map changes (see `Navigator`).
mod hybrid_astar;
mod straight_line;
pub use hybrid_astar::HybridAStar;
pub use straight_line::StraightLine;

use super::{Goal, Path};
use crate::{estimation::State, mapping::Map, vehicle::VehicleModel};

pub trait PathPlanner: Send {
    /// A route from `state.pose` to `goal`, or `None` if there is none.
    /// `Goal::None` and `Goal::Velocity` need no route: an empty path.
    fn plan(
        &mut self,
        state: &State,
        map: &Map,
        goal: &Goal,
        vehicle: &dyn VehicleModel,
    ) -> Option<Path>;
}
