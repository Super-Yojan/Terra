//! Local planning: pick the twist to command this tick, following the path
//! and avoiding obstacles close by.
mod teleop;
mod drive_pose;
mod dwa;
pub use teleop::Teleop;
pub use drive_pose::DrivePose;
pub use dwa::{Dwa, DwaWeights};

use super::{Goal, Path};
use crate::{estimation::State, mapping::Map, vehicle::{Twist, VehicleModel}};

pub trait LocalPlanner: Send {
    /// `goal` is passed too, so a stick (`Goal::Velocity`) can bypass the path.
    fn follow(&mut self, state: &State, map: &Map, goal: &Goal, path: &Path, vehicle: &dyn VehicleModel) -> Twist;
}
