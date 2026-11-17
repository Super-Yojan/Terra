//! Stage 4, Planning: what should I do next. Every planner turns state, map
//! and goal into a target twist.
mod teleop;
mod drive_pose;
mod dwa;
pub use teleop::Teleop;
pub use drive_pose::DrivePose;
pub use dwa::Dwa;

use crate::estimation::State;
use crate::geometry::Pose2;
use crate::mapping::Map;
use crate::vehicle::{Twist, VehicleModel};

pub enum Goal {
    None,
    /// Drive at this twist (a stick).
    Velocity(Twist),
    /// Reach this pose.
    Pose(Pose2),
}

pub trait Planner {
    fn plan(&mut self, state: &State, map: &Map, goal: &Goal, vehicle: &dyn VehicleModel) -> Twist;
}
