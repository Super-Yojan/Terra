//! Pass a stick's twist through, within the vehicle's limits.
//!
//! Note that the pipeline applies a live stick (the phone's, or the base
//! station's) itself, before any controller runs, so that a human can drive
//! even without a tracked pose. Picking `teleop` in a task therefore means
//! "no autonomous driving": the rover holds still unless a stick moves it.
use super::LocalPlanner;
use crate::planning::registry::ControllerSpec;
use crate::planning::{Goal, Path, PoseGoal};
use crate::{
    estimation::State,
    mapping::Map,
    vehicle::{Twist, VehicleModel},
};

pub struct Teleop;

impl Teleop {
    pub fn spec() -> ControllerSpec {
        ControllerSpec {
            id: "teleop",
            version: "1",
            description: "Drive from a stick",
            goals: vec![],
            params: vec![],
            make: |_| Box::new(Teleop),
        }
    }
}

impl LocalPlanner for Teleop {
    fn follow(
        &mut self,
        _state: &State,
        _map: &Map,
        goal: &Goal,
        _path: &Path,
        vehicle: &dyn VehicleModel,
    ) -> Twist {
        match goal {
            Goal::Velocity(stick) => vehicle.clamp(*stick),
            Goal::None | Goal::Pose(_) => Twist::STOP,
        }
    }

    /// A human is driving: no time limit.
    fn timeout(&self, _: &PoseGoal, _: &State, _: &Path, _: &dyn VehicleModel) -> Option<f64> {
        None
    }

    /// A human is driving: standing still is not being stuck.
    fn stuck_after(&self) -> Option<f64> {
        None
    }
}
