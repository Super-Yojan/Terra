//! The twist comes from the base station (`remote/cmd`), so a controller
//! can be written in any language on a laptop without rebuilding the rover.
//! A prototyping tool: the network delay is inside the control loop.
//!
//! Each tick the task runner (`tasks.rs`) publishes what the rover sees
//! (`RemoteState`: pose, twist, path, obstacles) and stores the newest
//! answer in [`Remote::command`]. With no answer for `COMMAND_LEASE` the
//! task stops and asks for attention.
use terra_comms::GoalKind;

use super::LocalPlanner;
use crate::planning::registry::ControllerSpec;
use crate::planning::{Goal, Path, PoseGoal};
use crate::{
    estimation::State,
    mapping::Map,
    vehicle::{Twist, VehicleModel},
};

#[derive(Default)]
pub struct Remote {
    /// The base station's newest twist; `None` when it has gone silent.
    pub command: Option<Twist>,
}

impl Remote {
    pub fn spec() -> ControllerSpec {
        ControllerSpec {
            id: "remote",
            version: "1",
            description: "Twist from the base station on remote/cmd (prototyping)",
            goals: vec![GoalKind::Local],
            params: vec![],
            make: |_| Box::new(Remote::default()),
        }
    }
}

impl LocalPlanner for Remote {
    fn follow(
        &mut self,
        _state: &State,
        _map: &Map,
        _goal: &Goal,
        _path: &Path,
        vehicle: &dyn VehicleModel,
    ) -> Twist {
        match self.command {
            Some(twist) => vehicle.clamp(twist),
            None => Twist::STOP,
        }
    }

    /// The remote program decides how long it takes.
    fn timeout(&self, _: &PoseGoal, _: &State, _: &Path, _: &dyn VehicleModel) -> Option<f64> {
        None
    }

    /// The remote program decides what progress is.
    fn stuck_after(&self) -> Option<f64> {
        None
    }

    fn as_remote(&mut self) -> Option<&mut Remote> {
        Some(self)
    }
}
