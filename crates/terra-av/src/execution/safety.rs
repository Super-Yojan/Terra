//! The last check before the motors: stale data, e-stop, tilt. Can only
//! reduce a command, never add to it.
use super::WheelCommand;
use crate::{estimation::State, sense::SensorFrame};

pub struct SafetyGate { pub max_data_age: f64 }

impl SafetyGate {
    pub fn check(&mut self, command: WheelCommand, frame: &SensorFrame, state: &State) -> WheelCommand { todo!() }
}
