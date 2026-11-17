//! The last step before the motors: if the e-stop is pressed, stop.
//! Nothing else.
use super::WheelCommand;
use crate::sense::SensorFrame;

pub struct SafetyGate;

impl SafetyGate {
    /// `frame.estop` set → zero wheels; otherwise `command` unchanged.
    pub fn check(&mut self, command: WheelCommand, frame: &SensorFrame) -> WheelCommand { todo!() }
}
