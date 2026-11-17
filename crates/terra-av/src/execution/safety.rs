//! The last step before the twist leaves the pipeline: if the e-stop is
//! pressed, stop. Nothing else.
use crate::{sense::SensorFrame, vehicle::Twist};

pub struct SafetyGate;

impl SafetyGate {
    /// `frame.estop` set → zero twist; otherwise `target` unchanged.
    pub fn check(&mut self, target: Twist, frame: &SensorFrame) -> Twist { todo!() }
}
