//! Stage 5, Execution: follow the target twist, then make it safe to send.
mod proportional;
mod safety;
pub use proportional::Proportional;
pub use safety::SafetyGate;

use crate::estimation::State;
use crate::vehicle::{Twist, VehicleModel};

/// Wheel efforts in [-1, 1], ready for the motor driver.
pub struct WheelCommand { pub left: f64, pub right: f64 }

pub trait Controller: Send {
    fn command(&mut self, target: Twist, state: &State, vehicle: &dyn VehicleModel, dt: f64) -> WheelCommand;
}
