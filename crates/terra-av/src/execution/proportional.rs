//! Proportional controller (roadmap 3): per-wheel speed error → effort.
use super::{Controller, WheelCommand};
use crate::{estimation::State, vehicle::{Twist, VehicleModel}};

pub struct Proportional { pub kp: f64 }

impl Controller for Proportional {
    fn command(&mut self, target: Twist, state: &State, vehicle: &dyn VehicleModel, dt: f64) -> WheelCommand { todo!() }
}
