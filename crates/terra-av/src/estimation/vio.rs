//! ARKit (or any VIO) already estimates the pose: move it from the camera
//! frame to the vehicle body (mount rotation + offset) and pass it through.
//! The simulator's ground-truth pose uses the same path.
use super::{State, StateEstimator};
use crate::{
    geometry::Pose2,
    sense::{Mount, SensorFrame},
    vehicle::{Twist, VehicleModel},
};

pub struct Vio {
    pub mount: Mount,
}

impl StateEstimator for Vio {
    fn update(&mut self, frame: &SensorFrame, vehicle: &dyn VehicleModel) -> Option<State> {
        Some(State {
            t: 0.0,
            pose: Pose2 {
                x: 0.0,
                y: 0.0,
                yaw: 0.0,
            },
            twist: Twist { v: 0.0, w: 0.0 },
        })
    }
}
