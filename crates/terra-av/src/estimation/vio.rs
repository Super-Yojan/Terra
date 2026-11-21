//! ARKit (or any VIO) state estimator: turns camera poses into rover body poses.
//!
//! # Frames in this file
//! - **Body (`B`)**: rover FLU frame (`+X` forward, `+Y` left, `+Z` up), origin at the ground pivot.
//! - **Camera (`C`)**: ARKit camera (`+X` right, `+Y` up, `-Z` out of the rear lens).
//! - **ARKit world (`A`)**: gravity-aligned Y-up world (`+Y` up, `-Z` initial forward, `+X` right).
//! - **Terra world (`W`)**: Z-up world (`+X` forward, `+Y` left, `+Z` up, `+yaw` CCW from above).
//!
//! # Transform chain
//! ```text
//! R_{W←B} = R_{W←A} · R_{A←C} · (R_{B←C})ᵀ
//!             │         │          └─ mount.camera_to_body.transpose()   (B → C)
//!             │         └──────────── vio.rotation                       (C → A)
//!             └────────────────────── arkit_to_world                     (A → W)
//!
//! p_B^W = R_{W←A} · vio.position  −  R_{W←B} · mount.camera_offset
//! ```
use std::f64::consts::PI;

use super::{State, StateEstimator};
use crate::{
    geometry::{Pose2, Vec3},
    sense::{Mount, SensorFrame},
    vehicle::{Twist, VehicleModel},
};

pub struct Vio {
    /// Physical phone mount (`camera_to_body` rotation and `camera_offset` in body frame).
    pub mount: Mount,
    /// Previous tracked `(t, pose)` used to finite-difference body twist `(v, w)`.
    prev: Option<(f64, Pose2)>,
}

impl Vio {
    pub fn new(mount: Mount) -> Self {
        Self { mount, prev: None }
    }
}

/// Rotate a vector from ARKit's Y-up world (`A`) into Terra's Z-up world (`W`).
///
/// This is the exact 3×3 proper rotation `R_{W←A}` (`RᵀR = I`, `det = +1`,
/// equivalent to `Rotation3::from_euler(90°, 0°, -90°)`) with its `0`s and `±1`s
/// simplified out:
/// ```text
/// [  0   0  -1 ] [ x_A ]   [ -z_A ]   (ARKit -Z forward → Terra +X forward)
/// [ -1   0   0 ] [ y_A ] = [ -x_A ]   (ARKit -X left    → Terra +Y left)
/// [  0   1   0 ] [ z_A ]   [ +y_A ]   (ARKit +Y up      → Terra +Z up)
/// ```
fn arkit_to_world(v: Vec3) -> Vec3 {
    Vec3 {
        x: -v.z,
        y: -v.x,
        z: v.y,
    }
}

/// Wrap an angle difference into `(-π, π]` so yaw-rate finite differences don't
/// spike when heading crosses `±π`.
fn wrap_angle(mut a: f64) -> f64 {
    while a > PI {
        a -= 2.0 * PI;
    }
    while a <= -PI {
        a += 2.0 * PI;
    }
    a
}

impl StateEstimator for Vio {
    fn update(&mut self, frame: &SensorFrame, _vehicle: &dyn VehicleModel) -> Option<State> {
        // Never invent a pose when VIO is absent or untracked; also clear `prev`
        // so the first frame after re-acquiring tracking doesn't produce a huge twist spike.
        let Some(vio) = &frame.vio else {
            self.prev = None;
            return None;
        };
        if !vio.tracked {
            self.prev = None;
            return None;
        }

        // Apply R_{W←B} = R_{W←A} · R_{A←C} · (R_{B←C})ᵀ right-to-left (B → C → A → W).
        let body_to_cam = self.mount.camera_to_body.transpose();
        let body_to_world = |v: Vec3| -> Vec3 {
            let in_cam = body_to_cam.apply(v);
            let in_arkit = vio.rotation.apply(in_cam);
            arkit_to_world(in_arkit)
        };

        // 1. Heading: push the body's forward unit vector (+X_B) into Terra world.
        let forward_world = body_to_world(Vec3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        });
        let yaw = forward_world.y.atan2(forward_world.x);

        // 2. Body origin (x, y): convert ARKit's camera position to Terra world, then
        //    subtract the mount offset rotated into Terra world. Rotating the offset first
        //    keeps the body pivot stationary during an in-place turn even though the camera
        //    swings in an arc.
        let cam_world = arkit_to_world(vio.position);
        let offset_world = body_to_world(self.mount.camera_offset);
        let pose = Pose2 {
            x: cam_world.x - offset_world.x,
            y: cam_world.y - offset_world.y,
            yaw,
        };

        // 3. Body twist (v, w): finite-difference consecutive tracked poses.
        //    On the first frame (or non-increasing timestamps), report zero twist.
        let twist = match self.prev {
            Some((prev_t, prev_pose)) if frame.t - prev_t > 1e-6 => {
                let dt = frame.t - prev_t;
                let dx = pose.x - prev_pose.x;
                let dy = pose.y - prev_pose.y;
                let dyaw = wrap_angle(pose.yaw - prev_pose.yaw);
                // Dot world velocity (dx/dt, dy/dt) with body forward (cos yaw, sin yaw).
                let v = (dx * pose.yaw.cos() + dy * pose.yaw.sin()) / dt;
                let w = dyaw / dt;
                Twist { v, w }
            }
            _ => Twist { v: 0.0, w: 0.0 },
        };

        self.prev = Some((frame.t, pose));
        Some(State {
            t: frame.t,
            pose,
            twist,
        })
    }
}
