//! Wires the stages together. One call per control tick.
use crate::comms::Link;
use crate::estimation::StateEstimator;
use crate::execution::SafetyGate;
use crate::mapping::Mapper;
use crate::planning::{Goal, global::PathPlanner, local::LocalPlanner};
use crate::sense::SensorFrame;
use crate::vehicle::{Twist, VehicleModel};

pub struct Pipeline {
    pub vehicle: Box<dyn VehicleModel>,
    pub estimator: Box<dyn StateEstimator>,
    pub mapper: Box<dyn Mapper>,
    pub path_planner: Box<dyn PathPlanner>,
    pub local_planner: Box<dyn LocalPlanner>,
    pub safety: SafetyGate,
    /// Base station; `Link::none()` when there is none.
    pub link: Link,
}

/// What the platform's velocity loop needs: where to go, and how fast the
/// rover is going now (from estimation; `None` until the pose is tracked).
pub struct DriveCommand {
    pub target: Twist,
    pub measured: Option<Twist>,
}

impl Pipeline {
    /// base-station commands → estimate → map → global plan → local plan →
    /// safety → publish. The global planner re-runs only when the goal or map
    /// changed.
    pub fn tick(&mut self, mut frame: SensorFrame, goal: &Goal) -> DriveCommand {
        // The remote e-stop joins the phone's; SafetyGate reads `frame.estop`.
        // Base-station teleop drives only when the phone gives no goal.
        let remote = self.link.commands(frame.t);
        frame.estop |= remote.estop;
        let goal = match (goal, remote.teleop) {
            (Goal::None, Some(t)) => Goal::Velocity(Twist { v: t.v, w: t.w }),
            _ => *goal,
        };

        let state = self.estimator.update(&frame, self.vehicle.as_ref());

        // TODO: map → plan → follow(goal) → clamp → SafetyGate. Those stages
        // are still todo!(), so the rover holds still for now.

        self.link
            .publish(frame.t, state.as_ref(), &frame.depth, frame.estop);

        // Only a velocity goal (phone or base-station teleop) drives for now.
        // It runs even without a tracked pose: a human is in the loop, and
        // with `measured: None` the platform's velocity loop is feedforward
        // only. Every other goal holds still until planning lands.
        let target = match goal {
            Goal::Velocity(twist) => twist,
            _ => Twist { v: 0.0, w: 0.0 },
        };
        DriveCommand {
            target: self.safety.check(target, &frame),
            // Feedback for the velocity loop: the estimator's body twist,
            // `None` while untracked.
            measured: state.as_ref().map(|s| s.twist),
        }
    }

    /// Publish a serialized ARKit `ARWorldMap` and its ARKit-world feature points
    /// to the base station.
    pub fn publish_world_map(
        &mut self,
        t: f64,
        points_arkit: &[crate::geometry::Vec3],
        data: Vec<u8>,
    ) {
        self.link.publish_world_map(t, points_arkit, data);
    }

    /// Publish one incremental ARKit `ARMeshAnchor` chunk (vertices in ARKit world `A`,
    /// flat triangle indices, and per-face `ARMeshClassification` bytes) to the base station.
    pub fn publish_mesh_chunk(
        &mut self,
        t: f64,
        id: String,
        vertices_arkit: &[crate::geometry::Vec3],
        indices: &[u32],
        classes: Vec<u8>,
    ) {
        self.link
            .publish_mesh_chunk(t, id, vertices_arkit, indices, classes);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::comms::Link;
    use crate::estimation::Vio;
    use crate::geometry::{Rotation3, Vec3};
    use crate::mapping::NoMap;
    use crate::planning::{global::StraightLine, local::Teleop};
    use crate::sense::{GpsFix, Mount, VioPose};
    use crate::vehicle::{DiffDrive, Limits};

    #[test]
    fn estop_overrides_a_stick_command() {
        let v = |x, y, z| Vec3 { x, y, z };
        let identity =
            || Rotation3::from_matrix([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);

        //+USER_REVIEW_START Line 53-72
        // This is a verry bad way of writing code. A configuration would be used multiple times,
        // so why is it hard coded
        //+USER_REVIEW_END
        let mut pipeline = Pipeline {
            vehicle: Box::new(DiffDrive {
                track: 0.5,
                limits: Limits {
                    max_wheel_speed: 0.8,
                    max_accel: 1.0,
                    max_yaw_accel: 2.0,
                },
            }),
            estimator: Box::new(Vio::new(Mount {
                camera_to_body: identity(),
                camera_offset: v(0.10, 0.0, 0.23),
            })),
            mapper: Box::new(NoMap),
            path_planner: Box::new(StraightLine),
            local_planner: Box::new(Teleop),
            safety: SafetyGate,
            link: Link::none(),
        };
        let frame = SensorFrame {
            t: 48213.512,
            estop: true,
            vio: Some(VioPose {
                position: v(-0.42, 0.01, -3.05),
                rotation: identity(),
                tracked: true,
            }),
            gyro: Some(v(0.12, 0.003, -0.002)),
            gps: Some(GpsFix {
                latitude: 37.774_912,
                longitude: -122.419_418,
                accuracy: 4.8,
            }),
            depth: vec![v(0.0, -0.30, -1.50), v(0.25, -0.10, -1.20)],
        };

        let out = pipeline.tick(frame, &Goal::Velocity(Twist { v: 0.5, w: 0.2 }));

        assert_eq!((out.target.v, out.target.w), (0.0, 0.0));
    }
}
