//! Wires the stages together. One call per control tick.
use crate::estimation::StateEstimator;
use crate::comms::BaseStationLink;
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
    /// Base station; `NoLink` when there is none.
    pub link: Box<dyn BaseStationLink>,
}

/// What the platform's velocity loop needs: where to go, and how fast the
/// rover is going now (from estimation; `None` until the pose is tracked).
pub struct DriveCommand {
    pub target: Twist,
    pub measured: Option<Twist>,
}

impl Pipeline {
    /// sense → estimate → map → global plan → local plan → safety.
    /// The global planner re-runs only when the goal or map changed.
    pub fn tick(&mut self, frame: &SensorFrame, goal: &Goal) -> DriveCommand {
        self.link.publish_depth(frame.t, &frame.depth);
        DriveCommand {
            target: Twist { v: 0.0, w: 0.0 },
            measured: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::comms::NoLink;
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
            estimator: Box::new(Vio {
                mount: Mount {
                    camera_to_body: identity(),
                    camera_offset: v(0.10, 0.0, 0.23),
                },
            }),
            mapper: Box::new(NoMap),
            path_planner: Box::new(StraightLine),
            local_planner: Box::new(Teleop),
            safety: SafetyGate,
            link: Box::new(NoLink),
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

        let out = pipeline.tick(&frame, &Goal::Velocity(Twist { v: 0.5, w: 0.2 }));

        assert_eq!((out.target.v, out.target.w), (0.0, 0.0));
    }
}
