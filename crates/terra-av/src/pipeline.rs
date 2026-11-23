//! Wires the stages together. One call per control tick.
//!
//! ```text
//! SensorFrame, phone goal
//!   │  + base station: e-stop, sticks, tasks            (link.commands)
//!   ▼
//! 1 · estimator.update ─────────► State, or None while the pose is lost
//! 0 · tasks.set_task / command    (what arrived; needs 1 for the GPS anchor)
//! 2 · mapper.update ────────────► Map                   (only with a State)
//! 3 · path planner ─┐
//! 4 · controller ───┴───────────► wanted twist          (see `tick`: who drives)
//! 5 · vehicle.clamp, limit_acceleration                 ┐ fixed chain: nothing
//! 6 · safety.check (e-stop) ────► DriveCommand.target   ┘ upstream can skip it
//! 7 · link.publish                (pose, depth, status, task status)
//! ```
use crate::comms::{Link, Report};
use crate::estimation::{StateEstimator, arkit_to_world};
use crate::execution::{SafetyGate, limit_acceleration};
use crate::mapping::Mapper;
use crate::planning::{Goal, Navigator};
use crate::sense::SensorFrame;
use crate::tasks::{self, Tasks};
use crate::vehicle::{Twist, VehicleModel};

/// The longest time step the acceleration limit credits (s). The tick runs
/// at 20 Hz; after a stall the target must not jump by the whole gap.
const MAX_TICK_S: f64 = 0.1;

pub struct Pipeline {
    pub vehicle: Box<dyn VehicleModel>,
    pub estimator: Box<dyn StateEstimator>,
    pub mapper: Box<dyn Mapper>,
    /// The configured path planner and controller (`terra-av.toml`), used
    /// for a pose goal from the phone.
    pub navigator: Navigator,
    /// The base station's task, with the planner and controller it names.
    pub tasks: Tasks,
    pub safety: SafetyGate,
    /// Base station; `Link::none()` when there is none.
    pub link: Link,
    /// The last target sent out and its frame time, for the acceleration limit.
    pub last_target: Option<(f64, Twist)>,
    /// Whether the dump gate is currently commanded open.
    pub gate_open: bool,
    /// What the last tick saw, for the phone's screen.
    pub summary: Summary,
}

/// What the platform's velocity loop needs: where to go, and how fast the
/// rover is going now (from estimation; `None` until the pose is tracked).
pub struct DriveCommand {
    pub target: Twist,
    pub measured: Option<Twist>,
    /// True when the dump gate servo should be held at its open position.
    pub gate_open: bool,
}

/// The last tick in a few words, for a status line on the phone.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Summary {
    /// The estimator has a pose.
    pub tracked: bool,
    /// Any e-stop is engaged (the phone's, or a base station's).
    pub estopped: bool,
    /// The base station's router is connected.
    pub base_connected: bool,
    /// True when the dump gate is currently commanded open.
    pub gate_open: bool,
    /// E.g. `task 7 · dwa · running, 3.2 m to go`, or `no task`.
    pub task: String,
}

impl Pipeline {
    pub fn tick(&mut self, mut frame: SensorFrame, phone_goal: &Goal) -> DriveCommand {
        let t = frame.t;
        let vehicle = self.vehicle.as_ref();

        // The base station's inputs. The remote e-stop joins the phone's.
        let remote = self.link.commands();
        frame.estop |= remote.estop;
        if let Some(open) = remote.gate {
            self.gate_open = open;
        }
        if frame.estop {
            self.gate_open = false;
        }

        // 1 · Where am I. `None`: no trustworthy pose right now.
        let state = self.estimator.update(&frame, vehicle);
        let gps_anchor = self.estimator.gps_anchor();

        // 0 · Tasks and task commands that arrived, applied between ticks.
        if let Some(task) = remote.task {
            self.tasks.set_task(task, gps_anchor.as_ref(), t);
        }
        for command in remote.task_commands {
            self.tasks.command(command, frame.estop, t);
        }

        // 2 · What is around me. Needs the pose to place the depth points.
        let tracked = match &state {
            Some(state) => Some((state, self.mapper.update(&frame, state))),
            None => None,
        };

        // 3–4 · The task's planner and controller. Called every tick, also
        // when something else drives below, so that the task's state
        // (reached, timeout, stuck, …) stays up to date.
        let task = self.tasks.step(&tasks::Tick {
            t,
            tracked,
            vehicle,
            estop: frame.estop,
            gps_anchor,
            remote_command: remote.remote,
        });

        // Who drives this tick. The phone's own goal comes first: it is a
        // human standing at the rover. Then the base station's stick,
        // unless an autonomous task is driving. Then the task.
        //
        // A stick drives even without a tracked pose: a human is in the
        // loop, and with `measured: None` the platform's velocity loop
        // runs feedforward only. Everything else holds still without one.
        let base_stick = if self.tasks.is_driving() {
            None
        } else {
            remote.teleop
        };
        let wanted = match (phone_goal, base_stick) {
            (Goal::Velocity(stick), _) => *stick,
            (Goal::Pose(goal), _) => match tracked {
                // No route to the goal: hold still.
                Some((state, map)) => self
                    .navigator
                    .drive(state, map, goal, vehicle)
                    .unwrap_or(Twist::STOP),
                None => Twist::STOP,
            },
            (Goal::None, Some(stick)) => stick,
            (Goal::None, None) => task.twist,
        };

        // 5 · Within the wheel-speed limit, and speeding up no faster than
        // the vehicle can. A gap longer than MAX_TICK_S counts as MAX_TICK_S.
        let (last_t, last_target) = self.last_target.unwrap_or((t, Twist::STOP));
        let dt = (t - last_t).clamp(0.0, MAX_TICK_S);
        let reachable = vehicle.clamp(wanted);
        let smooth = limit_acceleration(last_target, reachable, &vehicle.limits(), dt);

        // 6 · E-stop. The last word on the target.
        let target = self.safety.check(smooth, &frame);
        self.last_target = Some((t, target));

        // 7 · Tell the base station.
        self.summary = Summary {
            tracked: state.is_some(),
            estopped: frame.estop,
            base_connected: self.link.connected(),
            gate_open: self.gate_open,
            task: self.tasks.summary(),
        };
        let planned_path = if matches!(phone_goal, Goal::Pose(_)) {
            self.navigator.path()
        } else {
            self.tasks.path()
        };
        self.link.publish(Report {
            t,
            state: state.as_ref(),
            depth: &frame.depth,
            map: tracked.map(|(_state, map)| map),
            planned_path,
            estopped: frame.estop,
            gate_open: self.gate_open,
            gps_anchor,
            task: self.tasks.status(t),
            remote_state: task.remote_state,
        });

        DriveCommand {
            target,
            // Feedback for the velocity loop: the estimator's body twist,
            // `None` while untracked.
            measured: state.as_ref().map(|s| s.twist),
            gate_open: self.gate_open,
        }
    }

    /// Command the dump gate open or closed locally (from the phone UI).
    pub fn set_gate_open(&mut self, open: bool) {
        self.gate_open = open;
    }

    /// Ask the base station to send back its saved shared `ARWorldMap`.
    pub fn request_world_map(&mut self, t: f64, auto: bool) {
        self.link.request_world_map(t, auto);
    }

    /// Take the latest shared `ARWorldMap` (`NSKeyedArchiver` bytes) pushed by
    /// the base station for ARKit relocalization, if any.
    pub fn take_load_world_map(&self) -> Option<Vec<u8>> {
        self.link.take_load_world_map().map(|m| m.data)
    }

    /// Returns `true` if the base station asked this rover to capture and
    /// upload its current `ARWorldMap`.
    pub fn take_capture_world_map(&self) -> bool {
        self.link.take_capture_world_map()
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

    /// Ingest one incremental ARKit `ARMeshAnchor` chunk into the local mapper
    /// and publish it to the base station. Vertices arrive in ARKit world `A`
    /// (Y-up) and are converted to Terra world `W` (Z-up) for the mapper.
    pub fn publish_mesh_chunk(
        &mut self,
        t: f64,
        id: String,
        vertices_arkit: &[crate::geometry::Vec3],
        indices: &[u32],
        classes: Vec<u8>,
        colors: Vec<u8>,
    ) {
        if !id.is_empty() {
            let vertices_world: Vec<crate::geometry::Vec3> =
                vertices_arkit.iter().map(|&v| arkit_to_world(v)).collect();
            self.mapper
                .update_mesh(&id, &vertices_world, indices, &classes);
        }
        self.link
            .publish_mesh_chunk(t, id, vertices_arkit, indices, classes, colors);
    }

    /// Drain queued inbound WebRTC FPV signaling messages from the base station.
    pub fn take_fpv_signals(&self) -> Vec<(String, String)> {
        self.link.take_fpv()
    }

    /// Publish one outbound WebRTC FPV signaling message to the base station.
    pub fn publish_fpv_signal(&self, topic: &str, payload: String) {
        self.link.publish_fpv(topic, payload);
    }

    /// Whether the base-station router is currently connected.
    pub fn base_connected(&self) -> bool {
        self.link.connected()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::PipelineConfig;
    use crate::geometry::{Rotation3, Vec3};
    use crate::sense::{GpsFix, VioPose};

    /// The app's own configuration, without its base-station link.
    fn app_pipeline() -> Pipeline {
        let toml = include_str!("../../../apps/ios/TerraPhone/Resources/terra-av.toml");
        let without_comms = toml.split("[comms]").next().unwrap();
        PipelineConfig::from_toml(without_comms)
            .unwrap()
            .build(None)
            .unwrap()
    }

    #[test]
    fn estop_overrides_a_stick_command() {
        let v = |x, y, z| Vec3 { x, y, z };
        let mut pipeline = app_pipeline();
        let frame = SensorFrame {
            t: 48213.512,
            estop: true,
            vio: Some(VioPose {
                position: v(-0.42, 0.01, -3.05),
                rotation: Rotation3::from_matrix([
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ]),
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
