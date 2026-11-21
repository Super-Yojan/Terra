//! The base-station link, through `terra-comms` (wire contract, keys and
//! messages: `crates/terra-comms/API.md`). Not a pipeline stage: each tick the
//! pipeline reads the base station's commands and publishes what the rover
//! knows. Nothing here blocks the tick.
//!
//! What this build does with the link:
//! - publishes `Capabilities` once (the configured controller and planner;
//!   the link repeats it), `Pose` every tick, `Depth` when there are points,
//!   and `Status` / `TaskStatus` when they change (the link repeats them);
//! - hands the tick the remote e-stop and the base station's teleop twist;
//! - rejects tasks and task commands with a reason, because there is no
//!   controller registry yet, so `TaskStatus` stays `Idle`.
//!
//! [`Link::none`] (no `[comms]` table, or a build without the `zenoh`
//! feature) sends nothing and receives nothing.
use terra_comms::{
    Depth, Goal, MeshChunk, Pose, Rejection, Status, TaskState, TaskStatus, Teleop, Uplink,
    WorldMap,
};

use crate::estimation::State;
use crate::geometry::Vec3;

const NO_TASKS: &str = "this rover build can't run tasks yet (no controller registry)";

/// Base-station commands for one tick.
pub struct Commands {
    /// Remote e-stop, latched by the link until the base releases it.
    pub estop: bool,
    /// The base station's teleop twist; `None` after `COMMAND_LEASE` of silence.
    pub teleop: Option<Teleop>,
}

pub struct Link {
    #[cfg(feature = "zenoh")]
    rover: Option<terra_comms::RoverLink>,
    depth_sequence: u64,
    world_map_sequence: u64,
    /// Last pose, re-sent as untracked while the estimator has none.
    last_pose: Pose,
    last_status: Option<(bool, bool)>,
    last_task_status: Option<TaskStatus>,
    last_rejected: Option<Rejection>,
}

impl Link {
    /// No base station.
    pub fn none() -> Self {
        Self {
            #[cfg(feature = "zenoh")]
            rover: None,
            depth_sequence: 0,
            world_map_sequence: 0,
            last_pose: Pose { t: 0.0, x: 0.0, y: 0.0, yaw: 0.0, tracked: false },
            last_status: None,
            last_task_status: None,
            last_rejected: None,
        }
    }

    /// Connect in the background (returns at once) and announce what this
    /// rover can run. Fails only on a bad config.
    #[cfg(feature = "zenoh")]
    pub fn connect(
        config: &terra_comms::CommsConfig,
        capabilities: &terra_comms::Capabilities,
    ) -> Result<Self, String> {
        let rover = terra_comms::connect(config).and_then(|l| l.rover()).map_err(|e| e.to_string())?;
        rover.publish(capabilities);
        Ok(Self { rover: Some(rover), ..Self::none() })
    }

    /// Read the base station's commands. Tasks and task commands are refused
    /// here (see the module docs).
    pub fn commands(&mut self, t: f64) -> Commands {
        #[cfg(feature = "zenoh")]
        if let Some(rover) = &self.rover {
            while let Some(task) = rover.take_task() {
                self.last_rejected = Some(Rejection { task_id: task.id, reason: NO_TASKS.into(), t });
            }
            while let Some(cmd) = rover.take_task_command() {
                self.last_rejected = Some(Rejection { task_id: cmd.task_id, reason: "no task".into(), t });
            }
            return Commands { estop: rover.estop(), teleop: rover.teleop() };
        }
        Commands { estop: false, teleop: None }
    }

    /// Publish this tick's state. `state` is the estimator's output (`None`
    /// when untracked), `depth` the frame's points in the camera frame.
    pub fn publish(&mut self, t: f64, state: Option<&State>, depth: &[Vec3], estopped: bool) {
        if !self.is_connected_link() {
            return;
        }
        // Pose every tick: the base station uses it to tell the rover is alive.
        self.last_pose = match state {
            Some(s) => Pose { t, x: s.pose.x, y: s.pose.y, yaw: s.pose.yaw, tracked: true },
            None => Pose { t, tracked: false, ..self.last_pose.clone() },
        };
        self.send(&self.last_pose);

        if !depth.is_empty() {
            self.depth_sequence += 1;
            let points = depth.iter().map(|p| [p.x as f32, p.y as f32, p.z as f32]).collect();
            self.send(&Depth { t, sequence: self.depth_sequence, points });
        }

        // On change only; the link repeats them for the base station.
        let status = (estopped, self.connected());
        if self.last_status != Some(status) {
            self.last_status = Some(status);
            let (estopped, link_ok) = status;
            self.send(&Status { t, estopped, link_ok, detail: String::new(), gps_anchor: None });
        }
        let task_status = TaskStatus {
            t: 0.0, // set below, so the change check ignores it
            task_id: None,
            state: TaskState::Idle,
            controller: String::new(),
            controller_version: String::new(),
            params: Default::default(),
            goal: Goal::Stop,
            timeout: None,
            last_rejected: self.last_rejected.clone(),
        };
        if self.last_task_status.as_ref() != Some(&task_status) {
            self.send(&TaskStatus { t, ..task_status.clone() });
            self.last_task_status = Some(task_status);
        }
    }

    /// Publish a serialized ARKit `ARWorldMap` blob plus its feature points
    /// (received in ARKit world `A`, converted to Terra world `W`) to the base station.
    pub fn publish_world_map(&mut self, t: f64, points_arkit: &[Vec3], data: Vec<u8>) {
        if !self.is_connected_link() || (data.is_empty() && points_arkit.is_empty()) {
            return;
        }
        self.world_map_sequence += 1;
        // ARKit world A (Y-up) → Terra world W (Z-up): (x, y, z)_A → (-z, -x, y)_W
        let points = points_arkit
            .iter()
            .filter(|p| p.x.is_finite() && p.y.is_finite() && p.z.is_finite())
            .map(|p| [-p.z as f32, -p.x as f32, p.y as f32])
            .collect();
        self.send(&WorldMap {
            t,
            sequence: self.world_map_sequence,
            points,
            data,
        });
    }

    /// Publish one incremental ARKit `ARMeshAnchor` chunk to the base station.
    /// Converts vertices from ARKit world `A` (Y-up) to Terra world `W` (Z-up).
    /// Empty `vertices_arkit` + `indices` tells the base station that chunk `id` was removed.
    pub fn publish_mesh_chunk(
        &mut self,
        t: f64,
        id: String,
        vertices_arkit: &[Vec3],
        indices: &[u32],
        classes: Vec<u8>,
    ) {
        if !self.is_connected_link() || id.is_empty() {
            return;
        }
        // ARKit world A (Y-up) → Terra world W (Z-up): (x, y, z)_A → (-z, -x, y)_W
        let vertices = vertices_arkit
            .iter()
            .map(|v| [-v.z as f32, -v.x as f32, v.y as f32])
            .collect();
        let faces = indices
            .chunks_exact(3)
            .map(|tri| [tri[0], tri[1], tri[2]])
            .collect();
        self.send(&MeshChunk { t, id, vertices, indices: faces, classes });
    }

    /// Whether the router is connected. For a status light, not for control.
    pub fn connected(&self) -> bool {
        #[cfg(feature = "zenoh")]
        if let Some(rover) = &self.rover {
            return rover.connected();
        }
        false
    }

    fn is_connected_link(&self) -> bool {
        #[cfg(feature = "zenoh")]
        return self.rover.is_some();
        #[cfg(not(feature = "zenoh"))]
        false
    }

    fn send<M: Uplink>(&self, msg: &M) {
        #[cfg(feature = "zenoh")]
        if let Some(rover) = &self.rover {
            rover.publish(msg);
        }
    }
}
