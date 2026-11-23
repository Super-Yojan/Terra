//! The base-station link, through `terra-comms` (wire contract, keys and
//! messages: `crates/terra-comms/API.md`). Not a pipeline stage: each tick the
//! pipeline reads the base station's commands and publishes what the rover
//! knows. Nothing here blocks the tick.
//!
//! What the pipeline does with the link:
//! - publishes `Capabilities` once (the controller registry; the link
//!   repeats it), `Pose` every tick, `Depth` when there are points,
//!   `Occupancy` when the obstacle map changes (the link repeats it),
//!   `PlannedPath` when the planned route changes (the link repeats it),
//!   `Status` / `TaskStatus` when they change (the link repeats them), and
//!   `RemoteState` while the `remote` controller runs;
//! - reads the remote e-stop, the base station's teleop and `remote/cmd`
//!   twists, and the tasks and task commands that arrived since last tick.
//!
//! [`Link::none`] (no `[comms]` table, or a build without the `zenoh`
//! feature) sends nothing and receives nothing.
use terra_comms::{
    Depth, MeshChunk, Occupancy, PlannedPath, Pose, RemoteState, Status, Task, TaskCommand,
    TaskStatus, Uplink, WorldMap,
};

use crate::estimation::{GpsAnchor, State};
use crate::geometry::{Pose2, Vec3};
use crate::mapping::Map;
use crate::vehicle::Twist;

/// What arrived from the base station, read once per tick.
#[derive(Default)]
pub struct Commands {
    /// Remote e-stop, latched by the link until the base releases it.
    pub estop: bool,
    /// The base station's teleop twist; `None` after `COMMAND_LEASE` of silence.
    pub teleop: Option<Twist>,
    /// The twist for the `remote` controller; `None` after `COMMAND_LEASE` of silence.
    pub remote: Option<Twist>,
    /// A new or updated task, if one arrived (the newest wins).
    pub task: Option<Task>,
    /// Task commands (resume, pause, clear), in arrival order.
    pub task_commands: Vec<TaskCommand>,
}

/// What the rover reports after a tick.
pub struct Report<'a> {
    /// Frame time (s).
    pub t: f64,
    /// The estimator's output; `None` when untracked.
    pub state: Option<&'a State>,
    /// The frame's depth points, camera frame.
    pub depth: &'a [Vec3],
    /// The mapper's output; `None` when untracked (the map was not updated).
    pub map: Option<&'a Map>,
    /// The global planner's current route in the world frame (empty when idle).
    pub planned_path: &'a [Pose2],
    /// Any e-stop engaged (the phone's, or a base station's).
    pub estopped: bool,
    pub gps_anchor: Option<GpsAnchor>,
    pub task: TaskStatus,
    /// Only while the `remote` controller runs.
    pub remote_state: Option<RemoteState>,
}

pub struct Link {
    #[cfg(feature = "zenoh")]
    rover: Option<terra_comms::RoverLink>,
    depth_sequence: u64,
    world_map_sequence: u64,
    /// `Map::revision` of the obstacle map last sent, to send it on change only.
    last_map_revision: Option<u64>,
    /// Last planned route sent, to send it on change only.
    last_planned_path: Option<Vec<terra_comms::Pose2>>,
    /// Last pose, re-sent as untracked while the estimator has none.
    last_pose: Pose,
    /// Last `Status` and `TaskStatus` sent (with `t` zeroed), to send them
    /// on change only.
    last_status: Option<Status>,
    last_task_status: Option<TaskStatus>,
}

impl Link {
    /// No base station.
    pub fn none() -> Self {
        Self {
            #[cfg(feature = "zenoh")]
            rover: None,
            depth_sequence: 0,
            world_map_sequence: 0,
            last_map_revision: None,
            last_planned_path: None,
            last_pose: Pose { t: 0.0, x: 0.0, y: 0.0, yaw: 0.0, tracked: false },
            last_status: None,
            last_task_status: None,
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

    /// Read what arrived from the base station. Never waits.
    pub fn commands(&mut self) -> Commands {
        #[cfg(feature = "zenoh")]
        if let Some(rover) = &self.rover {
            let twist = |teleop: terra_comms::Teleop| Twist { v: teleop.v, w: teleop.w };
            return Commands {
                estop: rover.estop(),
                teleop: rover.teleop().map(twist),
                remote: rover.remote_cmd().map(twist),
                task: rover.take_task(),
                task_commands: std::iter::from_fn(|| rover.take_task_command()).collect(),
            };
        }
        Commands::default()
    }

    /// Publish this tick's report. Only queues; never waits for the network.
    pub fn publish(&mut self, report: Report) {
        if !self.is_connected_link() {
            return;
        }
        let t = report.t;

        // Pose every tick: the base station uses it to tell the rover is alive.
        self.last_pose = match report.state {
            Some(s) => Pose { t, x: s.pose.x, y: s.pose.y, yaw: s.pose.yaw, tracked: true },
            None => Pose { t, tracked: false, ..self.last_pose.clone() },
        };
        self.send(&self.last_pose);

        if !report.depth.is_empty() {
            self.depth_sequence += 1;
            let points = report.depth.iter().map(|p| [p.x as f32, p.y as f32, p.z as f32]).collect();
            self.send(&Depth { t, sequence: self.depth_sequence, points });
        }

        // The obstacle map, when it changed. A mapper without a grid
        // (`NoMap`, resolution 0) has nothing to show.
        if let Some(map) = report.map
            && map.resolution > 0.0
            && self.last_map_revision != Some(map.revision)
        {
            let cells = map.obstacles.iter().map(|cell| [cell.x as f32, cell.y as f32]).collect();
            self.send(&Occupancy { t, resolution: map.resolution, cells });
            self.last_map_revision = Some(map.revision);
        }

        // The global planner's route, when it changed.
        let planned_poses: Vec<terra_comms::Pose2> = report
            .planned_path
            .iter()
            .map(|p| terra_comms::Pose2 { x: p.x, y: p.y, yaw: p.yaw })
            .collect();
        if self.last_planned_path.as_ref() != Some(&planned_poses) {
            self.send(&PlannedPath { t, poses: planned_poses.clone() });
            self.last_planned_path = Some(planned_poses);
        }

        if let Some(remote_state) = &report.remote_state {
            self.send(remote_state);
        }

        // `Status` and `TaskStatus` on change only; the link repeats them
        // for the base station. They are compared with `t` zeroed, so the
        // clock alone is not a change.
        let status = Status {
            t: 0.0,
            estopped: report.estopped,
            link_ok: self.connected(),
            detail: String::new(),
            gps_anchor: report.gps_anchor.map(|anchor| terra_comms::GpsAnchor {
                latitude: anchor.latitude,
                longitude: anchor.longitude,
                heading: anchor.heading,
                accuracy: anchor.accuracy,
            }),
        };
        if self.last_status.as_ref() != Some(&status) {
            self.send(&Status { t, ..status.clone() });
            self.last_status = Some(status);
        }
        let task_status = TaskStatus { t: 0.0, ..report.task };
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
