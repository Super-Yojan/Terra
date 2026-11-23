//! Every message on the link. Plain structs; each carries `t`, the sender's
//! clock in seconds. Encodings are in `codec.rs`.
use serde::{Deserialize, Serialize};

use crate::params::{ParamSpec, Params};

// ---------------------------------------------------------------- rover → base

/// What this rover can run. Built once at startup from the controller
/// registry.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Capabilities {
    pub t: f64,
    pub controllers: Vec<StageOption>,
    pub planners: Vec<StageOption>,
    pub default_controller: String,
    /// `git describe` of the rover build.
    pub software: String,
    /// Physical rover body `[length, width]` in metres (`None` on older rovers).
    #[serde(default)]
    pub footprint: Option<[f64; 2]>,
}

impl Capabilities {
    pub fn controller(&self, id: &str) -> Option<&StageOption> {
        self.controllers.iter().find(|c| c.id == id)
    }

    pub fn planner(&self, id: &str) -> Option<&StageOption> {
        self.planners.iter().find(|p| p.id == id)
    }
}

/// One selectable controller or planner.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct StageOption {
    pub id: String,
    /// Bumped when behaviour changes.
    pub version: String,
    pub description: String,
    pub params: Vec<ParamSpec>,
    /// Goal kinds it accepts.
    pub goals: Vec<GoalKind>,
}

impl StageOption {
    pub fn accepts(&self, kind: GoalKind) -> bool {
        self.goals.contains(&kind)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GoalKind {
    /// World-frame `Goal::Pose`.
    Local,
    /// Raw `Goal::Gps`.
    Gps,
    /// Autonomous frontier exploration (`Goal::Explore`).
    Explore,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TaskStatus {
    pub t: f64,
    /// `None` when `Idle`.
    pub task_id: Option<u64>,
    pub state: TaskState,
    /// What is actually driving.
    pub controller: String,
    pub controller_version: String,
    /// After defaults were filled in.
    pub params: Params,
    /// As the controller received it (after GPS conversion and tolerance).
    /// `Goal::Stop` when `Idle`.
    pub goal: Goal,
    /// Seconds, set by the controller; `None` = no limit.
    pub timeout: Option<f64>,
    /// Latest refused request, if any.
    pub last_rejected: Option<Rejection>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum TaskState {
    /// No task.
    Idle,
    Running { distance_to_goal: f64, elapsed: f64 },
    /// Within tolerance; holding still, watching for drift.
    Reached,
    /// By the operator; holding still.
    Paused,
    /// Holding still until a human acts.
    NeedsAttention { reason: Attention, since: f64 },
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Attention {
    /// The controller's timeout passed.
    Timeout,
    /// No progress, by the controller's rule.
    Stuck,
    ControllerError(String),
    /// Estimator untracked for too long.
    PoseLost,
    /// A GPS goal lost its anchor.
    NoGpsAnchor,
    /// E-stop engaged (phone or remote).
    EStop,
    /// `remote` controller: no `remote/cmd` for `COMMAND_LEASE`.
    RemoteSilent,
    /// Pushed out of tolerance, and the controller won't return.
    Displaced,
    /// Task restored from disk at startup.
    RoverRestarted,
}

/// A request the rover refused. The current task is untouched.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Rejection {
    pub task_id: u64,
    pub reason: String,
    pub t: f64,
}

/// Rover pose in the world frame.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Pose {
    pub t: f64,
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub tracked: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Status {
    pub t: f64,
    pub estopped: bool,
    pub link_ok: bool,
    pub detail: String,
    /// `None` until the estimator has one.
    pub gps_anchor: Option<GpsAnchor>,
    /// True when the dump gate is currently commanded open.
    #[serde(default)]
    pub gate_open: bool,
}

/// Where the world frame sits on Earth.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GpsAnchor {
    /// World origin, WGS84 degrees.
    pub latitude: f64,
    pub longitude: f64,
    /// World +X, rad clockwise from true north.
    pub heading: f64,
    /// m, 1σ.
    pub accuracy: f64,
}

/// Depth points, camera frame, metres. Encoded as a JSON header plus
/// little-endian `f32`s.
#[derive(Clone, Debug, PartialEq)]
pub struct Depth {
    pub t: f64,
    pub sequence: u64,
    pub points: Vec<[f32; 3]>,
}

/// Serialized ARKit `ARWorldMap` (`NSKeyedArchiver` bytes) plus its 3D feature
/// point cloud in Terra world coordinates (metres) for rendering on the base station.
#[derive(Clone, Debug, PartialEq)]
pub struct WorldMap {
    pub t: f64,
    pub sequence: u64,
    pub points: Vec<[f32; 3]>,
    pub data: Vec<u8>,
}

/// Rover request asking the base station to send back the saved shared `ARWorldMap`
/// so this rover can relocalize into the fleet's shared world coordinate frame.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct WorldMapRequest {
    pub t: f64,
    /// `true` when triggered automatically on connect (so the base station skips
    /// sending a rover's own map back to it).
    #[serde(default)]
    pub auto: bool,
}

/// Base-to-rover command asking the rover to capture its current `ARWorldMap`
/// and upload it on [`Topic::WorldMap`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CaptureWorldMap {
    pub t: f64,
}

/// Base-to-rover command delivering a saved `ARWorldMap` (`NSKeyedArchiver` bytes)
/// for ARKit relocalization so multiple rovers share one world coordinate origin.
#[derive(Clone, Debug, PartialEq)]
pub struct LoadWorldMap {
    pub t: f64,
    pub sequence: u64,
    pub data: Vec<u8>,
}

/// One incremental ARKit `ARMeshAnchor` chunk in Terra world coordinates (Z-up, metres).
///
/// Sent when an anchor is added, updated, or removed (`vertices` and `indices` empty
/// means the anchor `id` was removed).
///
/// `classes` has one `ARMeshClassification` byte per face (same length as `indices`, or
/// empty if classification is unavailable):
/// `0 = none, 1 = wall, 2 = floor, 3 = ceiling, 4 = table, 5 = seat, 6 = window, 7 = door`.
///
/// `colors` has one `[r, g, b]` sRGB triple per vertex (same length as `vertices`, or
/// empty if vertex colors are unavailable).
#[derive(Clone, Debug, PartialEq)]
pub struct MeshChunk {
    pub t: f64,
    pub id: String,
    pub vertices: Vec<[f32; 3]>,
    pub indices: Vec<[u32; 3]>,
    pub classes: Vec<u8>,
    pub colors: Vec<[u8; 3]>,
}

/// The rover's occupancy map: occupied cells and observed-free cells in the
/// world frame. This is the map its planners and frontier explorer use.
///
/// Sent when it changes. Encoded as a JSON header plus little-endian `f32`s.
#[derive(Clone, Debug, PartialEq)]
pub struct Occupancy {
    pub t: f64,
    /// Side of one cell, m.
    pub resolution: f64,
    /// Centre `[x, y]` of every occupied cell, m.
    pub cells: Vec<[f32; 2]>,
    /// Centre `[x, y]` of every observed-free cell, m (empty on older rovers).
    pub free: Vec<[f32; 2]>,
}

/// The rover's current global-planner route in the world frame.
///
/// Sent when the route changes (and repeated by the link so late-joining
/// base stations see it). Empty `poses` means no active route.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PlannedPath {
    pub t: f64,
    pub poses: Vec<Pose2>,
}

/// What the `remote` controller sees each tick, world frame.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RemoteState {
    pub t: f64,
    pub pose: Pose2,
    pub twist: Twist,
    pub path: Vec<Pose2>,
    /// Obstacle points `[x, y]`, m.
    pub obstacles: Vec<[f64; 2]>,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Pose2 {
    pub x: f64,
    pub y: f64,
    /// rad, CCW from +X.
    pub yaw: f64,
}

/// Body twist: forward m/s, yaw rate rad/s (CCW +).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Twist {
    pub v: f64,
    pub w: f64,
}

// ---------------------------------------------------------------- base → rover

/// Where to go and how. Persistent until a human replaces or clears it.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Task {
    /// Chosen by the base, unique per rover. Same id = update of the current
    /// task (the goal must be unchanged).
    pub id: u64,
    pub goal: Goal,
    /// A `Capabilities.controllers` id.
    pub controller: String,
    pub params: Params,
    /// `None` → the rover's configured planner.
    pub planner: Option<String>,
    pub planner_params: Params,
}

/// Acts on the current task. Rejected if `task_id` isn't it.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TaskCommand {
    pub task_id: u64,
    pub action: TaskAction,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TaskAction {
    Resume,
    Pause,
    Clear,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Goal {
    /// Hold still (still a task).
    Stop,
    /// World frame.
    Pose {
        /// m.
        x: f64,
        y: f64,
        /// rad, CCW from +X.
        yaw: Option<f64>,
        /// m; `Reached` inside this.
        tolerance: f64,
    },
    /// WGS84. Uses the compass convention on purpose, under a different name.
    Gps {
        /// Degrees.
        latitude: f64,
        longitude: f64,
        /// rad, clockwise from true north.
        heading: Option<f64>,
        /// m; at least the fix accuracy.
        tolerance: f64,
    },
    /// Autonomous frontier exploration: pick reachable frontiers on the
    /// occupancy map until none remain (or `budget_s` elapses).
    Explore {
        /// Optional time budget in seconds; `None` = until no frontiers remain.
        #[serde(default)]
        budget_s: Option<f64>,
    },
}

impl Goal {
    /// The goal kind a controller must accept; `None` for `Stop`.
    pub fn kind(&self) -> Option<GoalKind> {
        match self {
            Self::Stop => None,
            Self::Pose { .. } => Some(GoalKind::Local),
            Self::Gps { .. } => Some(GoalKind::Gps),
            Self::Explore { .. } => Some(GoalKind::Explore),
        }
    }
}

/// Body twist command from a stick. Also the answer on `remote/cmd`, wrapped
/// in [`RemoteCmd`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Teleop {
    pub t: f64,
    pub v: f64,
    pub w: f64,
}

/// A [`Teleop`] twist for the `remote` controller (topic `remote/cmd`).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(transparent)]
pub struct RemoteCmd(pub Teleop);

/// Remote e-stop. Latched on the rover: only `engaged: false` clears it.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Estop {
    pub t: f64,
    pub engaged: bool,
}

/// Command the rover's dump gate servo open (`true`) or closed (`false`).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct GateCommand {
    pub t: f64,
    pub open: bool,
}

/// Raw JSON WebRTC FPV signaling payloads (`fpv-signaling` owns the schema).
/// Each is capped at 65 536 bytes on the wire and never enters the control inbox.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct WebrtcOffer(pub String);
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct WebrtcIcePhone(pub String);
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct WebrtcState(pub String);
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct WebrtcRequest(pub String);
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct WebrtcAnswer(pub String);
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct WebrtcIceDashboard(pub String);
