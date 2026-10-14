//! Scenario packet: map, object list, camera, rover, level, and the label.

use serde::{Deserialize, Serialize};
use terra_autonomy::Level;
use terra_mapping::MapSnapshot;

use crate::geom::Counts;

/// Planner footprint used by `astar_route` in this pack. Matches Terra's default.
pub const FOOTPRINT_RADIUS: f64 = 0.65;
/// Check 4 tolerance from issue #49.
pub const WAYPOINT_TOLERANCE_M: f64 = 0.5;
/// Prompt rule, not a calibrated detector threshold.
pub const CONFIDENCE_ASK_BELOW: f64 = 0.4;
/// Prompt rule for the dump proposal case.
pub const DUMP_BUCKET_RANGE_M: f64 = 1.6;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RoverState {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub carrying_balls: bool,
    pub timestamp_s: f64,
    pub map_revision: u64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CameraModel {
    pub width: u32,
    pub height: u32,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub range_m: f64,
}

impl CameraModel {
    pub fn standard() -> Self {
        Self {
            width: 640,
            height: 480,
            fx: 480.0,
            fy: 480.0,
            cx: 320.0,
            cy: 240.0,
            range_m: 5.0,
        }
    }

    pub fn half_fov(&self) -> f64 {
        ((self.width as f64) * 0.5 / self.fx).atan()
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct SceneObject {
    pub id: String,
    pub class: String,
    pub x: f64,
    pub y: f64,
    pub approach_x: f64,
    pub approach_y: f64,
    pub min_x: f64,
    pub min_y: f64,
    pub max_x: f64,
    pub max_y: f64,
    pub cell_count: u32,
    pub confidence: f64,
    pub age_s: f64,
    pub goal_eligible: bool,
    pub reachable: bool,
    pub carried: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ExpectedLabel {
    pub decision: crate::decision::DecisionKind,
    pub accept: Vec<crate::decision::DecisionKind>,
    pub target_id: Option<String>,
    pub dump_proposal: bool,
    pub waypoint_required: bool,
    /// A waypoint passes when the list coordinate is within 0.5 m of one of these.
    pub waypoints: Vec<[f64; 2]>,
}

#[derive(Clone, Debug)]
pub struct Scenario {
    pub id: String,
    pub title: String,
    /// Human label note. Never copied into the model prompt.
    pub notes: String,
    pub camera_source: String,
    pub camera_source_note: String,
    pub map: MapSnapshot,
    pub map_png: Vec<u8>,
    pub camera_png: Vec<u8>,
    pub map_png_sha256: String,
    pub camera_png_sha256: String,
    pub objects: Vec<SceneObject>,
    pub counts: Counts,
    pub symbolic_grid: String,
    pub rover: RoverState,
    pub level: Level,
    pub camera: CameraModel,
    pub expected: ExpectedLabel,
}

pub fn level_label(level: Level) -> &'static str {
    match level {
        Level::Teleop => "teleop (L0)",
        Level::AssistedTeleop => "assisted_teleop (L1)",
        Level::WaypointDirect => "waypoint_direct (L2)",
        Level::Waypoint => "waypoint (L3)",
        Level::Supervised => "supervised",
        Level::TargetSearch => "target_search (L4)",
    }
}

pub fn sha256_hex(bytes: &[u8]) -> String {
    use sha2::{Digest, Sha256};
    format!("{:x}", Sha256::digest(bytes))
}
