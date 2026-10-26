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

/// Synthetic planar rover pose and mission state included in the prompt.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RoverState {
    /// World x coordinate in metres.
    pub x: f64,
    /// World y coordinate in metres.
    pub y: f64,
    /// World heading in radians.
    pub yaw: f64,
    /// Whether the synthetic rover is carrying balls.
    pub carrying_balls: bool,
    /// Synthetic exposure timestamp in seconds.
    pub timestamp_s: f64,
    /// Revision identifying the occupancy observation.
    pub map_revision: u64,
}

/// Pinhole image calibration and the visibility range used by schematic rendering.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CameraModel {
    /// Image width in pixels.
    pub width: u32,
    /// Image height in pixels.
    pub height: u32,
    /// Horizontal focal length in pixels.
    pub fx: f64,
    /// Vertical focal length in pixels.
    pub fy: f64,
    /// Principal point column in pixels.
    pub cx: f64,
    /// Principal point row in pixels.
    pub cy: f64,
    /// Camera visibility range in metres.
    pub range_m: f64,
}

impl CameraModel {
    /// Return a 640 × 480 pinhole camera with 480 px focal lengths and 5 m range.
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

    /// Return horizontal half field of view in radians; assumes a positive focal length.
    pub fn half_fov(&self) -> f64 {
        ((self.width as f64) * 0.5 / self.fx).atan()
    }
}

/// Semantic or geometry-derived object with a validated approach candidate.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct SceneObject {
    /// Stable identifier shared by prompts, labels, and reports.
    pub id: String,
    /// Semantic object class, such as bucket, person, or frontier.
    pub class: String,
    /// World x coordinate in metres.
    pub x: f64,
    /// World y coordinate in metres.
    pub y: f64,
    /// Validated approach point world x coordinate in metres.
    pub approach_x: f64,
    /// Validated approach point world y coordinate in metres.
    pub approach_y: f64,
    /// Lower world x extent in metres.
    pub min_x: f64,
    /// Lower world y extent in metres.
    pub min_y: f64,
    /// Upper world x extent in metres.
    pub max_x: f64,
    /// Upper world y extent in metres.
    pub max_y: f64,
    /// Number of grid cells represented by this object.
    pub cell_count: u32,
    /// Uncalibrated confidence in the range zero to one.
    pub confidence: f64,
    /// Observation age in seconds.
    pub age_s: f64,
    /// Whether this object class may be selected as a waypoint target.
    pub goal_eligible: bool,
    /// Whether an observed-space route to the approach point was found.
    pub reachable: bool,
    /// Whether this object is already carried by the rover.
    pub carried: bool,
}

/// Ground-truth scoring criteria kept separate from model-visible observations.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ExpectedLabel {
    /// High-level decision category.
    pub decision: crate::decision::DecisionKind,
    /// Decision categories accepted by this scenario label.
    pub accept: Vec<crate::decision::DecisionKind>,
    /// Optional identifier of the selected object-list entry.
    pub target_id: Option<String>,
    /// Whether to propose dumping; this does not actuate hardware.
    pub dump_proposal: bool,
    /// Whether passing the scenario requires a validated waypoint.
    pub waypoint_required: bool,
    /// A waypoint passes when the list coordinate is within 0.5 m of one of these.
    pub waypoints: Vec<[f64; 2]>,
}

/// Complete synthetic evaluation packet, rendered images, and private scoring label.
#[derive(Clone, Debug)]
pub struct Scenario {
    /// Stable identifier shared by prompts, labels, and reports.
    pub id: String,
    /// Human-readable scenario title.
    pub title: String,
    /// Human label note. Never copied into the model prompt.
    pub notes: String,
    /// Camera image provenance label; synthetic fixtures use placeholder images.
    pub camera_source: String,
    /// Explanation of the camera image provenance and limitations.
    pub camera_source_note: String,
    /// Observed occupancy grid used for rendering and route validation.
    pub map: MapSnapshot,
    /// Encoded PNG bytes of the occupancy visualization.
    pub map_png: Vec<u8>,
    /// Encoded PNG bytes of the labelled camera schematic.
    pub camera_png: Vec<u8>,
    /// Lowercase SHA-256 digest of the map PNG bytes.
    pub map_png_sha256: String,
    /// Lowercase SHA-256 digest of the camera PNG bytes.
    pub camera_png_sha256: String,
    /// Detected and derived scene objects available to the model.
    pub objects: Vec<SceneObject>,
    /// Counts of free, occupied, and unknown map cells.
    pub counts: Counts,
    /// Coarse text occupancy grid included in the prompt.
    pub symbolic_grid: String,
    /// Synthetic rover pose and mission state.
    pub rover: RoverState,
    /// Autonomy level governing whether waypoint decisions are allowed.
    pub level: Level,
    /// Camera calibration and visibility range for the schematic.
    pub camera: CameraModel,
    /// Scoring label excluded from the model prompt.
    pub expected: ExpectedLabel,
}

pub fn level_label(level: Level) -> &'static str {
    match level {
        Level::Teleop => "teleop (L0)",
        Level::AssistedTeleop => "assisted_teleop (L1)",
        Level::WaypointDirect => "waypoint_direct (L2)",
        Level::Waypoint => "waypoint (L3)",
        Level::Supervised => "supervised",
        Level::Explore => "explore (time-bounded frontier)",
        Level::TargetSearch => "target_search (L4)",
        Level::Task => "task (L5)",
    }
}

pub fn sha256_hex(bytes: &[u8]) -> String {
    use sha2::{Digest, Sha256};
    format!("{:x}", Sha256::digest(bytes))
}
