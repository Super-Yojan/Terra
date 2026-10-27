//! Onboard authority, safety holds, and autonomy-level arbitration.
//!
//! The `contract` module defines requests and status messages; `arbiter` decides
//! which intent may run on each tick. `search_contract` and `search` implement
//! target-search messages and state. Read [`AutonomyArbiter::step`] to follow the
//! path from sensor evidence and operator intent to an allowed motion command.
//!
//! All control timestamps share one seconds clock; poses use world metres and
//! radians, while twists use metres per second and radians per second. A tick
//! requires strictly increasing finite nonnegative time. Freshness and authority
//! checks may hold an otherwise valid intent at zero; execute only the returned
//! [`ArbiterOutput::twist`]. Request decoders reject unknown fields and payloads
//! larger than 2048 bytes before the arbiter applies runtime authority checks.
//!
//! ```
//! use terra_autonomy::{ArbiterInput, AutonomyArbiter, decode_teleop};
//! use terra_navigation::{Pose, Twist};
//!
//! let mut arbiter = AutonomyArbiter::default(); // Starts in teleop.
//! let request = decode_teleop(br#"{"linear":0.2,"angular":0.0}"#).unwrap();
//! arbiter.accept_operator(request, 0.0);
//! let output = arbiter.step(ArbiterInput {
//!     now: 0.1,
//!     pose: Pose::default(),
//!     pose_time: 0.1,
//!     map: None,
//!     map_time: None,
//!     map_revision: 0,
//!     measured: Twist::default(),
//!     healthy: true,
//! });
//! assert_eq!(output.twist.linear, 0.2);
//! assert_eq!(output.status.active_source, "operator");
//! ```

mod search;
pub use search::*;
mod search_contract;
pub use search_contract::*;
mod contract;
pub use contract::*;
mod arbiter;
pub use arbiter::*;
mod task;
pub use task::*;
/// Detected buckets and balls in the local frame, with GPS when outdoor.
pub mod objects;
pub use terra_exploration::{
    Detection, EndReason, ExplorationConfig, ExplorationStatus, ExplorePhase, FindTargetsObjective,
    FrontierTarget, MissionObjective,
};

/// Dashboard telemetry for observed cells only; never includes simulator ground truth.
///
/// Copies map dimensions, metre resolution/origin, and signed occupancy values into
/// a schema-version-one JSON object with caller-supplied run and sequence IDs.
/// This function does not validate or modify the map.
pub fn occupancy_telemetry(
    rover_id: u64,
    run_id: &str,
    sequence: u64,
    map: &terra_mapping::MapSnapshot,
) -> serde_json::Value {
    serde_json::json!({
        "schema_version": 1,
        "rover_id": rover_id,
        "run_id": run_id,
        "sequence": sequence,
        "width": map.width,
        "height": map.height,
        "resolution": map.resolution,
        "origin_x": map.origin_x,
        "origin_y": map.origin_y,
        "occupancy": map.occupancy,
    })
}

/// Shared geographic waypoint controller used by phone and simulator.
pub mod outdoor;
