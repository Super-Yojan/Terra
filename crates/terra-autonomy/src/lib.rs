//! Onboard authority, safety holds, and autonomy-level arbitration.
//!
//! The `contract` module defines requests and status messages; `arbiter` decides
//! which intent may run on each tick. `search_contract` and `search` implement
//! target-search messages and state. Read [`AutonomyArbiter::step`] to follow the
//! path from sensor evidence and operator intent to an allowed motion command.

mod search;
pub use search::*;
mod search_contract;
pub use search_contract::*;
mod contract;
pub use contract::*;
mod arbiter;
pub use arbiter::*;
pub use terra_exploration::{
    Detection, EndReason, ExplorationConfig, ExplorationStatus, ExplorePhase, FindTargetsObjective,
    FrontierTarget, MissionObjective,
};

/// Dashboard telemetry for observed cells only; never includes simulator ground truth.
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
