mod contract;
pub use contract::*;
mod arbiter;
pub use arbiter::*;

/// Dashboard telemetry for observed cells only; never includes simulator ground truth.
pub fn occupancy_telemetry(
    rover_id: u64,
    run_id: &str,
    sequence: u64,
    map: &terra_mapping::MapSnapshot,
) -> serde_json::Value {
    serde_json::json!({"schema_version":1,"rover_id":rover_id,"run_id":run_id,"sequence":sequence,"width":map.width,"height":map.height,"resolution":map.resolution,"origin_x":map.origin_x,"origin_y":map.origin_y,"occupancy":map.occupancy})
}
