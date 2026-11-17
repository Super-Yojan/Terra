//! Pick one implementation per stage from a TOML file:
//!
//! ```toml
//! [vehicle]    type = "diff_drive", track = 0.5, max_wheel_speed = 0.8
//! [mount]      roll = -90, pitch = 0, yaw = 0, offset = [0.10, 0.0, 0.23]
//! [estimator]  type = "localization"
//! [mapper]     type = "occupancy_grid"
//! [path_planner]  type = "hybrid_astar", resolution = 0.1
//! [local_planner] type = "dwa", horizon = 2.0
//! ```
use crate::pipeline::Pipeline;

pub struct PipelineConfig;

impl PipelineConfig {
    pub fn from_toml(text: &str) -> Result<Self, String> { todo!() }
    pub fn build(&self) -> Pipeline { todo!() }
}
