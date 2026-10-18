//! Offline Phase P0 eval for Terra issue #49.
//!
//! Builds synthetic occupancy packets, renders them, and scores VLM decisions.
//! Nothing in this crate commands wheels, servos, or the arbiter.

mod decision;
mod font;
mod geom;
mod harness;
mod objects;
mod pack;
mod prompt;
mod render;
mod scene;

pub use decision::{
    DecisionKind, ModelDecision, Score, allows_model_waypoint, mock_decision, score_output,
    validate_waypoint,
};
pub use harness::{EvalReport, RunConfig, render_markdown, run_eval, write_report};
pub use pack::{scenarios, write_fixtures};
pub use prompt::{rules_sha256, user_prompt};
pub use scene::{FOOTPRINT_RADIUS, Scenario, WAYPOINT_TOLERANCE_M};

pub const CAMERA_SOURCE_PLACEHOLDER: &str = "placeholder";

pub const CAMERA_SOURCE_NOTE: &str = "\
Zorvane can boot the NEXT pitch headless (`TERRA_HEADLESS=1`) and can grab a \
windowed viewport under Xvfb (`scripts/capture-docs-frames.sh` in the Zorvane \
repo). Neither path exports an RGB camera frame posed to match one of these \
synthetic occupancy packets. The NEXT scene is not in this Terra checkout. \
These camera images are labelled schematics, not Zorvane renders.";
