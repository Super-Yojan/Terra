//! Offline Phase P0 eval for Terra issue #49.
//!
//! Builds synthetic occupancy packets, renders them, and scores VLM decisions.
//! Nothing in this crate commands wheels, servos, or the arbiter.
//!
//! # Workflow
//!
//! Use [`scenarios`] to access the cached synthetic pack, [`user_prompt`] to
//! inspect model-visible evidence, and [`score_output`] to validate a response.
//! [`run_eval`] evaluates the complete pack through either a deterministic mock
//! or a configured model endpoint; [`write_report`] saves Markdown and JSON results.
//!
//! Camera images are labelled schematics. Expected labels are for local scoring
//! and are excluded from model prompts. Mock results verify the harness, rather
//! than measuring a model's decision quality.
//!
//! # Example
//!
//! ```
//! use terra_vlm_eval::{mock_decision, scenarios, score_output};
//!
//! let scenario = &scenarios()[0];
//! let output = mock_decision(scenario);
//! assert!(score_output(&output, scenario).passed());
//! ```

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

/// Provenance tag identifying synthetic camera schematics rather than captured images.
pub const CAMERA_SOURCE_PLACEHOLDER: &str = "placeholder";

/// Human-readable explanation of why the fixture camera images are schematics.
pub const CAMERA_SOURCE_NOTE: &str = "\
Zorvane can boot the NEXT pitch headless (`TERRA_HEADLESS=1`) and can grab a \
windowed viewport under Xvfb (`scripts/capture-docs-frames.sh` in the Zorvane \
repo). Neither path exports an RGB camera frame posed to match one of these \
synthetic occupancy packets. The NEXT scene is not in this Terra checkout. \
These camera images are labelled schematics, not Zorvane renders.";
