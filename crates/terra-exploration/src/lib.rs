//! Time-bounded frontier exploration shared by Zorvane and TerraPhone.
//!
//! The crate plans where to go. It does not talk to motors, Zenoh, or a UI.
//! The autonomy arbiter executes each selected goal through the waypoint follower
//! and the local planner.
//!
//! Phase 1 stops on a time budget. Phase 2 can replace [`MissionObjective`] so a
//! detection ends the run or changes which frontier wins. This crate does not
//! detect anything itself.
//!
//! Poses, frontier coordinates, and footprint radii use world metres; time values
//! use seconds on a consistent control clock. [`Explorer::start`] validates a
//! budget but starts its clock only on the first valid tick. The caller executes
//! [`ExploreAction`] through its follower and supplies follower progress on the
//! next tick. Sensor freshness and platform safety are enforced by the integrating
//! arbiter, which can call [`Explorer::stop`] with a specific [`EndReason`].
//!
//! ```
//! use terra_exploration::{EndReason, ExploreAction, ExploreTick, Explorer};
//! use terra_mapping::MapSnapshot;
//! use terra_navigation::Pose;
//!
//! // A fully observed free map has no unknown boundary left to explore.
//! let map = MapSnapshot {
//!     width: 40, height: 40, resolution: 0.25,
//!     origin_x: -5.0, origin_y: -5.0, occupancy: vec![0; 1600],
//! };
//! let mut explorer = Explorer::default();
//! assert!(explorer.start(30.0, "example-run"));
//! let action = explorer.step(ExploreTick {
//!     now: 0.0, pose: Pose::default(), map: Some(&map),
//!     planner_blocked: false, goal_active: false, goal_arrived: false,
//!     goal_x: 0.0, goal_y: 0.0, goal_distance: 0.0,
//! });
//! assert_eq!(action, ExploreAction::Finished(EndReason::NoFrontiersLeft));
//! assert_eq!(explorer.status(0.0).coverage_m2, 100.0);
//! ```

mod explorer;
mod frontier;
mod objective;
pub use explorer::*;
pub use frontier::*;
pub use objective::*;
