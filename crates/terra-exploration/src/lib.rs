//! Time-bounded frontier exploration shared by Zorvane and TerraPhone.
//!
//! The crate plans where to go. It does not talk to motors, Zenoh, or a UI.
//! The autonomy arbiter executes each selected goal through the waypoint follower
//! and the local planner.
//!
//! Phase 1 stops on a time budget. Phase 2 can replace [`MissionObjective`] so a
//! detection ends the run or changes which frontier wins. This crate does not
//! detect anything itself.
mod explorer;
mod frontier;
mod objective;
pub use explorer::*;
pub use frontier::*;
pub use objective::*;
