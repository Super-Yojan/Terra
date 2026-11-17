//! Stage 4, Planning, in two layers:
//!
//! ```text
//! Goal ─► global::PathPlanner ─► Path ─► local::LocalPlanner ─► Twist
//!         (which route? slow)            (which v, ω now? every tick)
//! ```
pub mod global;
pub mod local;

use crate::geometry::Pose2;
use crate::vehicle::Twist;

pub enum Goal {
    None,
    /// Drive at this twist (a stick). No route needed.
    Velocity(Twist),
    /// Reach this pose.
    Pose(Pose2),
}

/// A route in the world frame, start to goal. Empty for `Goal::None` and
/// `Goal::Velocity`.
pub struct Path {
    pub poses: Vec<Pose2>,
}
