//! Stage 4, Planning, in two layers:
//!
//! ```text
//! Goal ─► global::PathPlanner ─► Path ─► local::LocalPlanner ─► Twist
//!         (which route? slow)            (which v, ω now? every tick)
//! ```
//!
//! [`Navigator`] runs the two layers together for one pose goal. The
//! planners and controllers that can be picked by name (from the config, or
//! from the base station) are listed in [`registry`].
pub mod exploration;
pub mod global;
pub mod local;
mod navigator;
pub mod registry;
pub use exploration::{ExploreStep, Explorer};
pub use navigator::{Navigator, NoRoute};

use crate::geometry::{Pose2, Vec2, wrap_angle};
use crate::vehicle::Twist;

/// A goal heading counts as reached within this (rad, about 6°).
pub const YAW_TOLERANCE: f64 = 0.1;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Goal {
    /// Hold still.
    None,
    /// Drive at this twist (a stick). No route needed.
    Velocity(Twist),
    /// Reach this pose.
    Pose(PoseGoal),
}

/// A pose to reach, in the world frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PoseGoal {
    /// m.
    pub x: f64,
    pub y: f64,
    /// Heading to end at (rad, CCW from +X). `None`: any heading will do.
    pub yaw: Option<f64>,
    /// Reached when the rover is within this distance (m).
    pub tolerance: f64,
}

impl PoseGoal {
    pub fn position(&self) -> Vec2 {
        Vec2 { x: self.x, y: self.y }
    }

    /// Straight-line distance from `pose` to the goal (m).
    pub fn distance(&self, pose: &Pose2) -> f64 {
        self.position().distance(pose.position())
    }

    /// How far `pose` still has to turn to face the goal heading (rad,
    /// + = turn left). Zero for a goal without a heading.
    pub fn yaw_error(&self, pose: &Pose2) -> f64 {
        match self.yaw {
            Some(yaw) => wrap_angle(yaw - pose.yaw),
            None => 0.0,
        }
    }

    /// Within `tolerance` of the position and, if the goal has a heading,
    /// within [`YAW_TOLERANCE`] of it.
    pub fn reached(&self, pose: &Pose2) -> bool {
        self.distance(pose) <= self.tolerance && self.yaw_error(pose).abs() <= YAW_TOLERANCE
    }
}

/// A route in the world frame, start to goal. Each pose's `yaw` is the
/// direction of travel there.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Path {
    pub poses: Vec<Pose2>,
}

impl Path {
    /// Length along the path (m): the sum of the straight pieces between
    /// neighbouring poses.
    pub fn length(&self) -> f64 {
        self.poses
            .windows(2)
            .map(|pair| pair[0].position().distance(pair[1].position()))
            .sum()
    }

    /// The point to steer at from `position`: the point `distance` metres
    /// further along the path than the path point closest to `position`.
    /// Past the end of the path it is the last pose. `None` for an empty
    /// path.
    ///
    /// The path is treated as straight pieces between neighbouring poses,
    /// so this works the same for a two-pose path and a dense one. The
    /// returned `yaw` is that of the pose at the end of the piece the point
    /// lies on: the heading to arrive there with.
    pub fn lookahead(&self, position: Vec2, distance: f64) -> Option<Pose2> {
        let last = *self.poses.last()?;

        // 1. The path point closest to `position`: on piece `closest_piece`
        //    (from pose i to pose i + 1), `closest_fraction` of the way along.
        let mut closest_piece = 0;
        let mut closest_fraction = 0.0;
        let mut closest_range = f64::INFINITY;
        for (piece, pair) in self.poses.windows(2).enumerate() {
            let (a, b) = (pair[0].position(), pair[1].position());
            let (ab_x, ab_y) = (b.x - a.x, b.y - a.y);
            let length_squared = ab_x * ab_x + ab_y * ab_y;
            // Project `position` onto the line a → b, and stay on the piece.
            let fraction = if length_squared > 0.0 {
                let along = (position.x - a.x) * ab_x + (position.y - a.y) * ab_y;
                (along / length_squared).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let point = Vec2 { x: a.x + ab_x * fraction, y: a.y + ab_y * fraction };
            let range = point.distance(position);
            if range < closest_range {
                (closest_piece, closest_fraction, closest_range) = (piece, fraction, range);
            }
        }

        // 2. Walk `distance` metres forward from there, piece by piece.
        //    If the path reverses direction at a cusp (e.g. Hybrid A* backing
        //    out and then driving forward), stop the lookahead at the cusp
        //    until the rover gets close to it, so the rover finishes backing
        //    out before switching to forward.
        const CUSP_CLEAR_M: f64 = 0.15;
        let piece_is_reverse = |pair: &[Pose2]| -> bool {
            let (a, b) = (pair[0], pair[1]);
            (b.x - a.x) * a.yaw.cos() + (b.y - a.y) * a.yaw.sin() < -1e-6
        };
        let mut left_to_walk = distance;
        let mut fraction = closest_fraction;
        let mut prev_reverse: Option<bool> = None;
        for pair in self.poses.windows(2).skip(closest_piece) {
            let reverse = piece_is_reverse(pair);
            if let Some(was_reverse) = prev_reverse
                && reverse != was_reverse
                && position.distance(pair[0].position()) > CUSP_CLEAR_M
            {
                return Some(pair[0]);
            }
            prev_reverse = Some(reverse);

            let (a, b) = (pair[0].position(), pair[1].position());
            let length = a.distance(b);
            let left_on_piece = length * (1.0 - fraction);
            if left_to_walk < left_on_piece {
                let fraction = fraction + left_to_walk / length;
                return Some(Pose2 {
                    x: a.x + (b.x - a.x) * fraction,
                    y: a.y + (b.y - a.y) * fraction,
                    yaw: pair[1].yaw,
                });
            }
            left_to_walk -= left_on_piece;
            fraction = 0.0;
        }
        Some(last)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lookahead_stops_at_cusp_until_reached() {
        // Back out from (0, 0) to (-0.6, 0) facing +X, then drive forward to (0.6, 0.5).
        let path = Path {
            poses: vec![
                Pose2 { x: 0.0, y: 0.0, yaw: 0.0 },
                Pose2 { x: -0.3, y: 0.0, yaw: 0.0 },
                Pose2 { x: -0.6, y: 0.0, yaw: 0.0 },
                Pose2 { x: 0.0, y: 0.25, yaw: 0.4 },
                Pose2 { x: 0.6, y: 0.5, yaw: 0.4 },
            ],
        };

        // At start (0, 0), even with 0.8 m lookahead (> 0.6 m backout), stop at cusp (-0.6, 0).
        let target = path.lookahead(Vec2 { x: 0.0, y: 0.0 }, 0.8).unwrap();
        assert!((target.x - -0.6).abs() < 1e-6 && target.y.abs() < 1e-6);

        // Once close to the cusp (-0.55, 0), lookahead continues past it into the forward leg.
        let target_after = path.lookahead(Vec2 { x: -0.55, y: 0.0 }, 0.8).unwrap();
        assert!(target_after.x > -0.5);
    }
}
