//! Local planning: pick the twist to command this tick, following the path
//! and avoiding obstacles close by. A local planner is what the base
//! station calls a "controller".
mod drive_pose;
mod dwa;
mod remote;
mod teleop;
pub use drive_pose::DrivePose;
pub use dwa::{Dwa, DwaWeights};
pub use remote::Remote;
pub use teleop::Teleop;

use super::{Goal, Path, PoseGoal, YAW_TOLERANCE};
use crate::estimation::State;
use crate::geometry::Pose2;
use crate::mapping::Map;
use crate::vehicle::{Twist, VehicleModel};

// The default time limit: distance / expected speed × margin + slack.
/// A rover is expected to average this fraction of its top speed.
const EXPECTED_SPEED_FRACTION: f64 = 0.5;
/// Allow this many times the expected driving time…
const TIMEOUT_MARGIN: f64 = 2.0;
/// …plus this (s), for turning on the spot at the start and the end.
const TIMEOUT_SLACK_S: f64 = 15.0;
/// Default for [`LocalPlanner::stuck_after`] (s).
const STUCK_AFTER_S: f64 = 10.0;

// Turning on the spot to the goal heading.
/// Yaw rate per radian of heading error (1/s).
const TURN_GAIN: f64 = 2.0;
/// Never turn slower than this (rad/s): a smaller command does not
/// overcome the friction of a skid-steer base, and the rover would sit
/// just outside the heading tolerance for ever.
const MIN_TURN_RATE: f64 = 0.3;

pub trait LocalPlanner: Send {
    /// The twist to command this tick.
    ///
    /// `goal` is passed too, so a stick (`Goal::Velocity`) can bypass the
    /// path. For `Goal::Pose`, `path` is the route to it. The result still
    /// goes through the pipeline's fixed chain (clamp, acceleration limit,
    /// e-stop), whatever is returned here.
    fn follow(
        &mut self,
        state: &State,
        map: &Map,
        goal: &Goal,
        path: &Path,
        vehicle: &dyn VehicleModel,
    ) -> Twist;

    /// How long (s) driving to `goal` may take from `state` along `path`
    /// before the task asks a human for attention. `None`: no limit.
    ///
    /// Default: the time to cover the distance at half the top speed,
    /// doubled, plus 15 s.
    fn timeout(
        &self,
        goal: &PoseGoal,
        state: &State,
        path: &Path,
        vehicle: &dyn VehicleModel,
    ) -> Option<f64> {
        // The path can be shorter than the straight line when it is empty.
        let distance = path.length().max(goal.distance(&state.pose));
        let expected_speed = EXPECTED_SPEED_FRACTION * vehicle.max_speed();
        Some(distance / expected_speed * TIMEOUT_MARGIN + TIMEOUT_SLACK_S)
    }

    /// How long (s) the rover may go without getting closer to the goal
    /// before the task asks for attention. `None`: never.
    fn stuck_after(&self) -> Option<f64> {
        Some(STUCK_AFTER_S)
    }

    /// After reaching the goal, if the rover is pushed away from it: drive
    /// back (`true`), or hold still and ask for attention (`false`).
    fn returns_to_goal(&self) -> bool {
        true
    }

    /// `Some` only for [`Remote`], whose twist comes from the base station:
    /// the task runner uses this to hand it the newest command.
    fn as_remote(&mut self) -> Option<&mut Remote> {
        None
    }
}

/// What to do once the rover is within the goal's position tolerance:
/// turn on the spot to the goal heading, then hold still. `None` while the
/// rover is still outside the tolerance (keep driving).
///
/// Shared by every controller that drives to a pose, so they all finish
/// the same way.
pub fn finish_at_goal(goal: &PoseGoal, pose: &Pose2, vehicle: &dyn VehicleModel) -> Option<Twist> {
    if goal.distance(pose) > goal.tolerance {
        return None;
    }
    let error = goal.yaw_error(pose);
    if error.abs() <= YAW_TOLERANCE {
        return Some(Twist::STOP);
    }
    // Proportional turn toward the goal heading, never below MIN_TURN_RATE.
    let rate = (TURN_GAIN * error.abs()).max(MIN_TURN_RATE);
    Some(vehicle.clamp(Twist { v: 0.0, w: rate.copysign(error) }))
}
