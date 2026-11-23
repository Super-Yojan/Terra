//! Drive-to-pose controller (roadmap 4): steer onto the next path pose,
//! position and heading. Does not look at the map: pair it with a path
//! planner that avoids obstacles (`hybrid_astar`), or use it on open ground.
//!
//! The classic polar-coordinate law (Siegwart & Nourbakhsh, *Introduction
//! to Autonomous Mobile Robots*, §3.6):
//!
//! ```text
//!                      target (its yaw = heading to arrive with)
//!                     ↗
//!           ρ       ╱   β = target yaw − line of sight
//!                 ╱
//!   rover ──────►        α = line of sight − rover yaw
//!        heading
//!
//!   v = k_rho · ρ                     drive faster when far
//!   w = k_alpha · α + k_beta · β      turn toward the target (α), and bend
//!                                     the approach so it arrives with the
//!                                     target's heading (β)
//! ```
//!
//! The law is stable for `k_rho > 0`, `k_beta < 0`, `k_alpha > k_rho`.
//! `k_beta` is negative on purpose: with the target dead ahead but its
//! heading to the left (β > 0), the rover first swings right, so that it
//! comes in curving left.
//!
//! Two additions to the textbook law:
//! - `v` is scaled by `cos α` (never below zero), so the rover turns
//!   toward a target that is beside or behind it before driving off;
//! - the result goes through `vehicle.clamp`, which slows a too-fast twist
//!   down but keeps `w / v`, so the rover drives the same curve, slower.
use terra_comms::GoalKind;

use super::{LocalPlanner, finish_at_goal};
use crate::geometry::wrap_angle;
use crate::planning::registry::{ControllerSpec, float, float_param};
use crate::planning::{Goal, Path};
use crate::{
    estimation::State,
    mapping::Map,
    vehicle::{Twist, VehicleModel},
};

/// Steer at the path pose this far ahead (m).
const LOOKAHEAD_M: f64 = 0.5;

pub struct DrivePose {
    pub k_rho: f64,
    pub k_alpha: f64,
    pub k_beta: f64,
}

impl DrivePose {
    pub fn spec() -> ControllerSpec {
        ControllerSpec {
            id: "drive_pose",
            version: "1",
            description: "Steer onto the next path pose (no obstacle avoidance)",
            goals: vec![GoalKind::Local, GoalKind::Explore],
            params: vec![
                float_param("k_rho", 1.0, 0.05, 10.0, "1/s", "Speed per metre of distance to the goal"),
                float_param("k_alpha", 3.0, 0.1, 20.0, "1/s", "Turn rate per radian off the line to the target; keep above k_rho"),
                float_param("k_beta", -0.5, -10.0, 0.0, "1/s", "Turn rate per radian between that line and the arrival heading; negative"),
            ],
            make: |params| {
                Box::new(DrivePose {
                    k_rho: float(params, "k_rho"),
                    k_alpha: float(params, "k_alpha"),
                    k_beta: float(params, "k_beta"),
                })
            },
        }
    }
}

impl LocalPlanner for DrivePose {
    fn follow(
        &mut self,
        state: &State,
        _map: &Map,
        goal: &Goal,
        path: &Path,
        vehicle: &dyn VehicleModel,
    ) -> Twist {
        let goal = match goal {
            Goal::None => return Twist::STOP,
            Goal::Velocity(stick) => return vehicle.clamp(*stick),
            Goal::Pose(goal) => goal,
        };
        let pose = state.pose;

        // Inside the position tolerance: turn to the goal heading, or stop.
        if let Some(twist) = finish_at_goal(goal, &pose, vehicle) {
            return twist;
        }
        // The pose to steer at. Without a path there is nothing to follow.
        let Some(target) = path.lookahead(pose.position(), LOOKAHEAD_M) else {
            return Twist::STOP;
        };

        let line_of_sight = (target.y - pose.y).atan2(target.x - pose.x);
        let alpha = wrap_angle(line_of_sight - pose.yaw);
        let beta = wrap_angle(target.yaw - line_of_sight);
        // ρ is measured to the goal, not to the lookahead pose (which stays
        // about LOOKAHEAD_M away), so the speed only drops near the goal.
        let rho = goal.distance(&pose);

        vehicle.clamp(Twist {
            v: self.k_rho * rho * alpha.cos().max(0.0),
            w: self.k_alpha * alpha + self.k_beta * beta,
        })
    }
}
