//! Dynamic Window Approach (roadmap 5): sample reachable twists, simulate
//! each with the vehicle model, score progress along the path, clearance
//! and speed. (Fox, Burgard & Thrun, 1997.)
//!
//! Each tick:
//!
//! 1. **Window.** The twists the rover can reach within [`WINDOW_S`] from
//!    its measured twist, given its acceleration limits:
//!    `v ∈ [v₀ − a·T, v₀ + a·T]`, `w ∈ [w₀ − α·T, w₀ + α·T]`, cut to what
//!    the vehicle can do at all. Reverse is allowed down to
//!    `-max_reverse_speed` (set to `0` for forward-only) so the rover can
//!    back out along a reverse segment from `hybrid_astar` or away from a
//!    front obstacle.
//! 2. **Sample.** A `samples × samples` grid of `(v, w)` over the window.
//! 3. **Roll out.** Hold each twist for `horizon` seconds with
//!    `vehicle.step`. A twist whose rollout comes within the rover's radius
//!    of an obstacle is thrown out.
//! 4. **Score** the others, each term in `0..=1`, and take the best:
//!    - heading: how squarely the rover's direction of travel (`preview.yaw`
//!      when forward, `preview.yaw + π` when reversing) faces the path's
//!      lookahead point [`STEER_PREVIEW_S`] into the rollout;
//!    - clearance: the smallest gap to an obstacle along the rollout;
//!    - speed: `|v|`, as a fraction of the top speed;
//!    - reverse twists pay [`REVERSE_PENALTY`] so forward always wins when
//!      both are feasible.
//!
//! Heading is judged early in the rollout, not at its end: by the end, a
//! fast twist has driven past the lookahead point and faces away from it,
//! which would make the slowest twist look best.
//!
//! If every twist was thrown out, the rover stops. Throwing out every
//! rollout that touches something within the horizon is cautious on
//! purpose: heading for something it cannot steer around, the rover slows
//! to `distance / horizon` and creeps to a halt well before it.
use terra_comms::GoalKind;

use super::{LocalPlanner, finish_at_goal};
use crate::geometry::{Pose2, Vec2, wrap_angle};
use crate::planning::registry::{ControllerSpec, float, float_param, int, int_param};
use crate::planning::{Goal, Path};
use crate::{
    estimation::State,
    mapping::Map,
    vehicle::{Twist, VehicleModel},
};

/// Steer at the path point at least this far ahead (m).
const LOOKAHEAD_M: f64 = 0.8;
/// Heading is judged this far into a rollout (s).
const STEER_PREVIEW_S: f64 = 0.5;
/// The window holds the twists reachable within this time (s).
const WINDOW_S: f64 = 0.25;
/// Time between the simulated poses of a rollout (s).
const ROLLOUT_STEP_S: f64 = 0.1;
/// A gap of this much (m) or more gets the full clearance score: beyond
/// it, more room is not better, so open space does not pull the rover
/// off the path.
const FULL_CLEARANCE_M: f64 = 0.5;
/// Near the goal, go no faster than this times the distance left
/// (m/s per m), so the rover arrives slowly instead of overshooting.
const APPROACH_GAIN: f64 = 1.0;
/// Score penalty applied to reverse twists so forward always wins when both
/// forward and reverse are feasible.
const REVERSE_PENALTY: f64 = 0.25;

pub struct Dwa {
    /// How far ahead each twist is simulated (s).
    pub horizon: f64,
    /// Twists sampled per axis: `samples` speeds × `samples` yaw rates.
    pub samples: usize,
    /// Maximum reverse speed for backing out (m/s; `0.0` disables reverse).
    pub max_reverse_speed: f64,
    pub weights: DwaWeights,
}

pub struct DwaWeights {
    pub heading: f64,
    pub clearance: f64,
    pub speed: f64,
}

impl Dwa {
    pub fn spec() -> ControllerSpec {
        ControllerSpec {
            id: "dwa",
            version: "2",
            description: "Dynamic window approach: follows the path, avoids obstacles, and can back out",
            goals: vec![GoalKind::Local],
            params: vec![
                float_param("horizon", 2.0, 0.5, 10.0, "s", "How far ahead each candidate twist is simulated"),
                int_param("samples", 20, 2, 60, "Candidate twists per axis (speeds × yaw rates)"),
                float_param("max_reverse_speed", 0.25, 0.0, 1.0, "m/s", "Maximum backing-out speed (0 = forward only)"),
                // Defaults in the proportions of the original paper (2 : 0.2 : 0.2).
                float_param("heading_weight", 2.0, 0.0, 10.0, "", "Weight of facing the path ahead"),
                float_param("clearance_weight", 0.2, 0.0, 10.0, "", "Weight of keeping away from obstacles"),
                float_param("speed_weight", 0.2, 0.0, 10.0, "", "Weight of driving fast"),
            ],
            make: |params| {
                Box::new(Dwa {
                    horizon: float(params, "horizon"),
                    samples: int(params, "samples") as usize,
                    max_reverse_speed: float(params, "max_reverse_speed"),
                    weights: DwaWeights {
                        heading: float(params, "heading_weight"),
                        clearance: float(params, "clearance_weight"),
                        speed: float(params, "speed_weight"),
                    },
                })
            },
        }
    }

    /// Simulate holding `twist` for `horizon` seconds from `start`.
    /// Returns the pose after [`STEER_PREVIEW_S`], and the smallest gap (m)
    /// between the rover's edge and an obstacle over the whole horizon;
    /// `None` if the rover would touch one.
    fn rollout(
        &self,
        start: Pose2,
        twist: Twist,
        obstacles: &[Vec2],
        vehicle: &dyn VehicleModel,
    ) -> Option<(Pose2, f64)> {
        let steps = (self.horizon / ROLLOUT_STEP_S).ceil() as usize;
        let preview_step = (STEER_PREVIEW_S / ROLLOUT_STEP_S).round() as usize;
        let mut pose = start;
        let mut preview = start;
        let mut smallest_gap = f64::INFINITY;
        for step in 1..=steps {
            pose = vehicle.step(pose, twist, ROLLOUT_STEP_S);
            if step == preview_step {
                preview = pose;
            }
            for obstacle in obstacles {
                let gap = obstacle.distance(pose.position()) - vehicle.radius();
                smallest_gap = smallest_gap.min(gap);
            }
            if smallest_gap <= 0.0 {
                return None;
            }
        }
        Some((preview, smallest_gap))
    }

    /// The `index`-th of `samples` evenly spaced values from `low` to `high`.
    fn sample(&self, low: f64, high: f64, index: usize) -> f64 {
        let fraction = index as f64 / (self.samples - 1).max(1) as f64;
        low + (high - low) * fraction
    }
}

impl LocalPlanner for Dwa {
    fn follow(
        &mut self,
        state: &State,
        map: &Map,
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

        // 1. The dynamic window around the measured twist.
        let limits = vehicle.limits();
        let top_speed = vehicle.max_speed().min(APPROACH_GAIN * goal.distance(&pose));
        let top_reverse = self.max_reverse_speed.max(0.0).min(top_speed);
        let top_yaw_rate = vehicle.max_yaw_rate();
        let v_high = (state.twist.v + limits.max_accel * WINDOW_S).clamp(-top_reverse, top_speed);
        let v_low = (state.twist.v - limits.max_accel * WINDOW_S).clamp(-top_reverse, v_high);
        let w_high = (state.twist.w + limits.max_yaw_accel * WINDOW_S).clamp(-top_yaw_rate, top_yaw_rate);
        let w_low = (state.twist.w - limits.max_yaw_accel * WINDOW_S).clamp(-top_yaw_rate, w_high);

        // The point to head for: far enough along the path that no rollout
        // reaches it within the steering preview. Without a path there is
        // nothing to follow.
        let max_v_mag = v_high.abs().max(v_low.abs());
        let lookahead = LOOKAHEAD_M.max(2.0 * max_v_mag * STEER_PREVIEW_S);
        let Some(target) = path.lookahead(pose.position(), lookahead) else {
            return Twist::STOP;
        };

        // Only obstacles a rollout could come near matter: within the
        // farthest reach of a rollout, plus the rover's radius, plus the
        // gap that still changes the clearance score.
        let reach = max_v_mag * self.horizon + vehicle.radius() + FULL_CLEARANCE_M;
        let nearby: Vec<Vec2> = map
            .obstacles
            .iter()
            .copied()
            .filter(|obstacle| obstacle.distance(pose.position()) <= reach)
            .collect();

        // 2–4. Sample, roll out, score; keep the best.
        let mut best: Option<(f64, Twist)> = None;
        for v_index in 0..self.samples {
            for w_index in 0..self.samples {
                let twist = Twist {
                    v: self.sample(v_low, v_high, v_index),
                    w: self.sample(w_low, w_high, w_index),
                };
                // `clamp` changes a twist only if a wheel would be too fast.
                if vehicle.clamp(twist) != twist {
                    continue;
                }
                let Some((preview, gap)) = self.rollout(pose, twist, &nearby, vehicle) else {
                    continue;
                };

                // Heading: 1 when the direction of travel faces the target,
                // 0 when it faces away. When backing up, the rear of the
                // rover (`preview.yaw + π`) moves toward the target.
                let bearing = (target.y - preview.y).atan2(target.x - preview.x);
                let travel_yaw = if twist.v < 0.0 {
                    preview.yaw + std::f64::consts::PI
                } else {
                    preview.yaw
                };
                let off_target = wrap_angle(bearing - travel_yaw).abs();
                let heading = 1.0 - off_target / std::f64::consts::PI;
                let clearance = gap.min(FULL_CLEARANCE_M) / FULL_CLEARANCE_M;
                let speed = twist.v.abs() / vehicle.max_speed();
                let reverse_penalty = if twist.v < 0.0 { REVERSE_PENALTY } else { 0.0 };

                let score = self.weights.heading * heading
                    + self.weights.clearance * clearance
                    + self.weights.speed * speed
                    - reverse_penalty;
                if best.is_none_or(|(best_score, _)| score > best_score) {
                    best = Some((score, twist));
                }
            }
        }

        match best {
            Some((_, twist)) => twist,
            // Every reachable twist ends in an obstacle: stop.
            None => Twist::STOP,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::PoseGoal;
    use crate::planning::global::{HybridAStar, PathPlanner};
    use crate::vehicle::{DiffDrive, Limits};

    fn test_vehicle() -> DiffDrive {
        DiffDrive {
            track: 0.30,
            radius: 0.25,
            limits: Limits { max_wheel_speed: 1.0, max_accel: 1.0, max_yaw_accel: 2.0 },
        }
    }

    fn default_dwa(max_reverse_speed: f64) -> Dwa {
        Dwa {
            horizon: 2.0,
            samples: 20,
            max_reverse_speed,
            weights: DwaWeights { heading: 2.0, clearance: 0.2, speed: 0.2 },
        }
    }

    #[test]
    fn drives_forward_when_path_is_ahead() {
        let vehicle = test_vehicle();
        let mut dwa = default_dwa(0.25);
        let state = State {
            t: 0.0,
            pose: Pose2 { x: 0.0, y: 0.0, yaw: 0.0 },
            twist: Twist::STOP,
        };
        let goal = Goal::Pose(PoseGoal { x: 2.0, y: 0.0, yaw: None, tolerance: 0.2 });
        let path = Path {
            poses: vec![Pose2 { x: 0.0, y: 0.0, yaw: 0.0 }, Pose2 { x: 2.0, y: 0.0, yaw: 0.0 }],
        };
        let twist = dwa.follow(&state, &Map::default(), &goal, &path, &vehicle);
        assert!(twist.v > 0.0, "expected forward motion, got {twist:?}");
    }

    #[test]
    fn forward_only_when_max_reverse_speed_is_zero() {
        let vehicle = test_vehicle();
        let mut dwa = default_dwa(0.0);
        let state = State {
            t: 0.0,
            pose: Pose2 { x: 0.0, y: 0.0, yaw: 0.0 },
            twist: Twist::STOP,
        };
        let goal = Goal::Pose(PoseGoal { x: -1.5, y: 0.0, yaw: None, tolerance: 0.2 });
        let path = Path {
            poses: vec![Pose2 { x: 0.0, y: 0.0, yaw: 0.0 }, Pose2 { x: -1.5, y: 0.0, yaw: 0.0 }],
        };
        let twist = dwa.follow(&state, &Map::default(), &goal, &path, &vehicle);
        assert!(twist.v >= 0.0, "expected non-negative v, got {twist:?}");
    }

    #[test]
    fn backs_out_with_hybrid_astar_in_dead_end() {
        let vehicle = test_vehicle();
        // U-shaped dead-end pocket around (0, 0) facing +X, open behind (-X).
        let mut obstacles = Vec::new();
        for i in -6..=6 {
            obstacles.push(Vec2 { x: 0.65, y: i as f64 * 0.1 });
        }
        for i in -6..=6 {
            obstacles.push(Vec2 { x: i as f64 * 0.1, y: 0.65 });
            obstacles.push(Vec2 { x: i as f64 * 0.1, y: -0.65 });
        }
        let map = Map { obstacles, resolution: 0.1, revision: 1 };
        let state = State {
            t: 0.0,
            pose: Pose2 { x: 0.0, y: 0.0, yaw: 0.0 },
            twist: Twist::STOP,
        };
        let goal = Goal::Pose(PoseGoal { x: -1.8, y: 0.0, yaw: None, tolerance: 0.2 });

        let mut planner = HybridAStar {
            resolution: 0.1,
            yaw_bins: 72,
            step: 0.3,
            heuristic_weight: 1.5,
            allow_reverse: true,
            reverse_penalty: 2.0,
        };
        let path = planner.plan(&state, &map, &goal, &vehicle).expect("hybrid_astar backout path");

        let mut dwa = default_dwa(0.25);
        let twist = dwa.follow(&state, &map, &goal, &path, &vehicle);
        assert!(twist.v < 0.0, "expected DWA to back out (v < 0), got {twist:?}");
    }
}
