//! Deterministic local motion planning over observed occupancy.
//!
//! [`LocalPlanner::plan`] samples reachable velocities and chooses the safe one
//! closest to the requested intent. [`LocalPlanner::admissible`] checks the swept
//! footprint through the prediction horizon and braking period. [`frontier`] and
//! [`reachable`] search the grid; unknown cells are not treated as free paths.
//!
//! # Choosing an API
//!
//! - Use [`LocalPlanner`] to turn a velocity request into a checked local command.
//! - Use [`RouteJob`] for a drivable route to a goal (Hybrid A* over position and
//!   heading, turning only along arcs), spread over control ticks, including goals
//!   beyond the observed window; [`hybrid_astar_route`] runs one search at once.
//! - Use [`SearchNavigator`] to accumulate observations and select exploration waypoints.
//!
//! # Coordinates and occupancy
//!
//! Positions are in metres in the [`MapSnapshot`] world frame. Angles are in
//! radians. Occupancy is row-major: negative values mean unknown, `0..65` means
//! traversable, and values of at least `65` mean occupied. The rover body is a
//! [`Footprint`] rectangle checked at its heading; a cell blocks it if the
//! rotated rectangle overlaps the cell at all.
//! Unknown cells under the starting footprint may be tolerated so the rover can
//! leave its unobserved underbody; this exception never permits occupied cells.
//!
//! Most planning helpers require nonzero map dimensions of at most 1,000 cells
//! per axis, matching occupancy storage, finite origins, and a finite resolution
//! of at least 0.01 m. Route searches additionally cap storage at 250,000 cells.
//!
//! # Example
//!
//! Request forward motion from rest on an observed, empty map:
//!
//! ```
//! use terra_mapping::MapSnapshot;
//! use terra_navigation::{LocalPlanner, Pose, Twist};
//!
//! let map = MapSnapshot {
//!     width: 40,
//!     height: 40,
//!     resolution: 0.25,
//!     origin_x: -5.0,
//!     origin_y: -5.0,
//!     occupancy: vec![0; 40 * 40],
//! };
//! let planner = LocalPlanner::default();
//! let desired = Twist { linear: 1.0, angular: 0.0 };
//! let decision = planner.plan(&map, Pose::default(), Twist::default(), desired, 0.1);
//!
//! // Acceleration limits allow only a small first step toward the requested speed.
//! assert!(decision.twist.linear > 0.0);
//! assert!(decision.twist.linear <= 0.1);
//! assert_eq!(decision.reason, "assisted");
//! ```

mod footprint;
pub use footprint::{
    FitGrid, Footprint, backoff_clear, footprint_fits, footprint_fits_some_heading,
    footprint_rejection_at, pull_forward_clear, rectangle_clearance,
};
mod hybrid_astar;
pub use hybrid_astar::{
    HybridConfig, HybridGoal, HybridSearch, HybridWaypoint, hybrid_astar_route,
};
mod route;
pub use route::{HybridRoute, RouteJob};
mod search;
pub use search::*;
mod maneuver;
mod memory;
pub use maneuver::ManeuverSegment;
pub use memory::{LongRangeMemory, MEMORY_CELLS, MEMORY_RESOLUTION};
fn direction(v: f64) -> &'static str {
    if v < 0. {
        "reverse"
    } else if v > 0. {
        "forward"
    } else {
        "stationary"
    }
}
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use terra_mapping::MapSnapshot;
/// Forward and yaw velocities in the rover's local frame.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Twist {
    /// Forward speed in metres per second; negative values command reverse motion.
    pub linear: f64,
    /// Yaw rate in radians per second.
    pub angular: f64,
}
/// Position and heading in the occupancy map's world frame.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct Pose {
    /// World x coordinate in metres.
    pub x: f64,
    /// World y coordinate in metres.
    pub y: f64,
    /// Heading in radians, measured from the positive x axis.
    pub yaw: f64,
}
impl Pose {
    /// Whether all components are finite; does not require a normalized heading.
    pub fn finite(self) -> bool {
        [self.x, self.y, self.yaw].iter().all(|v| v.is_finite())
    }
}
/// Motion limits and the rectangular body footprint used by the local planner.
///
/// All values must be finite and positive; the body is capped at 5 m and horizon at 10 s.
/// Invalid configurations cause [`LocalPlanner::plan`] to return a blocked decision.
///
/// # Defaults
///
/// The default footprint is Terra's 0.61 m square body with 5 cm margin, speed limits are 2 m/s and 2 rad/s,
/// acceleration limits are 1 m/s² and 2 rad/s², braking is 1 m/s², and the
/// prediction horizon is 2 s.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PlannerConfig {
    /// Conservative reverse speed cap, metres/second.
    #[serde(default = "default_reverse")]
    pub max_reverse: f64,
    /// Nonnegative cost per reverse metre/second.
    #[serde(default = "default_reverse_penalty")]
    pub reverse_penalty: f64,
    /// Rover body rectangle and margin, checked at the rover's heading.
    #[serde(default)]
    pub footprint: Footprint,
    /// Maximum forward or reverse speed magnitude in metres per second.
    pub max_linear: f64,
    /// Maximum yaw rate magnitude in radians per second.
    pub max_angular: f64,
    /// Linear acceleration limit in metres per second squared.
    pub acceleration: f64,
    /// Angular acceleration limit in radians per second squared.
    pub yaw_acceleration: f64,
    /// Linear deceleration used to predict stopping time, in metres per second squared.
    pub braking: f64,
    /// Constant-velocity prediction duration in seconds, before the braking tail.
    pub horizon: f64,
}
fn default_reverse() -> f64 {
    0.15
}
fn default_reverse_penalty() -> f64 {
    0.05
}
impl Default for PlannerConfig {
    fn default() -> Self {
        Self {
            max_reverse: default_reverse(),
            reverse_penalty: default_reverse_penalty(),
            footprint: Footprint::default(),
            max_linear: 2.,
            max_angular: 2.,
            acceleration: 1.,
            yaw_acceleration: 2.,
            braking: 1.,
            horizon: terra_types::timing::PLANNER_HORIZON_SECONDS,
        }
    }
}
/// Deterministic velocity sampler that checks motion against observed occupancy.
#[derive(Clone, Debug, Default)]
pub struct LocalPlanner {
    /// Footprint, prediction duration, and velocity limits for candidate evaluation.
    pub config: PlannerConfig,
}
/// Selected velocity and a machine-readable explanation of the intervention.
#[derive(Clone, Copy, Debug)]
pub struct PlanningDecision {
    /// Selected command, or zero velocity when planning is blocked.
    pub twist: Twist,
    /// `active`, `assisted`, or `obstacle_blocked`; invalid input also yields the latter.
    pub reason: &'static str,
}
/// Diagnostic geometry from the actual candidate checks. Points are millimetre
/// offsets from `origin`, keeping all 121 paths below the telemetry size budget.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct PlannerTrace {
    /// Nominal multi-point arc segments; refreshed/rechecked during execution.
    #[serde(default)]
    pub maneuver_segments: Vec<ManeuverSegment>,
    /// Seconds elapsed since this candidate set was evaluated.
    pub age: f64,
    /// World pose from which every candidate was simulated.
    pub origin: Pose,
    /// Candidates in linear-sample then angular-sample order.
    pub candidates: Vec<CandidateTrace>,
    /// Index into `candidates`, or `None` if no safe command was selected.
    pub selected: Option<usize>,
}
/// Sampled geometry and rejection status for one simulated velocity.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct CandidateTrace {
    /// Forward, reverse, or stationary.
    #[serde(default)]
    pub direction: String,
    /// Velocity evaluated for this candidate.
    pub velocity: Twist,
    /// World-axis x/y offsets from the trace origin, rounded to millimetres.
    pub points_mm: Vec<[i32; 2]>,
    /// First failed check, or `None` when the full trajectory was admissible.
    pub rejection: Option<String>,
}

impl LocalPlanner {
    fn valid(&self) -> bool {
        let config = &self.config;
        [
            config.max_reverse,
            config.max_linear,
            config.max_angular,
            config.acceleration,
            config.yaw_acceleration,
            config.braking,
            config.horizon,
        ]
        .iter()
        .all(|v| v.is_finite() && *v > 0.)
            && config.max_reverse <= 2.
            && config.reverse_penalty.is_finite()
            && config.reverse_penalty >= 0.
            && config.footprint.valid()
            && config.horizon <= 10.
    }
    /// Check the swept footprint through the motion horizon and the braking tail.
    ///
    /// Out-of-map, occupied, or newly entered unknown cells reject the trajectory.
    /// Invalid maps, configurations, poses, or nonfinite velocities return `false`.
    /// This checks collision admissibility only: it does not enforce acceleration
    /// or configured speed limits. Use [`Self::plan`] to sample a bounded command.
    pub fn admissible(&self, map: &MapSnapshot, pose: Pose, trajectory: Twist) -> bool {
        self.evaluate(map, pose, trajectory, false).0.is_none()
    }
    fn evaluate(
        &self,
        map: &MapSnapshot,
        pose: Pose,
        trajectory: Twist,
        record: bool,
    ) -> (Option<&'static str>, Vec<[i32; 2]>) {
        let mut points = Vec::new();
        if !self.valid()
            || !map.valid()
            || !pose.finite()
            || !trajectory.linear.is_finite()
            || !trajectory.angular.is_finite()
        {
            return (Some("invalid_input"), points);
        }
        let config = &self.config;
        // No point of the body moves more than half a cell per step (translation
        // plus a corner swinging round), with a 100 ms ceiling. Growing the body
        // by a quarter cell then covers everything swept between samples.
        let corner_speed =
            trajectory.linear.abs() + trajectory.angular.abs() * config.footprint.circumradius();
        let step = (map.resolution / (2. * corner_speed.max(0.1))).min(0.1);
        let pad = map.resolution / 4.;
        // A pure spin sweeps the disc around the body; tolerate unknown cells there
        // as the circle planner did. Reversing never enters unknown space.
        let unknown = if trajectory.linear == 0. && trajectory.angular != 0. {
            footprint::Unknown::SpinDisc(pose)
        } else if trajectory.linear >= 0. {
            footprint::Unknown::UnderBody(pose)
        } else {
            footprint::Unknown::Refused
        };
        let duration = config.horizon + trajectory.linear.abs() / config.braking;
        let record_interval = (duration / 14.).max(step);
        let mut next_record = 0.;
        let mut sample_pose = pose;
        let mut elapsed = 0.;
        let offset = |p: Pose| {
            [
                ((p.x - pose.x) * 1000.).round() as i32,
                ((p.y - pose.y) * 1000.).round() as i32,
            ]
        };
        while elapsed < duration {
            let rejection = footprint::rejection(map, &config.footprint, sample_pose, pad, unknown);
            if record && (elapsed >= next_record || rejection.is_some()) {
                points.push(offset(sample_pose));
                next_record = elapsed + record_interval;
            }
            if rejection.is_some() {
                return (rejection, points);
            }
            // Scale both velocities together during braking to preserve path curvature.
            let scale = if elapsed <= config.horizon {
                1.
            } else {
                (1. - (elapsed - config.horizon) * config.braking
                    / trajectory.linear.abs().max(0.001))
                .max(0.)
            };
            sample_pose.yaw += trajectory.angular * scale * step;
            sample_pose.x += trajectory.linear * scale * sample_pose.yaw.cos() * step;
            sample_pose.y += trajectory.linear * scale * sample_pose.yaw.sin() * step;
            elapsed += step;
        }
        if record {
            points.push(offset(sample_pose));
        }
        (
            footprint::rejection(map, &config.footprint, sample_pose, pad, unknown),
            points,
        )
    }
    /// Sample acceleration-bounded velocities and minimize deviation from intent.
    ///
    /// `pose` is the current world pose, `current` is the current velocity, and
    /// `intent` is the requested velocity. `dt` is the control interval in seconds
    /// and must be finite and in `0.0..=0.5`.
    ///
    /// Evaluates an 11 × 11 velocity grid using [`Self::admissible`]'s collision
    /// checks. Candidates minimize squared linear error plus 0.3 times squared
    /// angular error; equal scores retain the first candidate in sample order.
    ///
    /// # Returns
    ///
    /// The reason is `active` when both velocity errors are at most 0.15 in their
    /// respective units, and `assisted` when either error exceeds that threshold.
    /// Returns zero twist with `obstacle_blocked` when no candidate is admissible
    /// or the planning inputs are invalid; the reason alone does not prove an obstacle.
    pub fn plan(
        &self,
        map: &MapSnapshot,
        pose: Pose,
        current: Twist,
        intent: Twist,
        dt: f64,
    ) -> PlanningDecision {
        self.plan_recorded(map, pose, current, intent, dt, false, None)
            .0
    }
    /// Plan as in [`Self::plan`], also recording candidate paths and the selected index.
    ///
    /// The trace starts at age zero; callers update its age when retaining it for telemetry.
    /// Candidate geometry includes the braking tail and stops at the first failed
    /// footprint check. Inputs rejected before sampling produce an empty trace.
    /// [`PlannerTrace::selected`] indexes the candidate matching the returned command.
    pub fn plan_with_trace(
        &self,
        map: &MapSnapshot,
        pose: Pose,
        current: Twist,
        intent: Twist,
        dt: f64,
    ) -> (PlanningDecision, PlannerTrace) {
        self.plan_recorded(map, pose, current, intent, dt, true, None)
    }
    /// Preview managed candidates before collision checks; commit the selected twist in the arbiter.
    pub fn plan_with_turning(
        &self,
        map: &MapSnapshot,
        pose: Pose,
        current: Twist,
        intent: Twist,
        dt: f64,
        limiter: &terra_types::turning::TurningLimiter,
        now: f64,
    ) -> (PlanningDecision, PlannerTrace) {
        self.plan_recorded(map, pose, current, intent, dt, true, Some((limiter, now)))
    }
    fn plan_recorded(
        &self,
        map: &MapSnapshot,
        pose: Pose,
        current: Twist,
        intent: Twist,
        dt: f64,
        record: bool,
        turning: Option<(&terra_types::turning::TurningLimiter, f64)>,
    ) -> (PlanningDecision, PlannerTrace) {
        let mut trace = PlannerTrace {
            maneuver_segments: Vec::new(),
            age: 0.,
            origin: pose,
            candidates: Vec::new(),
            selected: None,
        };
        let stop = PlanningDecision {
            twist: Twist::default(),
            reason: "obstacle_blocked",
        };
        if !self.valid()
            || !pose.finite()
            || !dt.is_finite()
            || !(0.0..=0.5).contains(&dt)
            || [
                current.linear,
                current.angular,
                intent.linear,
                intent.angular,
            ]
            .iter()
            .any(|v| !v.is_finite())
        {
            return (stop, trace);
        }
        let config = &self.config;
        let min_linear = (current.linear - config.acceleration * dt).clamp(
            -config.max_reverse.min(config.max_linear),
            config.max_linear,
        );
        let max_linear = (current.linear + config.acceleration * dt)
            .min(config.max_linear)
            .clamp(
                -config.max_reverse.min(config.max_linear),
                config.max_linear,
            );
        let min_angular = (current.angular - config.yaw_acceleration * dt)
            .clamp(-config.max_angular, config.max_angular);
        let max_angular = (current.angular + config.yaw_acceleration * dt)
            .clamp(-config.max_angular, config.max_angular);
        let mut best: Option<(f64, Twist)> = None;
        // Fixed sample order and strict score improvement make ties deterministic.
        for linear_sample in 0..=10 {
            for angular_sample in 0..=10 {
                let trajectory = Twist {
                    linear: min_linear + (max_linear - min_linear) * linear_sample as f64 / 10.,
                    angular: min_angular
                        + (max_angular - min_angular) * angular_sample as f64 / 10.,
                };
                let trajectory = if let Some((limiter, now)) = turning {
                    let Ok(t) = limiter.clone().step(terra_types::VelocityTarget {
                        timestamp: now,
                        forward: trajectory.linear,
                        yaw_rate: trajectory.angular,
                    }) else {
                        continue;
                    };
                    Twist {
                        linear: t.forward,
                        angular: t.yaw_rate,
                    }
                } else {
                    trajectory
                };
                let (rejection, points_mm) = self.evaluate(map, pose, trajectory, record);
                if record {
                    trace.candidates.push(CandidateTrace {
                        direction: direction(trajectory.linear).into(),
                        velocity: trajectory,
                        points_mm,
                        rejection: rejection.map(str::to_owned),
                    });
                }
                if rejection.is_some() {
                    continue;
                }
                let score = (trajectory.linear - intent.linear).powi(2)
                    + 0.3 * (trajectory.angular - intent.angular).powi(2)
                    + config.reverse_penalty * (-trajectory.linear).max(0.);
                if best.is_none_or(|b| score < b.0) {
                    best = Some((score, trajectory));
                    if record {
                        trace.selected = Some(trace.candidates.len() - 1);
                    }
                }
            }
        }
        let decision = best
            .map(|(_, twist)| PlanningDecision {
                twist,
                reason: if (twist.linear - intent.linear).abs() > 0.15
                    || (twist.angular - intent.angular).abs() > 0.15
                {
                    "assisted"
                } else {
                    "active"
                },
            })
            .unwrap_or(stop);
        (decision, trace)
    }
}
/// Whether the body fits centred at `(x, y)` at some heading (one of eight, 22.5
/// degrees apart). Use it where the heading is not yet known.
///
/// With `current: None`, every cell the body touches must be observed and
/// unoccupied. With `Some(pose)`, unknown cells under the body at that starting
/// pose are tolerated. Map boundaries remain blocked.
pub fn footprint_clear(
    m: &MapSnapshot,
    footprint: &Footprint,
    x: f64,
    y: f64,
    current: Option<Pose>,
) -> bool {
    footprint::footprint_fits_some_heading(m, footprint, x, y, current)
}
/// Copy of `map` with every unknown cell marked free. Observed cells and map bounds are kept.
///
/// Exploration through unknown space plans routes, local rollouts and the
/// look-around turn on this copy, so it relies on the local planner stopping for
/// obstacles as the camera observes them. Occupied cells still reject motion.
pub fn unknown_as_free(map: &MapSnapshot) -> MapSnapshot {
    let mut copy = map.clone();
    for cell in &mut copy.occupancy {
        if *cell < 0 {
            *cell = 0;
        }
    }
    copy
}
/// Breadth-first search over cells where the body fits at some heading. Stable
/// ordering breaks ties.
///
/// Supervised mode proposes this point and waits for approval. Time-bounded autonomous
/// exploration lives in `terra-exploration` and uses the same clearance helpers.
///
/// # Selection and return value
///
/// Returns a reachable, observed cell center near unknown space, in world metres.
/// Candidates must be farther than `max(r, 0.8)` m from `pose`, where `r` is the
/// circle around the body, and more than 1 m from every point in `excluded`.
/// Nearby unknown cells increase the score; grid travel distance decreases it.
///
/// Returns `None` for invalid map, pose or footprint, or when no candidate qualifies.
pub fn frontier(
    map: &MapSnapshot,
    pose: Pose,
    footprint: &Footprint,
    excluded: &[(f64, f64)],
) -> Option<(f64, f64)> {
    if !pose.finite() || !footprint.valid() {
        return None;
    }
    let radius = footprint.circumradius();
    let start = map.cell(pose.x, pose.y)?;
    let mut traverse = FitGrid::new(map, footprint, Some(pose));
    let mut observed = FitGrid::new(map, footprint, None);
    let mut seen = vec![false; map.occupancy.len()];
    let mut queue = VecDeque::from([(start.0, start.1, 0usize)]);
    let mut best: Option<(f64, (f64, f64))> = None;
    while let Some((x, y, distance_cells)) = queue.pop_front() {
        let index = y * map.width as usize + x;
        if seen[index] {
            continue;
        }
        seen[index] = true;
        let px = map.origin_x + (x as f64 + 0.5) * map.resolution;
        let py = map.origin_y + (y as f64 + 0.5) * map.resolution;
        if !traverse.fits(x, y) {
            continue;
        }
        let reach = (radius / map.resolution).ceil() as isize + 2;
        let mut unknown_neighbors = 0;
        for dy in -reach..=reach {
            for dx in -reach..=reach {
                let a = x as isize + dx;
                let b = y as isize + dy;
                if a >= 0
                    && b >= 0
                    && a < map.width as isize
                    && b < map.height as isize
                    && map.occupancy[b as usize * map.width as usize + a as usize] < 0
                {
                    unknown_neighbors += 1;
                }
            }
        }
        if unknown_neighbors > 0
            && (px - pose.x).hypot(py - pose.y) > radius.max(0.8)
            && excluded.iter().all(|&(a, b)| (px - a).hypot(py - b) > 1.)
            && observed.fits(x, y)
        {
            // Favor nearby observation opportunities; keep the first equal-scoring cell.
            let score = unknown_neighbors as f64 / (1. + distance_cells as f64);
            if best.is_none_or(|b| score > b.0) {
                best = Some((score, (px, py)));
            }
        }
        for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
            let a = x as isize + dx;
            let b = y as isize + dy;
            if a >= 0 && b >= 0 && a < map.width as isize && b < map.height as isize {
                queue.push_back((a as usize, b as usize, distance_cells + 1));
            }
        }
    }
    best.map(|b| b.1)
}
/// A quick, heading-free reachability filter.
///
/// Tests four-connected reachability from `pose` to the cell containing `(x, y)`
/// through cells where the body fits at some heading. The body at the target must
/// fit in fully observed space; unknown cells under the starting body are allowed
/// during traversal. It ignores the turning radius, so it can accept a goal the
/// rover cannot turn into: confirm real decisions with [`hybrid_astar_route`] or
/// [`RouteJob`]. Returns `false` for invalid inputs, a blocked target, or no path.
pub fn reachable(map: &MapSnapshot, pose: Pose, footprint: &Footprint, x: f64, y: f64) -> bool {
    let Some(start) = map.cell(pose.x, pose.y) else {
        return false;
    };
    let Some(target) = map.cell(x, y) else {
        return false;
    };
    if !pose.finite() || !footprint.valid() || !footprint_clear(map, footprint, x, y, None) {
        return false;
    }
    let mut grid = FitGrid::new(map, footprint, Some(pose));
    let mut seen = vec![false; map.occupancy.len()];
    let mut queue = VecDeque::from([start]);
    while let Some((column, row)) = queue.pop_front() {
        let index = row * map.width as usize + column;
        if seen[index] {
            continue;
        }
        seen[index] = true;
        if !grid.fits(column, row) {
            continue;
        }
        if (column, row) == target {
            return true;
        }
        for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
            let nx = column as isize + dx;
            let ny = row as isize + dy;
            if nx >= 0 && ny >= 0 && nx < map.width as isize && ny < map.height as isize {
                queue.push_back((nx as usize, ny as usize));
            }
        }
    }
    false
}
/// Clearance to observed obstacle cell boundaries; unknown space is not evidence of clearance.
///
/// Returns distance in metres from the body rectangle (with margin, at the pose's
/// heading) to the nearest occupied cell; it can read up to half a cell diagonal
/// low, never high. Unknown cells and map edges do not contribute.
/// Returns `None` for invalid input or when no occupied cell is observed, and zero
/// when the body touches or overlaps an observed obstacle.
pub fn observed_clearance(map: &MapSnapshot, pose: Pose, footprint: &Footprint) -> Option<f64> {
    footprint::rectangle_clearance(map, footprint, pose, f64::INFINITY)
}
