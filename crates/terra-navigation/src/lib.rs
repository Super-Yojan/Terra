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
//! - Use [`astar_route`] for a route to an observed goal, or [`astar_progress_route`]
//!   to make progress toward a goal beyond the observed window.
//! - Use [`SearchNavigator`] to accumulate observations and select exploration waypoints.
//!
//! # Coordinates and occupancy
//!
//! Positions and radii are in metres in the [`MapSnapshot`] world frame. Angles
//! are in radians. Occupancy is row-major: negative values mean unknown,
//! `0..65` means traversable, and values of at least `65` mean occupied.
//! Footprint checks conservatively include cells whose corners may touch the rover.
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

mod astar;
pub use astar::*;
mod search;
pub use search::*;
mod maneuver;
pub use maneuver::ManeuverSegment;
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
/// Motion limits and circular footprint used by the local planner.
///
/// All values must be finite and positive; radius is capped at 5 m and horizon at 10 s.
/// Invalid configurations cause [`LocalPlanner::plan`] to return a blocked decision.
///
/// # Defaults
///
/// The default footprint radius is 0.65 m, speed limits are 2 m/s and 2 rad/s,
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
    /// Circular rover footprint radius in metres.
    pub radius: f64,
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
            radius: 0.65,
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

fn valid_map(map: &MapSnapshot) -> bool {
    map.width > 0
        && map.height > 0
        && map.width <= 1000
        && map.height <= 1000
        && map.occupancy.len() == map.width as usize * map.height as usize
        && map.resolution.is_finite()
        && map.resolution >= 0.01
        && map.origin_x.is_finite()
        && map.origin_y.is_finite()
}
fn cell(map: &MapSnapshot, x: f64, y: f64) -> Option<(usize, usize)> {
    if !valid_map(map) || !x.is_finite() || !y.is_finite() {
        return None;
    }
    let column = ((x - map.origin_x) / map.resolution).floor();
    let row = ((y - map.origin_y) / map.resolution).floor();
    (column >= 0. && row >= 0. && column < (map.width as f64) && row < (map.height as f64))
        .then_some((column as usize, row as usize))
}
// Check every cell touched by the circular footprint, including cell corners.
// Unknown space is allowed only inside the footprint at the starting pose;
// that exception lets the rover leave its own unobserved underbody cells.
fn clear(map: &MapSnapshot, x: f64, y: f64, radius: f64, current: Option<Pose>) -> bool {
    footprint_rejection(map, x, y, radius, current).is_none()
}
fn footprint_rejection(
    map: &MapSnapshot,
    x: f64,
    y: f64,
    radius: f64,
    current: Option<Pose>,
) -> Option<&'static str> {
    let Some((center_column, center_row)) = cell(map, x, y) else {
        return Some("outside_map");
    };
    let cell_reach = (radius / map.resolution).ceil() as isize + 1;
    for dy in -cell_reach..=cell_reach {
        for dx in -cell_reach..=cell_reach {
            let column = center_column as isize + dx;
            let row = center_row as isize + dy;
            let cell_x = map.origin_x + (column as f64 + 0.5) * map.resolution;
            let cell_y = map.origin_y + (row as f64 + 0.5) * map.resolution;
            // Half a cell diagonal conservatively includes cells touched by the circle.
            if (cell_x - x).hypot(cell_y - y) > radius + map.resolution * 0.7072 {
                continue;
            }
            if column < 0 || row < 0 || column >= map.width as isize || row >= map.height as isize {
                return Some("outside_map");
            }
            let occupancy = map.occupancy[row as usize * map.width as usize + column as usize];
            if occupancy >= 65 {
                return Some("occupied");
            }
            if occupancy < 0
                && current.is_none_or(|sample_pose| {
                    (cell_x - sample_pose.x).hypot(cell_y - sample_pose.y)
                        > radius + map.resolution * 0.7072
                })
            {
                return Some("unknown");
            }
        }
    }
    None
}
impl LocalPlanner {
    fn valid(&self) -> bool {
        let config = &self.config;
        [
            config.max_reverse,
            config.radius,
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
            && config.radius <= 5.
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
            || !valid_map(map)
            || !pose.finite()
            || !trajectory.linear.is_finite()
            || !trajectory.angular.is_finite()
        {
            return (Some("invalid_input"), points);
        }
        let config = &self.config;
        // Advance at most half a cell per step, with a 100 ms time-step ceiling.
        let step = (map.resolution / (2. * trajectory.linear.abs().max(0.1))).min(0.1);
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
            let rejection = footprint_rejection(
                map,
                sample_pose.x,
                sample_pose.y,
                config.radius,
                if trajectory.linear < 0. {
                    None
                } else {
                    Some(pose)
                },
            );
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
            footprint_rejection(
                map,
                sample_pose.x,
                sample_pose.y,
                config.radius,
                if trajectory.linear < 0. {
                    None
                } else {
                    Some(pose)
                },
            ),
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
        limiter: &terra_control::TurningLimiter,
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
        turning: Option<(&terra_control::TurningLimiter, f64)>,
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
/// Convert world coordinates in metres to a zero-based `(column, row)` index.
///
/// Cells include their lower boundaries and exclude their upper boundaries.
/// Returns `None` for invalid map geometry, nonfinite coordinates, or a point
/// outside the map, including its upper x/y edges.
pub fn grid_cell(m: &MapSnapshot, x: f64, y: f64) -> Option<(usize, usize)> {
    cell(m, x, y)
}
/// Footprint clearance on observed cells. Unknown cells count as blocked outside `current`.
///
/// `(x, y)` is the footprint center and `radius` is in metres. With `current: None`,
/// every touched cell must be observed and unoccupied. With `Some(pose)`, unknown
/// cells within that starting footprint are tolerated. Map boundaries remain blocked.
///
/// Callers must supply a finite positive radius and, if present, a finite current
/// pose; this low-level helper does not validate those arguments independently.
pub fn footprint_clear(
    m: &MapSnapshot,
    x: f64,
    y: f64,
    radius: f64,
    current: Option<Pose>,
) -> bool {
    clear(m, x, y, radius, current)
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
/// Breadth-first search through inflated observed free cells. Stable ordering breaks ties.
///
/// Supervised mode proposes this point and waits for approval. Time-bounded autonomous
/// exploration lives in `terra-exploration` and uses the same clearance helpers.
///
/// # Selection and return value
///
/// Returns a reachable, observed cell center near unknown space, in world metres.
/// Candidates must be farther than `max(radius, 0.8)` m from `pose` and more than
/// 1 m from every point in `excluded`. Nearby unknown cells increase the score;
/// grid travel distance decreases it.
///
/// Returns `None` for invalid map/pose/radius input or when no candidate qualifies.
/// `radius` must be finite and in `0.0 < radius <= 5.0`.
pub fn frontier(
    map: &MapSnapshot,
    pose: Pose,
    radius: f64,
    excluded: &[(f64, f64)],
) -> Option<(f64, f64)> {
    if !pose.finite() || !radius.is_finite() || radius <= 0. || radius > 5. {
        return None;
    }
    let start = cell(map, pose.x, pose.y)?;
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
        if !clear(map, px, py, radius, Some(pose)) {
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
        if clear(map, px, py, radius, None)
            && unknown_neighbors > 0
            && (px - pose.x).hypot(py - pose.y) > radius.max(0.8)
            && excluded.iter().all(|&(a, b)| (px - a).hypot(py - b) > 1.)
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
/// Reachability check for proposal approval using observed, footprint-inflated free cells.
///
/// Tests four-connected cell-center reachability from `pose` to the cell containing
/// world point `(x, y)`. The target footprint must be entirely observed and clear;
/// unknown cells under the starting footprint are allowed during traversal.
/// Returns `false` for invalid inputs, a radius outside `0.0 < radius <= 5.0`,
/// a blocked target, or absence of a connected path. Use [`astar_route`] when a
/// polyline with checked connecting segments is required.
pub fn reachable(map: &MapSnapshot, pose: Pose, radius: f64, x: f64, y: f64) -> bool {
    let Some(start) = cell(map, pose.x, pose.y) else {
        return false;
    };
    let Some(target) = cell(map, x, y) else {
        return false;
    };
    if !pose.finite()
        || !radius.is_finite()
        || radius <= 0.
        || radius > 5.
        || !clear(map, x, y, radius, None)
    {
        return false;
    }
    let mut seen = vec![false; map.occupancy.len()];
    let mut queue = VecDeque::from([start]);
    while let Some((column, row)) = queue.pop_front() {
        let index = row * map.width as usize + column;
        if seen[index] {
            continue;
        }
        seen[index] = true;
        let cell_x = map.origin_x + (column as f64 + 0.5) * map.resolution;
        let cell_y = map.origin_y + (row as f64 + 0.5) * map.resolution;
        if !clear(map, cell_x, cell_y, radius, Some(pose)) {
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
/// Returns distance in metres from the circular footprint to the nearest occupied
/// cell's rectangular boundary. Unknown cells and map edges do not contribute.
/// Returns `None` for invalid input or when no occupied cell is observed, and zero
/// when the circular footprint touches or overlaps an observed obstacle.
pub fn observed_clearance(map: &MapSnapshot, pose: Pose, radius: f64) -> Option<f64> {
    if !valid_map(map) || !pose.finite() || !radius.is_finite() || radius <= 0. {
        return None;
    }
    let mut best = f64::INFINITY;
    for (i, v) in map.occupancy.iter().enumerate() {
        if *v < 65 {
            continue;
        }
        let x = map.origin_x + (i % map.width as usize) as f64 * map.resolution;
        let y = map.origin_y + (i / map.width as usize) as f64 * map.resolution;
        let dx = (x - pose.x).max(0.).max(pose.x - x - map.resolution);
        let dy = (y - pose.y).max(0.).max(pose.y - y - map.resolution);
        best = best.min(dx.hypot(dy) - radius);
    }
    best.is_finite().then_some(best.max(0.))
}
