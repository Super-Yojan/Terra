//! Deterministic local motion planning over observed occupancy.
//!
//! [`LocalPlanner::plan`] samples reachable velocities and chooses the safe one
//! closest to the requested intent. [`LocalPlanner::admissible`] checks the swept
//! footprint through the prediction horizon and braking period. [`frontier`] and
//! [`reachable`] search the grid; unknown cells are not treated as free paths.

mod astar;
pub use astar::*;
mod search;
pub use search::*;
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use terra_mapping::MapSnapshot;
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Twist {
    pub linear: f64,
    pub angular: f64,
}
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}
impl Pose {
    pub fn finite(self) -> bool {
        [self.x, self.y, self.yaw].iter().all(|v| v.is_finite())
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PlannerConfig {
    pub radius: f64,
    pub max_linear: f64,
    pub max_angular: f64,
    pub acceleration: f64,
    pub yaw_acceleration: f64,
    pub braking: f64,
    pub horizon: f64,
}
impl Default for PlannerConfig {
    fn default() -> Self {
        Self {
            radius: 0.65,
            max_linear: 2.,
            max_angular: 2.,
            acceleration: 1.,
            yaw_acceleration: 2.,
            braking: 1.,
            horizon: 2.,
        }
    }
}
#[derive(Clone, Debug, Default)]
pub struct LocalPlanner {
    pub config: PlannerConfig,
}
#[derive(Clone, Copy, Debug)]
pub struct PlanningDecision {
    pub twist: Twist,
    pub reason: &'static str,
}
/// Diagnostic geometry from the actual candidate checks. Points are millimetre
/// offsets from `origin`, keeping all 121 paths below the telemetry size budget.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct PlannerTrace {
    /// Seconds elapsed since this candidate set was evaluated.
    pub age: f64,
    pub origin: Pose,
    pub candidates: Vec<CandidateTrace>,
    pub selected: Option<usize>,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct CandidateTrace {
    pub velocity: Twist,
    pub points_mm: Vec<[i32; 2]>,
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
            && config.radius <= 5.
            && config.horizon <= 10.
    }
    /// Check the swept footprint through the motion horizon and the braking tail.
    /// Out-of-map, occupied, or newly entered unknown cells reject the trajectory.
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
            let rejection =
                footprint_rejection(map, sample_pose.x, sample_pose.y, config.radius, Some(pose));
            if record && (elapsed >= next_record || rejection.is_some()) {
                points.push(offset(sample_pose));
                next_record = elapsed + record_interval;
            }
            if rejection.is_some() {
                return (rejection, points);
            }
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
            footprint_rejection(map, sample_pose.x, sample_pose.y, config.radius, Some(pose)),
            points,
        )
    }
    /// Sample acceleration-bounded velocities and minimize deviation from intent.
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
        self.plan_recorded(map, pose, current, intent, dt, false).0
    }
    pub fn plan_with_trace(
        &self,
        map: &MapSnapshot,
        pose: Pose,
        current: Twist,
        intent: Twist,
        dt: f64,
    ) -> (PlanningDecision, PlannerTrace) {
        self.plan_recorded(map, pose, current, intent, dt, true)
    }
    fn plan_recorded(
        &self,
        map: &MapSnapshot,
        pose: Pose,
        current: Twist,
        intent: Twist,
        dt: f64,
        record: bool,
    ) -> (PlanningDecision, PlannerTrace) {
        let mut trace = PlannerTrace {
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
        let min_linear = (current.linear - config.acceleration * dt).max(-config.max_linear);
        let max_linear = (current.linear + config.acceleration * dt).min(config.max_linear);
        let min_angular = (current.angular - config.yaw_acceleration * dt).max(-config.max_angular);
        let max_angular = (current.angular + config.yaw_acceleration * dt).min(config.max_angular);
        let mut best: Option<(f64, Twist)> = None;
        for linear_sample in 0..=10 {
            for angular_sample in 0..=10 {
                let trajectory = Twist {
                    linear: min_linear + (max_linear - min_linear) * linear_sample as f64 / 10.,
                    angular: min_angular
                        + (max_angular - min_angular) * angular_sample as f64 / 10.,
                };
                let (rejection, points_mm) = self.evaluate(map, pose, trajectory, record);
                if record {
                    trace.candidates.push(CandidateTrace {
                        velocity: trajectory,
                        points_mm,
                        rejection: rejection.map(str::to_owned),
                    });
                }
                if rejection.is_some() {
                    continue;
                }
                let score = (trajectory.linear - intent.linear).powi(2)
                    + 0.3 * (trajectory.angular - intent.angular).powi(2);
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
/// Grid index of a world point. `None` when the map or the point is unusable.
pub fn grid_cell(m: &MapSnapshot, x: f64, y: f64) -> Option<(usize, usize)> {
    cell(m, x, y)
}
/// Footprint clearance on observed cells. Unknown cells count as blocked outside `current`.
pub fn footprint_clear(
    m: &MapSnapshot,
    x: f64,
    y: f64,
    radius: f64,
    current: Option<Pose>,
) -> bool {
    clear(m, x, y, radius, current)
}
/// Breadth-first search through inflated observed free cells. Stable ordering breaks ties.
///
/// Supervised mode proposes this point and waits for approval. Time-bounded autonomous
/// exploration lives in `terra-exploration` and uses the same clearance helpers.
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
