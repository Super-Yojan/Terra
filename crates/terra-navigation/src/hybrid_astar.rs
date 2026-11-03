//! Hybrid A*: global routing for a rover that must not turn in place.
//!
//! The search runs over continuous poses `(x, y, yaw)`. Each expansion drives a
//! short arc whose curvature never exceeds `1 / min_turn_radius`, forward or (if
//! allowed) in reverse, so every route is drivable without skidding the rover
//! around its centre. Poses are binned by map cell and heading only to stop the
//! search revisiting near-identical states; the poses themselves stay exact.
//!
//! Collisions use the rover's real rectangle at its real heading, so the rover
//! can line itself up and pass gaps narrower than the circle around its corners.
//! Unknown cells count as blocked, except those under the starting footprint.
//!
//! The heuristic is the larger of the straight-line distance and an
//! obstacle-aware 2D distance, both scaled so they never overestimate. With a goal
//! heading, a Dubins curve to the goal is tried as a shortcut near the goal.
use crate::footprint::{OCCUPIED, for_each_overlapping_cell};
use crate::{Footprint, Pose, cell, valid_map};
use serde::{Deserialize, Serialize};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::f64::consts::{PI, SQRT_2, TAU};
use terra_mapping::MapSnapshot;

/// Longest 8-connected grid distance relative to the straight-line distance.
const OCTILE_OVER_EUCLID: f64 = 1.082_392_2;
/// Half a cell diagonal, in cells.
const HALF_DIAGONAL: f64 = std::f64::consts::FRAC_1_SQRT_2;
/// Largest map the search accepts, matching `astar_route`.
const MAX_CELLS: usize = 250_000;

/// Rover shape, turning limit and search costs for [`hybrid_astar_route`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct HybridConfig {
    /// Rover body rectangle and the margin kept around it.
    pub footprint: Footprint,
    /// Tightest turn the rover may drive, metres. Never zero: no turning in place.
    pub min_turn_radius: f64,
    /// Whether the route may drive backwards.
    pub allow_reverse: bool,
    /// Cost multiplier for reverse distance; at least 1.
    pub reverse_penalty: f64,
    /// Added cost, in metres, each time the route changes between forward and reverse.
    pub switch_penalty: f64,
    /// Extra cost fraction for driving the tightest turn (scaled down for gentler turns).
    pub steer_penalty: f64,
    /// Added cost, in metres, each time the curvature changes.
    pub steer_change_penalty: f64,
    /// Number of heading bins; 72 is 5 degrees.
    pub heading_bins: u32,
    /// Distance from the goal position that counts as arriving, metres.
    pub goal_tolerance: f64,
    /// Heading error that counts as arriving when the goal has a heading, radians.
    pub goal_heading_tolerance: f64,
    /// Expansions before the search gives up with `route_search_limit`.
    pub max_expansions: u32,
}

impl Default for HybridConfig {
    /// The Terra profile body (0.61 m square) and the default skid-safe turn radius.
    fn default() -> Self {
        Self {
            footprint: Footprint::default(),
            min_turn_radius: 0.5,
            allow_reverse: true,
            reverse_penalty: 2.0,
            switch_penalty: 1.0,
            steer_penalty: 0.2,
            steer_change_penalty: 0.1,
            heading_bins: 72,
            goal_tolerance: 0.15,
            goal_heading_tolerance: 0.1,
            max_expansions: 200_000,
        }
    }
}

impl HybridConfig {
    /// The planner's footprint and the turn radius from the turning policy.
    pub fn for_body(footprint: Footprint, turning: &terra_control::TurningConfig) -> Self {
        Self {
            footprint,
            min_turn_radius: turning.min_turn_radius,
            ..Self::default()
        }
    }

    fn valid(&self) -> bool {
        let positive = [
            self.min_turn_radius,
            self.goal_tolerance,
            self.goal_heading_tolerance,
        ];
        let nonnegative = [
            self.switch_penalty,
            self.steer_penalty,
            self.steer_change_penalty,
        ];
        positive.iter().all(|v| v.is_finite() && *v > 0.)
            && nonnegative.iter().all(|v| v.is_finite() && *v >= 0.)
            && self.footprint.valid()
            && self.min_turn_radius <= 20.
            && self.reverse_penalty.is_finite()
            && self.reverse_penalty >= 1.
            && (8..=360).contains(&self.heading_bins)
            && self.max_expansions > 0
    }
}

/// Where to go. With `yaw`, the rover must also arrive facing that way.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct HybridGoal {
    pub x: f64,
    pub y: f64,
    pub yaw: Option<f64>,
}

/// One sample of a Hybrid A* route.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct HybridWaypoint {
    pub x: f64,
    pub y: f64,
    /// Body heading in radians.
    pub yaw: f64,
    /// The rover drives backwards to reach this sample.
    pub reverse: bool,
    /// Signed path curvature, 1/metres; positive turns left. Never above `1 / min_turn_radius`.
    pub curvature: f64,
}

/// Plan a drivable route for a rectangular rover that turns only along arcs.
///
/// The route starts at `start` (the body centre) and is sampled no further than
/// one map cell apart. It ends within `goal_tolerance` of the goal position and,
/// when `goal.yaw` is set, within `goal_heading_tolerance` of that heading.
///
/// # Errors
/// - `route_invalid_map`: invalid map or configuration, nonfinite start, or more
///   than 250,000 cells.
/// - `route_start_outside_map` / `route_goal_outside_map`.
/// - `route_start_blocked`: the body at `start` touches an obstacle.
/// - `route_goal_blocked`: the body cannot fit at the goal.
/// - `route_no_path`: no drivable route exists.
/// - `route_search_limit`: `max_expansions` was reached first.
pub fn hybrid_astar_route(
    map: &MapSnapshot,
    start: Pose,
    goal: HybridGoal,
    config: &HybridConfig,
) -> Result<Vec<HybridWaypoint>, &'static str> {
    if !valid_map(map)
        || map.occupancy.len() > MAX_CELLS
        || !config.valid()
        || !start.finite()
        || goal.yaw.is_some_and(|yaw| !yaw.is_finite())
    {
        return Err("route_invalid_map");
    }
    let start_cell = cell(map, start.x, start.y).ok_or("route_start_outside_map")?;
    let goal_cell = cell(map, goal.x, goal.y).ok_or("route_goal_outside_map")?;
    let search = Search::new(map, start, config);
    if !search.body_clear(start.x, start.y, start.yaw, 0.) {
        return Err("route_start_blocked");
    }
    if let Some(yaw) = goal.yaw
        && !search.body_clear(goal.x, goal.y, yaw, 0.)
    {
        return Err("route_goal_blocked");
    }
    let heuristic = search.heuristic_field(goal_cell);
    if !heuristic[index(map, goal_cell)].is_finite() {
        return Err("route_goal_blocked");
    }
    if !heuristic[index(map, start_cell)].is_finite() {
        return Err("route_no_path");
    }
    search.run(goal, &heuristic)
}

/// One expanded pose and the arc that reached it from its parent.
#[derive(Clone, Copy)]
struct Node {
    x: f64,
    y: f64,
    yaw: f64,
    g: f64,
    parent: Option<usize>,
    curvature: f64,
    /// Signed arc length from the parent; negative is reverse.
    length: f64,
}

struct Search<'a> {
    map: &'a MapSnapshot,
    config: &'a HybridConfig,
    start: Pose,
    /// Unknown cells under the starting body, which the rover may drive off.
    tolerated: HashSet<usize>,
    /// Lower-bound distance (cells, 8-connected) to the nearest blocked or unknown cell.
    blocked_distance: Vec<f64>,
    kappa: f64,
    step: f64,
}

impl<'a> Search<'a> {
    fn new(map: &'a MapSnapshot, start: Pose, config: &'a HybridConfig) -> Self {
        let mut search = Self {
            map,
            config,
            start,
            tolerated: HashSet::new(),
            blocked_distance: distance_field(map, |v| !(0..OCCUPIED).contains(&v)),
            kappa: 1. / config.min_turn_radius,
            // Long enough to leave the current cell from any heading.
            step: map.resolution * 1.6,
        };
        let (a, b) = search.half_extents(0.);
        for_each_overlapping_cell(map, start.x, start.y, start.yaw, a, b, |i| {
            if let Some(i) = i
                && map.occupancy[i] < 0
            {
                search.tolerated.insert(i);
            }
            true
        });
        search
    }

    fn half_extents(&self, pad: f64) -> (f64, f64) {
        self.config.footprint.half_extents(pad)
    }

    /// Whether the body, grown by `pad` on every side, is clear at this pose.
    fn body_clear(&self, x: f64, y: f64, yaw: f64, pad: f64) -> bool {
        let (a, b) = self.half_extents(pad);
        let Some(c) = cell(self.map, x, y) else {
            return false;
        };
        // Far from everything blocked or unknown: no need to look at cells.
        let lower_bound = (self.blocked_distance[index(self.map, c)] / OCTILE_OVER_EUCLID
            - 2. * HALF_DIAGONAL)
            * self.map.resolution;
        if lower_bound > a.hypot(b) {
            return true;
        }
        for_each_overlapping_cell(self.map, x, y, yaw, a, b, |i| match i {
            None => false,
            Some(i) => {
                let v = self.map.occupancy[i];
                (0..OCCUPIED).contains(&v) || v < 0 && self.tolerated.contains(&i)
            }
        })
    }

    /// Whether the body stays clear along an arc of signed `length` from `from`.
    fn arc_clear(&self, from: (f64, f64, f64), curvature: f64, length: f64) -> bool {
        // Sample so no body point moves more than half a cell between checks, then
        // grow the body by a quarter cell: the swept area is then fully covered.
        let (a, b) = self.half_extents(0.);
        let reach = 1. + a.hypot(b) * curvature.abs();
        let spacing = (self.map.resolution / 2.) / reach;
        let samples = (length.abs() / spacing).ceil().max(1.) as usize;
        (1..=samples).all(|k| {
            let (x, y, yaw) = drive(from, curvature, length * k as f64 / samples as f64);
            self.body_clear(x, y, yaw, self.map.resolution / 4.)
        })
    }

    /// Admissible 2D distance (metres) from every cell to the goal, through cells
    /// the body's inscribed circle could occupy. Infinite where unreachable.
    fn heuristic_field(&self, goal: (usize, usize)) -> Vec<f64> {
        let (_, inscribed) = self.half_extents(0.);
        let res = self.map.resolution;
        let obstacle = distance_field(self.map, |v| v >= OCCUPIED);
        let passable: Vec<bool> = obstacle
            .iter()
            .map(|d| (d + HALF_DIAGONAL) * res >= inscribed)
            .collect();
        let mut field = vec![f64::INFINITY; self.map.occupancy.len()];
        let source = index(self.map, goal);
        if !passable[source] {
            return field;
        }
        field[source] = 0.;
        let mut open = BinaryHeap::from([Open(0., source)]);
        while let Some(Open(d, i)) = open.pop() {
            if d > field[i] {
                continue;
            }
            for (j, step) in neighbours(self.map, i) {
                let next = d + step * res / OCTILE_OVER_EUCLID;
                if passable[j] && next < field[j] {
                    field[j] = next;
                    open.push(Open(next, j));
                }
            }
        }
        field
    }

    fn h(&self, heuristic: &[f64], x: f64, y: f64, goal: HybridGoal) -> f64 {
        let Some(c) = cell(self.map, x, y) else {
            return f64::INFINITY;
        };
        let straight = (goal.x - x).hypot(goal.y - y);
        let grid = heuristic[index(self.map, c)] - 2. * HALF_DIAGONAL * self.map.resolution;
        straight.max(grid)
    }

    fn bin(&self, x: f64, y: f64, yaw: f64) -> Option<usize> {
        let c = cell(self.map, x, y)?;
        let bins = self.config.heading_bins as usize;
        let heading = ((yaw.rem_euclid(TAU) / TAU) * bins as f64) as usize % bins;
        Some(index(self.map, c) * bins + heading)
    }

    fn arrived(&self, x: f64, y: f64, yaw: f64, goal: HybridGoal) -> bool {
        (goal.x - x).hypot(goal.y - y) <= self.config.goal_tolerance
            && goal
                .yaw
                .is_none_or(|g| wrap(yaw - g).abs() <= self.config.goal_heading_tolerance)
    }

    fn run(
        &self,
        goal: HybridGoal,
        heuristic: &[f64],
    ) -> Result<Vec<HybridWaypoint>, &'static str> {
        let start = Node {
            x: self.start.x,
            y: self.start.y,
            yaw: self.start.yaw,
            g: 0.,
            parent: None,
            curvature: 0.,
            length: 0.,
        };
        let mut nodes = vec![start];
        let mut best: HashMap<usize, f64> = HashMap::new();
        let mut closed: HashSet<usize> = HashSet::new();
        let mut open = BinaryHeap::from([Open(self.h(heuristic, start.x, start.y, goal), 0)]);
        let curvatures = [-1., -0.5, 0., 0.5, 1.].map(|k| k * self.kappa);
        let directions: &[f64] = if self.config.allow_reverse {
            &[1., -1.]
        } else {
            &[1.]
        };
        let mut expansions = 0u32;
        while let Some(Open(_, id)) = open.pop() {
            let node = nodes[id];
            let Some(key) = self.bin(node.x, node.y, node.yaw) else {
                continue;
            };
            if !closed.insert(key) {
                continue;
            }
            if self.arrived(node.x, node.y, node.yaw, goal) {
                return Ok(self.waypoints(&nodes, id, None));
            }
            expansions += 1;
            if expansions > self.config.max_expansions {
                return Err("route_search_limit");
            }
            if let Some(shortcut) = self.dubins_shortcut(&node, goal, expansions) {
                return Ok(self.waypoints(&nodes, id, Some(shortcut)));
            }
            for &direction in directions {
                for &curvature in &curvatures {
                    let length = direction * self.step;
                    let from = (node.x, node.y, node.yaw);
                    let (x, y, yaw) = drive(from, curvature, length);
                    let Some(child_key) = self.bin(x, y, yaw) else {
                        continue;
                    };
                    if closed.contains(&child_key) {
                        continue;
                    }
                    let g = node.g + self.cost(&node, curvature, length);
                    if best.get(&child_key).is_some_and(|&b| b <= g) {
                        continue;
                    }
                    if !self.arc_clear(from, curvature, length) {
                        continue;
                    }
                    let h = self.h(heuristic, x, y, goal);
                    if !h.is_finite() {
                        continue;
                    }
                    best.insert(child_key, g);
                    nodes.push(Node {
                        x,
                        y,
                        yaw,
                        g,
                        parent: Some(id),
                        curvature,
                        length,
                    });
                    open.push(Open(g + h, nodes.len() - 1));
                }
            }
        }
        Err("route_no_path")
    }

    fn cost(&self, parent: &Node, curvature: f64, length: f64) -> f64 {
        let c = self.config;
        let reverse = length < 0.;
        let mut cost = length.abs() * (1. + c.steer_penalty * curvature.abs() / self.kappa);
        if reverse {
            cost *= c.reverse_penalty;
        }
        if parent.parent.is_some() {
            if (parent.length < 0.) != reverse {
                cost += c.switch_penalty;
            }
            if (parent.curvature - curvature).abs() > 1e-9 {
                cost += c.steer_change_penalty;
            }
        }
        cost
    }

    /// Near a goal with a heading, try a collision-free forward Dubins curve to it.
    fn dubins_shortcut(
        &self,
        node: &Node,
        goal: HybridGoal,
        expansions: u32,
    ) -> Option<Vec<(f64, f64)>> {
        let yaw = goal.yaw?;
        let near = (goal.x - node.x).hypot(goal.y - node.y) <= 10. * self.config.min_turn_radius;
        if !near && !expansions.is_multiple_of(10) {
            return None;
        }
        let path = dubins(
            (node.x, node.y, node.yaw),
            (goal.x, goal.y, yaw),
            self.config.min_turn_radius,
        )?;
        let mut at = (node.x, node.y, node.yaw);
        for &(curvature, length) in &path {
            if !self.arc_clear(at, curvature, length) {
                return None;
            }
            at = drive(at, curvature, length);
        }
        Some(path)
    }

    /// Route samples from the start to node `last`, then along `shortcut`.
    fn waypoints(
        &self,
        nodes: &[Node],
        last: usize,
        shortcut: Option<Vec<(f64, f64)>>,
    ) -> Vec<HybridWaypoint> {
        let mut chain = vec![last];
        while let Some(parent) = nodes[*chain.last().unwrap()].parent {
            chain.push(parent);
        }
        chain.reverse();
        let first = nodes[chain[0]];
        let mut out = vec![HybridWaypoint {
            x: first.x,
            y: first.y,
            yaw: first.yaw,
            reverse: false,
            curvature: 0.,
        }];
        let mut arcs: Vec<(f64, f64)> = chain[1..]
            .iter()
            .map(|&i| (nodes[i].curvature, nodes[i].length))
            .collect();
        arcs.extend(shortcut.unwrap_or_default());
        let mut at = (first.x, first.y, first.yaw);
        for (curvature, length) in arcs {
            let samples = (length.abs() / self.map.resolution).ceil().max(1.) as usize;
            for k in 1..=samples {
                let (x, y, yaw) = drive(at, curvature, length * k as f64 / samples as f64);
                out.push(HybridWaypoint {
                    x,
                    y,
                    yaw: wrap(yaw),
                    reverse: length < 0.,
                    curvature,
                });
            }
            at = drive(at, curvature, length);
        }
        out
    }
}

/// Pose after driving signed arc `length` at constant `curvature` from `from`.
fn drive((x, y, yaw): (f64, f64, f64), curvature: f64, length: f64) -> (f64, f64, f64) {
    if curvature.abs() < 1e-12 {
        return (x + length * yaw.cos(), y + length * yaw.sin(), yaw);
    }
    let end = yaw + curvature * length;
    (
        x + (end.sin() - yaw.sin()) / curvature,
        y - (end.cos() - yaw.cos()) / curvature,
        end,
    )
}

fn wrap(angle: f64) -> f64 {
    (angle + PI).rem_euclid(TAU) - PI
}

fn index(map: &MapSnapshot, (column, row): (usize, usize)) -> usize {
    row * map.width as usize + column
}

/// 8-connected distance, in cells, from each cell to the nearest blocked cell or
/// the map edge. It is at least the straight-line distance between cell centres.
fn distance_field(map: &MapSnapshot, blocked: impl Fn(i8) -> bool) -> Vec<f64> {
    let (w, h) = (map.width as usize, map.height as usize);
    let mut field = vec![f64::INFINITY; w * h];
    let mut open = BinaryHeap::new();
    for (i, distance) in field.iter_mut().enumerate() {
        let (c, r) = (i % w, i / w);
        let edge = (c + 1).min(r + 1).min(w - c).min(h - r) as f64;
        *distance = if blocked(map.occupancy[i]) { 0. } else { edge };
        open.push(Open(*distance, i));
    }
    while let Some(Open(d, i)) = open.pop() {
        if d > field[i] {
            continue;
        }
        for (j, step) in neighbours(map, i) {
            if d + step < field[j] {
                field[j] = d + step;
                open.push(Open(d + step, j));
            }
        }
    }
    field
}

fn neighbours(map: &MapSnapshot, i: usize) -> impl Iterator<Item = (usize, f64)> {
    let (w, h) = (map.width as i64, map.height as i64);
    let (c, r) = ((i as i64) % w, (i as i64) / w);
    [
        (-1, -1),
        (0, -1),
        (1, -1),
        (-1, 0),
        (1, 0),
        (-1, 1),
        (0, 1),
        (1, 1),
    ]
    .into_iter()
    .filter_map(move |(dc, dr): (i64, i64)| {
        let (nc, nr) = (c + dc, r + dr);
        (nc >= 0 && nr >= 0 && nc < w && nr < h).then(|| {
            (
                (nr * w + nc) as usize,
                if dc != 0 && dr != 0 { SQRT_2 } else { 1. },
            )
        })
    })
}

/// Min-heap entry ordered by cost.
#[derive(PartialEq)]
struct Open(f64, usize);
impl Eq for Open {}
impl Ord for Open {
    fn cmp(&self, other: &Self) -> Ordering {
        other.0.total_cmp(&self.0).then(other.1.cmp(&self.1))
    }
}
impl PartialOrd for Open {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Shortest forward Dubins path from `from` to `to` with turn radius `radius`,
/// as (curvature, length) arcs. Every candidate word is integrated and kept only
/// if it really lands on the goal pose.
fn dubins(from: (f64, f64, f64), to: (f64, f64, f64), radius: f64) -> Option<Vec<(f64, f64)>> {
    let m = |v: f64| v.rem_euclid(TAU);
    let (dx, dy) = (to.0 - from.0, to.1 - from.1);
    let d = dx.hypot(dy) / radius;
    let theta = m(dy.atan2(dx));
    let (a, b) = (m(from.2 - theta), m(to.2 - theta));
    let (sa, sb, ca, cb) = (a.sin(), b.sin(), a.cos(), b.cos());
    let cab = (a - b).cos();
    let (l, s, r) = (1. / radius, 0., -1. / radius);
    let mut words: Vec<[(f64, f64); 3]> = Vec::new();
    // LSL
    let p2 = 2. + d * d - 2. * cab + 2. * d * (sa - sb);
    if p2 >= 0. {
        let t = (cb - ca).atan2(d + sa - sb);
        words.push([(l, m(-a + t)), (s, p2.sqrt()), (l, m(b - t))]);
    }
    // RSR
    let p2 = 2. + d * d - 2. * cab + 2. * d * (sb - sa);
    if p2 >= 0. {
        let t = (ca - cb).atan2(d - sa + sb);
        words.push([(r, m(a - t)), (s, p2.sqrt()), (r, m(-b + t))]);
    }
    // LSR
    let p2 = -2. + d * d + 2. * cab + 2. * d * (sa + sb);
    if p2 >= 0. {
        let p = p2.sqrt();
        let t = (-ca - cb).atan2(d + sa + sb) - (-2f64).atan2(p);
        words.push([(l, m(-a + t)), (s, p), (r, m(-b + t))]);
    }
    // RSL
    let p2 = -2. + d * d + 2. * cab - 2. * d * (sa + sb);
    if p2 >= 0. {
        let p = p2.sqrt();
        let t = (ca + cb).atan2(d - sa - sb) - 2f64.atan2(p);
        words.push([(r, m(a - t)), (s, p), (l, m(b - t))]);
    }
    // RLR
    let k = (6. - d * d + 2. * cab + 2. * d * (sa - sb)) / 8.;
    if k.abs() <= 1. {
        let p = m(TAU - k.acos());
        let t = m(a - (ca - cb).atan2(d - sa + sb) + p / 2.);
        words.push([(r, t), (l, p), (r, m(a - b - t + p))]);
    }
    // LRL
    let k = (6. - d * d + 2. * cab + 2. * d * (sb - sa)) / 8.;
    if k.abs() <= 1. {
        let p = m(TAU - k.acos());
        let t = m(-a - (ca - cb).atan2(d + sa - sb) + p / 2.);
        words.push([(l, t), (r, p), (l, m(b - a - t + p))]);
    }
    words
        .into_iter()
        .map(|word| word.map(|(curvature, units)| (curvature, units * radius)))
        .filter(|word| {
            let end = word.iter().fold(from, |at, &(c, len)| drive(at, c, len));
            (end.0 - to.0).hypot(end.1 - to.1) < 1e-6 && wrap(end.2 - to.2).abs() < 1e-6
        })
        .min_by(|x, y| {
            let total = |w: &[(f64, f64); 3]| w.iter().map(|(_, len)| len).sum::<f64>();
            total(x).total_cmp(&total(y))
        })
        .map(|word| word.into_iter().filter(|(_, len)| *len > 1e-9).collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dubins_lands_on_the_goal_and_goes_straight_when_aligned() {
        let path = dubins((0., 0., 0.), (3., 0., 0.), 0.5).unwrap();
        let total: f64 = path.iter().map(|(_, len)| len).sum();
        assert!((total - 3.).abs() < 1e-9, "{path:?}");
        for (x, y, yaw) in [
            (2., 1., PI / 2.),
            (-1., 2., PI),
            (0.3, -0.2, -2.),
            (4., -3., 1.),
        ] {
            let path =
                dubins((0.5, -0.5, 0.7), (x, y, yaw), 0.5).expect("some word reaches any pose");
            assert!(path.iter().all(|(c, _)| c.abs() <= 2. + 1e-9));
        }
    }

    #[test]
    fn drive_matches_a_quarter_circle() {
        let (x, y, yaw) = drive((0., 0., 0.), 2., PI / 4.);
        assert!(
            (x - 0.5).abs() < 1e-12 && (y - 0.5).abs() < 1e-12 && (yaw - PI / 2.).abs() < 1e-12
        );
    }
}
