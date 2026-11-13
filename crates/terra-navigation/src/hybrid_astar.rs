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
//! obstacle-aware 2D distance, both scaled so they never overestimate. Near the
//! goal, a direct curve to it is tried as a shortcut: a Dubins curve with a goal
//! heading, otherwise one turn and a straight line.
use crate::footprint::{distance_field, for_each_overlapping_cell};
use crate::{Footprint, Pose};
use serde::{Deserialize, Serialize};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::f64::consts::{PI, SQRT_2, TAU};
use terra_mapping::MapSnapshot;
use terra_mapping::OCCUPIED;

/// Longest 8-connected grid distance relative to the straight-line distance.
const OCTILE_OVER_EUCLID: f64 = 1.082_392_2;
/// Half a cell diagonal, in cells.
const HALF_DIAGONAL: f64 = std::f64::consts::FRAC_1_SQRT_2;
/// Largest map the search accepts.
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
    /// Longest stretch driven backwards in one go, metres. Reversing is for
    /// short manoeuvres (backing out, three-point turns), not for travel: the
    /// camera faces forward. A route also always arrives driving forward.
    pub max_reverse_length: f64,
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
    /// Heuristic weight, at least 1. Above 1 the search expands far fewer
    /// states, and a route is at most this factor longer than the best one.
    pub heuristic_weight: f64,
    /// Clearance from observed obstacles the route prefers, metres. Closer
    /// than this costs extra, so routes hug obstacles only when they must.
    pub preferred_clearance: f64,
    /// Extra cost per metre at zero clearance (scaled down linearly to none at
    /// `preferred_clearance`).
    pub clearance_penalty: f64,
}

impl Default for HybridConfig {
    /// The Terra profile body (0.61 m square) and the default skid-safe turn radius.
    fn default() -> Self {
        Self {
            footprint: Footprint::default(),
            min_turn_radius: 0.5,
            allow_reverse: true,
            max_reverse_length: 1.0,
            reverse_penalty: 2.0,
            switch_penalty: 1.0,
            steer_penalty: 0.2,
            steer_change_penalty: 0.1,
            heading_bins: 72,
            goal_tolerance: 0.15,
            goal_heading_tolerance: 0.1,
            max_expansions: 200_000,
            heuristic_weight: 1.2,
            preferred_clearance: 0.3,
            clearance_penalty: 2.0,
        }
    }
}

impl HybridConfig {
    /// The planner's footprint and the turn radius from the turning policy.
    pub fn for_body(footprint: Footprint, turning: &terra_types::turning::TurningConfig) -> Self {
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
            && self.max_reverse_length.is_finite()
            && (0.0..=20.).contains(&self.max_reverse_length)
            && self.reverse_penalty.is_finite()
            && self.reverse_penalty >= 1.
            && (8..=360).contains(&self.heading_bins)
            && self.preferred_clearance.is_finite()
            && (0.0..=5.).contains(&self.preferred_clearance)
            && self.clearance_penalty.is_finite()
            && self.clearance_penalty >= 0.
            && self.heuristic_weight.is_finite()
            && (1.0..=5.).contains(&self.heuristic_weight)
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
/// Unknown cells are blocked except under the starting body. This runs the whole
/// search at once; use [`crate::RouteJob`] to spread it over control ticks.
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
    let mut search = HybridSearch::new(map, start, goal, config, None)?;
    loop {
        if let Some(result) = search.advance(u32::MAX) {
            return result;
        }
    }
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
    /// Distance driven backwards in one go, ending at this node.
    reversed: f64,
}

/// A Hybrid A* search that can be advanced a bounded number of expansions at a
/// time, so a control loop can spread it over several ticks. It keeps its own
/// copy of the map, so later map updates do not affect it.
pub struct HybridSearch {
    map: MapSnapshot,
    config: HybridConfig,
    goal: HybridGoal,
    /// `Some(k)`: unknown cells are passable, at `k` times the cost.
    unknown_cost: Option<f64>,
    /// Unknown cells under the starting body, which the rover may drive off.
    tolerated: HashSet<usize>,
    /// Lower-bound distance (cells, 8-connected) to the nearest blocked or unknown cell.
    blocked_distance: Vec<f64>,
    /// 8-connected distance (cells) to the nearest observed obstacle or the map edge.
    obstacle_distance: Vec<f64>,
    /// Admissible distance to the goal from every cell, built incrementally.
    heuristic: Vec<f64>,
    /// Cells the inscribed circle could occupy, for the heuristic.
    passable: Vec<bool>,
    /// Pending heuristic cells; empty once the field is complete.
    heuristic_open: BinaryHeap<Open>,
    /// Cells where the body is certain to overlap unknown space (only when
    /// unknown cells are allowed at a cost): the heuristic charges them that cost.
    surely_unknown: Vec<bool>,
    start_cell: usize,
    kappa: f64,
    step: f64,
    nodes: Vec<Node>,
    best: HashMap<usize, f64>,
    closed: HashSet<usize>,
    open: BinaryHeap<Open>,
    expansions: u32,
    finished: bool,
}

impl HybridSearch {
    /// Validate the request and prepare the search. `unknown_cost`, when set
    /// (at least 1), lets the route cross unknown cells at that cost multiple;
    /// otherwise unknown cells block, except under the starting body.
    ///
    /// # Errors
    /// The same as [`hybrid_astar_route`], found before any expansion where possible.
    pub fn new(
        map: &MapSnapshot,
        start: Pose,
        goal: HybridGoal,
        config: &HybridConfig,
        unknown_cost: Option<f64>,
    ) -> Result<Self, &'static str> {
        if !map.valid()
            || map.occupancy.len() > MAX_CELLS
            || !config.valid()
            || !start.finite()
            || !goal.x.is_finite()
            || !goal.y.is_finite()
            || goal.yaw.is_some_and(|yaw| !yaw.is_finite())
            || unknown_cost.is_some_and(|k| !k.is_finite() || k < 1.)
        {
            return Err("route_invalid_map");
        }
        let start_cell = map
            .cell(start.x, start.y)
            .ok_or("route_start_outside_map")?;
        let goal_cell = map.cell(goal.x, goal.y).ok_or("route_goal_outside_map")?;
        let mut search = Self {
            map: map.clone(),
            config: config.clone(),
            goal,
            unknown_cost,
            tolerated: HashSet::new(),
            blocked_distance: distance_field(map, |_, v| !(0..OCCUPIED).contains(&v)),
            obstacle_distance: distance_field(map, |_, v| v >= OCCUPIED),
            heuristic: vec![f64::INFINITY; map.occupancy.len()],
            passable: Vec::new(),
            heuristic_open: BinaryHeap::new(),
            surely_unknown: Vec::new(),
            start_cell: index(map, start_cell),
            kappa: 1. / config.min_turn_radius,
            // Long enough to leave the current cell from any heading.
            step: map.resolution * 1.6,
            nodes: Vec::new(),
            best: HashMap::new(),
            closed: HashSet::new(),
            open: BinaryHeap::new(),
            expansions: 0,
            finished: false,
        };
        let (a, b) = search.half_extents(0.);
        let tolerated = &mut search.tolerated;
        for_each_overlapping_cell(map, start.x, start.y, start.yaw, a, b, |i| {
            if let Some(i) = i
                && map.occupancy[i] < 0
            {
                tolerated.insert(i);
            }
            true
        });
        // The camera's blind zone around the start (see Footprint::blind_zone_radius).
        let reach = (config.footprint.blind_zone_radius() / map.resolution).ceil() as i64 + 1;
        if let Some((c0, r0)) = map.cell(start.x, start.y) {
            for r in (r0 as i64 - reach).max(0)..=(r0 as i64 + reach).min(map.height as i64 - 1) {
                for c in (c0 as i64 - reach).max(0)..=(c0 as i64 + reach).min(map.width as i64 - 1)
                {
                    let i = r as usize * map.width as usize + c as usize;
                    if map.occupancy[i] < 0
                        && crate::footprint::in_blind_zone(map, &config.footprint, i, start)
                    {
                        tolerated.insert(i);
                    }
                }
            }
        }
        if search.body(start.x, start.y, start.yaw, 0.).is_none() {
            return Err("route_start_blocked");
        }
        if let Some(yaw) = goal.yaw
            && search.body(goal.x, goal.y, yaw, 0.).is_none()
        {
            return Err("route_goal_blocked");
        }
        search.passable = search.passable_cells();
        if search.unknown_cost.is_some() {
            // The body contains its inscribed circle, so a cell whose centre is
            // within that radius (less a cell's half diagonal) of an unknown
            // cell is a pose where the body overlaps unknown space.
            let inscribed = search.config.footprint.inradius() / map.resolution - HALF_DIAGONAL;
            let tolerated = &search.tolerated;
            let to_unknown = distance_field(map, |_, v| v < 0);
            search.surely_unknown = to_unknown
                .iter()
                .enumerate()
                .map(|(i, d)| *d <= inscribed && !tolerated.contains(&i))
                .collect();
        }
        let goal_index = index(map, goal_cell);
        if !search.passable[goal_index] {
            return Err("route_goal_blocked");
        }
        search.heuristic[goal_index] = 0.;
        search.heuristic_open.push(Open(0., goal_index));
        search.nodes.push(Node {
            x: start.x,
            y: start.y,
            yaw: start.yaw,
            g: 0.,
            parent: None,
            curvature: 0.,
            length: 0.,
            reversed: 0.,
        });
        Ok(search)
    }

    /// Run up to `budget` expansions. Returns `None` while still searching, then
    /// the route or the reason there is none (`route_no_path`, `route_search_limit`).
    pub fn advance(&mut self, budget: u32) -> Option<Result<Vec<HybridWaypoint>, &'static str>> {
        if self.finished {
            return Some(Err("route_no_path"));
        }
        if !self.heuristic_open.is_empty() || self.open.is_empty() && self.nodes.len() == 1 {
            // Building the heuristic: a cell costs a few hundredths of an
            // expansion, so a 200 x 200 map's field usually fits in one call.
            if !self.extend_heuristic(budget.saturating_mul(300)) {
                return None;
            }
            if !self.heuristic[self.start_cell].is_finite() {
                self.finished = true;
                return Some(Err("route_no_path"));
            }
            let h = self.h(self.start_x(), self.start_y());
            self.open.push(Open(self.config.heuristic_weight * h, 0));
        }
        let curvatures = [-1., -0.5, 0., 0.5, 1.].map(|k| k * self.kappa);
        let directions: &[f64] = if self.config.allow_reverse {
            &[1., -1.]
        } else {
            &[1.]
        };
        let mut used = 0u32;
        while let Some(Open(_, id)) = self.open.pop() {
            let node = self.nodes[id];
            let Some(key) = self.bin(node.x, node.y, node.yaw, node.length < 0.) else {
                continue;
            };
            if !self.closed.insert(key) {
                continue;
            }
            // Arrive driving forward (or not having moved at all).
            if node.length >= 0. && self.arrived(node.x, node.y, node.yaw) {
                self.finished = true;
                return Some(Ok(self.waypoints(id, None)));
            }
            self.expansions += 1;
            if self.expansions > self.config.max_expansions {
                self.finished = true;
                return Some(Err("route_search_limit"));
            }
            if let Some(shortcut) = self.dubins_shortcut(&node) {
                self.finished = true;
                return Some(Ok(self.waypoints(id, Some(shortcut))));
            }
            for &direction in directions {
                for &curvature in &curvatures {
                    let length = direction * self.step;
                    let from = (node.x, node.y, node.yaw);
                    let (x, y, yaw) = drive(from, curvature, length);
                    let reversed = if length < 0. {
                        length.abs() + if node.length < 0. { node.reversed } else { 0. }
                    } else {
                        0.
                    };
                    if reversed > self.config.max_reverse_length + 1e-9 {
                        continue;
                    }
                    let Some(child_key) = self.bin(x, y, yaw, length < 0.) else {
                        continue;
                    };
                    if self.closed.contains(&child_key) {
                        continue;
                    }
                    let mut g =
                        node.g + self.cost(&node, curvature, length) + self.closeness(x, y, length);
                    if self.best.get(&child_key).is_some_and(|&b| b <= g) {
                        continue;
                    }
                    let Some(unknown) = self.arc(from, curvature, length) else {
                        continue;
                    };
                    if unknown {
                        g = node.g
                            + self.cost(&node, curvature, length) * self.unknown_cost.unwrap_or(1.)
                            + self.closeness(x, y, length);
                    }
                    let h = self.h(x, y);
                    if !h.is_finite() {
                        continue;
                    }
                    self.best.insert(child_key, g);
                    self.nodes.push(Node {
                        x,
                        y,
                        yaw,
                        g,
                        parent: Some(id),
                        curvature,
                        length,
                        reversed,
                    });
                    self.open.push(Open(
                        g + self.config.heuristic_weight * h,
                        self.nodes.len() - 1,
                    ));
                }
            }
            used += 1;
            if used >= budget {
                return None;
            }
        }
        self.finished = true;
        Some(Err("route_no_path"))
    }

    fn half_extents(&self, pad: f64) -> (f64, f64) {
        self.config.footprint.half_extents(pad)
    }

    /// The body, grown by `pad`, at this pose: `None` if it is blocked, otherwise
    /// whether it overlaps unknown cells the search was told it may cross.
    fn body(&self, x: f64, y: f64, yaw: f64, pad: f64) -> Option<bool> {
        let (a, b) = self.half_extents(pad);
        let c = self.map.cell(x, y)?;
        // Far from everything blocked or unknown: no need to look at cells.
        let lower_bound = (self.blocked_distance[index(&self.map, c)] / OCTILE_OVER_EUCLID
            - 2. * HALF_DIAGONAL)
            * self.map.resolution;
        if lower_bound > a.hypot(b) {
            return Some(false);
        }
        let mut unknown = false;
        let clear = for_each_overlapping_cell(&self.map, x, y, yaw, a, b, |i| match i {
            None => false,
            Some(i) => {
                let v = self.map.occupancy[i];
                if (0..OCCUPIED).contains(&v) || v < 0 && self.tolerated.contains(&i) {
                    true
                } else if v < 0 && self.unknown_cost.is_some() {
                    unknown = true;
                    true
                } else {
                    false
                }
            }
        });
        clear.then_some(unknown)
    }

    /// The body along an arc of signed `length` from `from`: `None` if it hits
    /// something, otherwise whether it crosses unknown cells.
    fn arc(&self, from: (f64, f64, f64), curvature: f64, length: f64) -> Option<bool> {
        // Sample so no body point moves more than half a cell between checks, then
        // grow the body by a quarter cell: the swept area is then fully covered.
        let (a, b) = self.half_extents(0.);
        let reach = 1. + a.hypot(b) * curvature.abs();
        let spacing = (self.map.resolution / 2.) / reach;
        let samples = (length.abs() / spacing).ceil().max(1.) as usize;
        let mut unknown = false;
        for k in 1..=samples {
            let (x, y, yaw) = drive(from, curvature, length * k as f64 / samples as f64);
            unknown |= self.body(x, y, yaw, self.map.resolution / 4.)?;
        }
        Some(unknown)
    }

    /// Cells the body's inscribed circle could occupy: the heuristic searches
    /// only these, so it never overestimates the remaining distance.
    fn passable_cells(&self) -> Vec<bool> {
        let (_, inscribed) = self.half_extents(0.);
        let res = self.map.resolution;
        let unknown_blocks = self.unknown_cost.is_none();
        distance_field(&self.map, |_, v| v >= OCCUPIED || unknown_blocks && v < 0)
            .iter()
            .enumerate()
            .map(|(i, d)| (d + HALF_DIAGONAL) * res >= inscribed || self.tolerated.contains(&i))
            .collect()
    }

    /// Grow the admissible 2D distance-to-goal field by up to `cells` cells.
    /// Returns true once it is complete. Unreached cells stay infinite.
    fn extend_heuristic(&mut self, cells: u32) -> bool {
        let res = self.map.resolution;
        for _ in 0..cells {
            let Some(Open(d, i)) = self.heuristic_open.pop() else {
                return true;
            };
            if d > self.heuristic[i] {
                continue;
            }
            for (j, step) in neighbours(&self.map, i) {
                // Where the body surely overlaps unknown space, a step really
                // costs the unknown multiple: still admissible.
                let unknown = self.surely_unknown.get(j).copied().unwrap_or(false);
                let factor = if unknown {
                    self.unknown_cost.unwrap_or(1.)
                } else {
                    1.
                };
                let next = d + step * res / OCTILE_OVER_EUCLID * factor;
                if self.passable[j] && next < self.heuristic[j] {
                    self.heuristic[j] = next;
                    self.heuristic_open.push(Open(next, j));
                }
            }
        }
        self.heuristic_open.is_empty()
    }

    fn start_x(&self) -> f64 {
        self.nodes[0].x
    }

    fn start_y(&self) -> f64 {
        self.nodes[0].y
    }

    fn h(&self, x: f64, y: f64) -> f64 {
        let Some(c) = self.map.cell(x, y) else {
            return f64::INFINITY;
        };
        let straight = (self.goal.x - x).hypot(self.goal.y - y);
        let grid = self.heuristic[index(&self.map, c)] - 2. * HALF_DIAGONAL * self.map.resolution;
        straight.max(grid)
    }

    /// State bin: map cell, heading, and whether the state was reached in
    /// reverse (a reversed state may not reverse further, so the two differ).
    fn bin(&self, x: f64, y: f64, yaw: f64, reverse: bool) -> Option<usize> {
        let c = self.map.cell(x, y)?;
        let bins = self.config.heading_bins as usize;
        let heading = ((yaw.rem_euclid(TAU) / TAU) * bins as f64) as usize % bins;
        Some((index(&self.map, c) * bins + heading) * 2 + usize::from(reverse))
    }

    fn arrived(&self, x: f64, y: f64, yaw: f64) -> bool {
        (self.goal.x - x).hypot(self.goal.y - y) <= self.config.goal_tolerance
            && self.goal.yaw.is_none_or(|g| {
                terra_types::wrap_angle(yaw - g).abs() <= self.config.goal_heading_tolerance
            })
    }

    fn cost(&self, parent: &Node, curvature: f64, length: f64) -> f64 {
        let c = &self.config;
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

    /// Extra cost for an arc of `length` ending at `(x, y)` closer to observed
    /// obstacles than the preferred clearance. Uses the circle around the body,
    /// so it never undercharges a pose that is actually close.
    fn closeness(&self, x: f64, y: f64, length: f64) -> f64 {
        let c = &self.config;
        if c.clearance_penalty == 0. || c.preferred_clearance == 0. {
            return 0.;
        }
        let Some(cell_xy) = self.map.cell(x, y) else {
            return 0.;
        };
        let centre = self.obstacle_distance[index(&self.map, cell_xy)] / OCTILE_OVER_EUCLID
            * self.map.resolution;
        let clearance = (centre - c.footprint.circumradius()).max(0.);
        if clearance >= c.preferred_clearance {
            return 0.;
        }
        length.abs() * c.clearance_penalty * (1. - clearance / c.preferred_clearance)
    }

    /// Try a collision-free forward curve from `node` straight onto the goal:
    /// a Dubins curve when the goal has a heading, otherwise one turn toward
    /// the goal and then a straight line. Without this, the search can only
    /// line up with the goal by chaining its fixed arcs, which zig-zags (an S
    /// into the goal) because headings come in fixed steps.
    fn dubins_shortcut(&self, node: &Node) -> Option<Vec<(f64, f64)>> {
        let near =
            (self.goal.x - node.x).hypot(self.goal.y - node.y) <= 10. * self.config.min_turn_radius;
        if !near && !self.expansions.is_multiple_of(10) {
            return None;
        }
        let from = (node.x, node.y, node.yaw);
        let radius = self.config.min_turn_radius;
        let Some(yaw) = self.goal.yaw else {
            // Prefer a gentler turn when it fits: easier to track, and faster
            // under the yaw-rate cap.
            return [2. * radius, radius].into_iter().find_map(|r| {
                let path = turn_then_straight(from, (self.goal.x, self.goal.y), r)?;
                // Through observed space only: crossing unknown costs extra in
                // the search, and a shortcut must not skip that.
                self.path_clear(from, &path, false).then_some(path)
            });
        };
        let path = dubins(from, (self.goal.x, self.goal.y, yaw), radius)?;
        self.path_clear(from, &path, true).then_some(path)
    }

    /// Whether the body fits along these arcs from `from`; crossing unknown
    /// cells the search may cross counts as clear only if `unknown_ok`.
    fn path_clear(&self, from: (f64, f64, f64), path: &[(f64, f64)], unknown_ok: bool) -> bool {
        let mut at = from;
        for &(curvature, length) in path {
            match self.arc(at, curvature, length) {
                Some(unknown) if unknown_ok || !unknown => {}
                _ => return false,
            }
            at = drive(at, curvature, length);
        }
        true
    }

    /// Route samples from the start to node `last`, then along `shortcut`.
    fn waypoints(&self, last: usize, shortcut: Option<Vec<(f64, f64)>>) -> Vec<HybridWaypoint> {
        let nodes = &self.nodes;
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
                    yaw: terra_types::wrap_angle(yaw),
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

fn index(map: &MapSnapshot, (column, row): (usize, usize)) -> usize {
    row * map.width as usize + column
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

/// Shortest forward path from `from` to the point `to` that turns at `radius`
/// (left or right) and then drives straight, as (curvature, length) arcs.
/// `None` when the point is inside both turning circles.
fn turn_then_straight(
    from: (f64, f64, f64),
    to: (f64, f64),
    radius: f64,
) -> Option<Vec<(f64, f64)>> {
    let (x, y, yaw) = from;
    let (sin, cos) = yaw.sin_cos();
    [1f64, -1.]
        .into_iter()
        .filter_map(|side| {
            // Turning centre to the left (side 1) or right (side -1).
            let (cx, cy) = (x - side * radius * sin, y + side * radius * cos);
            let (vx, vy) = (to.0 - cx, to.1 - cy);
            let d = vx.hypot(vy);
            if d < radius {
                return None;
            }
            // Angle (from the centre) of the point where the straight leg leaves the circle.
            let beta = vy.atan2(vx);
            let leave = beta - side * (radius / d).acos();
            let start = yaw - side * PI / 2.;
            let sweep = (side * (leave - start)).rem_euclid(TAU);
            let sweep = if TAU - sweep < 1e-9 { 0. } else { sweep };
            let straight = (d * d - radius * radius).max(0.).sqrt();
            Some((
                sweep * radius + straight,
                [(side / radius, sweep * radius), (0., straight)],
            ))
        })
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .map(|(_, arcs)| arcs.into_iter().filter(|(_, len)| *len > 1e-9).collect())
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
            (end.0 - to.0).hypot(end.1 - to.1) < 1e-6
                && terra_types::wrap_angle(end.2 - to.2).abs() < 1e-6
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
    fn turn_then_straight_lands_on_the_point() {
        for (x, y) in [
            (3., 0.),
            (2., 1.5),
            (-1., 2.),
            (0.5, -2.),
            (-3., -0.1),
            (4., 0.3),
        ] {
            for r in [0.5, 1.] {
                let from = (0.2, -0.1, 0.4);
                let path = turn_then_straight(from, (x, y), r).expect("outside the circles");
                assert!(path.len() <= 2, "{path:?}");
                let end = path.iter().fold(from, |at, &(c, len)| drive(at, c, len));
                assert!((end.0 - x).hypot(end.1 - y) < 1e-6, "{x} {y} {r}: {end:?}");
                assert!(
                    path.iter()
                        .all(|(c, len)| c.abs() <= 1. / r + 1e-9 && *len > 0.)
                );
            }
        }
        // Straight ahead needs no turn at all.
        let path = turn_then_straight((0., 0., 0.), (3., 0.), 0.5).unwrap();
        assert!(path.len() == 1 && path[0].0 == 0. && (path[0].1 - 3.).abs() < 1e-9);
        // Just beside the rover, inside the left circle: it loops round to the right.
        let path = turn_then_straight((0., 0., 0.), (0., 0.3), 0.5).unwrap();
        assert!(path[0].0 < 0., "{path:?}");
    }

    #[test]
    fn chamfer_distance_matches_dijkstra() {
        let (w, h) = (37usize, 23usize);
        let occupancy: Vec<i8> = (0..w * h)
            .map(|i| {
                if (i * 7919 + i / 13) % 17 == 0 {
                    100
                } else {
                    0
                }
            })
            .collect();
        let map = MapSnapshot {
            width: w as u32,
            height: h as u32,
            resolution: 0.1,
            origin_x: 0.,
            origin_y: 0.,
            occupancy,
        };
        let fast = distance_field(&map, |_, v| v >= OCCUPIED);
        // Reference: Dijkstra from every blocked cell and the map edge.
        let mut slow: Vec<f64> = (0..w * h)
            .map(|i| {
                let (c, r) = (i % w, i / w);
                if map.occupancy[i] >= OCCUPIED {
                    0.
                } else {
                    (c + 1).min(r + 1).min(w - c).min(h - r) as f64
                }
            })
            .collect();
        let mut open: BinaryHeap<Open> = (0..w * h).map(|i| Open(slow[i], i)).collect();
        while let Some(Open(d, i)) = open.pop() {
            if d > slow[i] {
                continue;
            }
            for (j, step) in neighbours(&map, i) {
                if d + step < slow[j] {
                    slow[j] = d + step;
                    open.push(Open(d + step, j));
                }
            }
        }
        for i in 0..w * h {
            assert!(
                (fast[i] - slow[i]).abs() < 1e-9,
                "cell {i}: {} vs {}",
                fast[i],
                slow[i]
            );
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
