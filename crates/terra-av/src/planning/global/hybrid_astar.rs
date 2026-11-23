//! Hybrid A*: A* over (x, y, yaw) that expands drivable arcs from the
//! vehicle model, so the route respects the minimum turning radius.
//! (Dolgov, Thrun, Montemerlo & Diebel, 2008.)
//!
//! Plain A* moves from grid cell to grid cell, which a rover cannot follow
//! exactly. Here every search node is a real pose, and its successors are
//! the poses reached by driving one short arc from it (straight, or curving
//! left or right). The grid is only used to say "this pose is about the same
//! as one I already reached more cheaply": two poses in the same
//! `resolution` × `resolution` cell and the same yaw bin count as one.
//!
//! - **Cost so far (g):** metres driven.
//! - **Estimate to go (h):** straight-line distance to the goal, times
//!   `heuristic_weight`. With weight 1 the route is the shortest the arcs
//!   allow; a larger weight searches less and may give a longer route.
//! - **Obstacles:** every map obstacle blocks the grid cells within the
//!   rover's radius (plus [`ROUTE_MARGIN_M`]) of it. An arc is allowed if
//!   it ends in an unblocked cell. Arcs are shorter than a blocked region is wide (`step` should
//!   stay below the rover's diameter), so an arc cannot jump over one.
//! - **Done** when a node is within one `step` of the goal: the goal pose
//!   is appended as the last pose.
//!
//! Limits, so one plan cannot stall the control tick: at most
//! [`MAX_EXPANSIONS`] nodes; past that there is "no route". When
//! `allow_reverse` is enabled, reverse arcs are also expanded with a cost
//! multiplier (`reverse_penalty`) and a direction-change penalty so the
//! planner prefers driving forward and backs out only when needed.
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use super::PathPlanner;
use crate::geometry::{Pose2, Vec2, wrap_angle};
use crate::planning::registry::{
    PlannerSpec, bool_param, boolean, float, float_param, int, int_param,
};
use crate::planning::{Goal, Path};
use crate::{
    estimation::State,
    mapping::Map,
    vehicle::{Twist, VehicleModel},
};

/// The route stays this much further from obstacles (m) than the rover's
/// radius. The controller that follows the route cuts corners a little and
/// refuses to touch an obstacle, so a route that only just fits would
/// leave it no way through.
const ROUTE_MARGIN_M: f64 = 0.15;
/// Give up after expanding this many nodes.
const MAX_EXPANSIONS: usize = 20_000;
/// Extra cost (equivalent metres) when switching between forward and reverse,
/// so the route does not zig-zag unnecessarily.
const DIRECTION_CHANGE_PENALTY_M: f64 = 0.5;
/// The arcs tried from every node, as turning radii in rover radii:
/// straight, a wide turn and a tight turn, each to the left and right.
/// A radius of 1 pivots about the rover's own edge.
const TURN_RADII_IN_ROVER_RADII: [f64; 2] = [2.0, 1.0];

pub struct HybridAStar {
    /// Grid cell size for visited-state pruning (m).
    pub resolution: f64,
    /// Number of yaw bins.
    pub yaw_bins: usize,
    /// Length of one expanded arc (m).
    pub step: f64,
    /// >1 trades optimality for speed.
    pub heuristic_weight: f64,
    /// Expand reverse arcs so the rover can back out of tight spots.
    pub allow_reverse: bool,
    /// Cost multiplier for reverse arcs (≥ 1; higher prefers forward).
    pub reverse_penalty: f64,
}

/// The grid cell a position falls in.
type Cell = (i32, i32);
/// A cell plus a yaw bin: poses with the same key count as the same state.
type Key = (i32, i32, usize);

/// A pose the search has reached.
struct Node {
    pose: Pose2,
    /// Metres driven from the start to here (including reverse/switch penalties).
    cost: f64,
    /// Direction of the arc that reached this node: `1` forward, `-1` reverse, `0` start.
    dir: i8,
    /// Index of the node this one was reached from; `None` for the start.
    parent: Option<usize>,
}

/// An entry of the open list: node `index`, ordered so that the entry with
/// the smallest `estimate` (cost so far + estimate to go) is popped first.
struct Open {
    estimate: f64,
    index: usize,
}

impl Ord for Open {
    fn cmp(&self, other: &Self) -> Ordering {
        // `BinaryHeap` pops the largest entry, so compare the other way round.
        other.estimate.total_cmp(&self.estimate)
    }
}
impl PartialOrd for Open {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl PartialEq for Open {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}
impl Eq for Open {}

impl HybridAStar {
    pub fn spec() -> PlannerSpec {
        PlannerSpec {
            id: "hybrid_astar",
            version: "2",
            description: "A* over drivable forward and reverse arcs, around the mapped obstacles",
            params: vec![
                float_param("resolution", 0.1, 0.02, 1.0, "m", "Grid cell size"),
                int_param("yaw_bins", 72, 8, 360, "Headings told apart per cell"),
                float_param("step", 0.3, 0.05, 2.0, "m", "Length of one arc; keep below the rover's diameter"),
                float_param("heuristic_weight", 1.5, 1.0, 5.0, "", "1 = shortest route; larger = faster search"),
                bool_param("allow_reverse", true, "Allow backing out of tight spots when forward is blocked"),
                float_param("reverse_penalty", 2.0, 1.0, 10.0, "", "Cost multiplier for reverse arcs (higher prefers forward)"),
            ],
            make: |params| {
                Box::new(HybridAStar {
                    resolution: float(params, "resolution"),
                    yaw_bins: int(params, "yaw_bins") as usize,
                    step: float(params, "step"),
                    heuristic_weight: float(params, "heuristic_weight"),
                    allow_reverse: boolean(params, "allow_reverse"),
                    reverse_penalty: float(params, "reverse_penalty"),
                })
            },
        }
    }

    fn cell(&self, position: Vec2) -> Cell {
        (
            (position.x / self.resolution).floor() as i32,
            (position.y / self.resolution).floor() as i32,
        )
    }

    fn cell_centre(&self, (ix, iy): Cell) -> Vec2 {
        Vec2 {
            x: (f64::from(ix) + 0.5) * self.resolution,
            y: (f64::from(iy) + 0.5) * self.resolution,
        }
    }

    fn key(&self, pose: &Pose2) -> Key {
        use std::f64::consts::PI;
        let (ix, iy) = self.cell(pose.position());
        // Yaw in (−π, π] → a fraction of a full turn in (0, 1] → a bin.
        let turn_fraction = (wrap_angle(pose.yaw) + PI) / (2.0 * PI);
        let bin = (turn_fraction * self.yaw_bins as f64) as usize % self.yaw_bins;
        (ix, iy, bin)
    }

    /// The cells the rover's centre must stay out of: every cell whose
    /// Spatial index mapping each grid cell within `reach_m` of any obstacle
    /// to the list of obstacles within that radius. Cells absent from the map
    /// are guaranteed collision-free in O(1) without trig; cells present get an
    /// exact oriented-rectangle clearance check.
    fn nearby_obstacles(&self, obstacles: &[Vec2], reach_m: f64) -> HashMap<Cell, Vec<Vec2>> {
        let reach = (reach_m / self.resolution).ceil() as i32;
        let mut map: HashMap<Cell, Vec<Vec2>> = HashMap::new();
        for &obstacle in obstacles {
            let (ox, oy) = self.cell(obstacle);
            for ix in ox - reach..=ox + reach {
                for iy in oy - reach..=oy + reach {
                    if self.cell_centre((ix, iy)).distance(obstacle) <= reach_m {
                        map.entry((ix, iy)).or_default().push(obstacle);
                    }
                }
            }
        }
        map
    }

    /// True if the rover's oriented rectangle at `pose` comes within `margin`
    /// metres of any obstacle.
    fn pose_blocked(
        &self,
        pose: Pose2,
        footprint: crate::vehicle::Footprint,
        margin: f64,
        nearby: &HashMap<Cell, Vec<Vec2>>,
    ) -> bool {
        let Some(obstacles) = nearby.get(&self.cell(pose.position())) else {
            return false;
        };
        obstacles.iter().any(|&o| footprint.clearance(pose, o) <= margin)
    }
}

impl PathPlanner for HybridAStar {
    fn plan(
        &mut self,
        state: &State,
        map: &Map,
        goal: &Goal,
        vehicle: &dyn VehicleModel,
    ) -> Option<Path> {
        let Goal::Pose(goal) = goal else {
            return Some(Path::default());
        };
        let goal_position = goal.position();
        let footprint = vehicle.footprint();
        // Scale route margin down proportionally for small rovers (e.g. 5.6 cm
        // for a 0.16 m wide rover, capped at ROUTE_MARGIN_M = 0.15 m for larger ones).
        let margin = ROUTE_MARGIN_M.min(0.35 * footprint.length.min(footprint.width));

        // Coarse bounding-circle spatial index: any cell outside `coarse_reach`
        // is guaranteed clear regardless of heading.
        let half_cell_diagonal = self.resolution * std::f64::consts::FRAC_1_SQRT_2;
        let coarse_reach = footprint.radius() + margin + half_cell_diagonal;
        let nearby = self.nearby_obstacles(&map.obstacles, coarse_reach);

        // Quick goal rejection: if goal yaw is fixed, check exact footprint;
        // otherwise reject only if even the inscribed circle cannot fit.
        if let Some(yaw) = goal.yaw {
            let goal_pose = Pose2 { x: goal.x, y: goal.y, yaw };
            if self.pose_blocked(goal_pose, footprint, margin, &nearby) {
                return None;
            }
        } else if let Some(obs) = nearby.get(&self.cell(goal_position)) {
            let min_half = footprint.length.min(footprint.width) * 0.5 + margin;
            if obs.iter().any(|&o| goal_position.distance(o) <= min_half) {
                return None;
            }
        }

        // One arc per curvature: w = v · curvature, held for 1 s at
        // v = `step`, so every arc is `step` metres long.
        let mut curvatures = vec![0.0];
        for radii in TURN_RADII_IN_ROVER_RADII {
            let curvature = 1.0 / (radii * vehicle.radius());
            curvatures.extend([curvature, -curvature]);
        }

        let estimate_to_go =
            |pose: &Pose2| self.heuristic_weight * pose.position().distance(goal_position);

        let mut nodes = vec![Node { pose: state.pose, cost: 0.0, dir: 0, parent: None }];
        // The cheapest cost found so far for each key.
        let mut cheapest: HashMap<Key, f64> = HashMap::from([(self.key(&state.pose), 0.0)]);
        let mut open = BinaryHeap::from([Open { estimate: estimate_to_go(&state.pose), index: 0 }]);

        let directions: &[i8] = if self.allow_reverse { &[1, -1] } else { &[1] };

        let mut expansions = 0;
        while let Some(Open { index, .. }) = open.pop() {
            let pose = nodes[index].pose;
            let cost = nodes[index].cost;
            let node_dir = nodes[index].dir;

            // A cheaper way to this key was found after this entry was
            // queued: the entry is out of date.
            if cost > cheapest[&self.key(&pose)] {
                continue;
            }

            // Done: walk the parents back to the start, then add the goal.
            if pose.position().distance(goal_position) <= self.step {
                let mut poses = vec![Pose2 {
                    x: goal.x,
                    y: goal.y,
                    yaw: goal.yaw.unwrap_or(pose.yaw),
                }];
                let mut at = Some(index);
                while let Some(i) = at {
                    poses.push(nodes[i].pose);
                    at = nodes[i].parent;
                }
                poses.reverse();
                return Some(Path { poses });
            }

            expansions += 1;
            if expansions > MAX_EXPANSIONS {
                return None;
            }

            for &dir in directions {
                let v = f64::from(dir) * self.step;
                let step_cost = if dir < 0 { self.step * self.reverse_penalty } else { self.step };
                let switch_cost = if node_dir != 0 && dir != node_dir {
                    DIRECTION_CHANGE_PENALTY_M
                } else {
                    0.0
                };
                for &curvature in &curvatures {
                    let arc = Twist { v, w: v * curvature };
                    let mid = vehicle.step(pose, arc, 0.5);
                    let next = vehicle.step(pose, arc, 1.0);
                    if self.pose_blocked(mid, footprint, margin, &nearby)
                        || self.pose_blocked(next, footprint, margin, &nearby)
                    {
                        continue;
                    }
                    let next_cost = cost + step_cost + switch_cost;
                    let key = self.key(&next);
                    if cheapest.get(&key).is_some_and(|&known| known <= next_cost) {
                        continue;
                    }
                    cheapest.insert(key, next_cost);
                    nodes.push(Node { pose: next, cost: next_cost, dir, parent: Some(index) });
                    open.push(Open {
                        estimate: next_cost + estimate_to_go(&next),
                        index: nodes.len() - 1,
                    });
                }
            }
        }
        // The open list ran dry: every reachable pose was tried.
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::PoseGoal;
    use crate::vehicle::{DiffDrive, Footprint, Limits};

    fn test_vehicle() -> DiffDrive {
        DiffDrive {
            track: 0.30,
            footprint: Footprint::from_radius(0.25),
            limits: Limits { max_wheel_speed: 1.0, max_accel: 1.0, max_yaw_accel: 2.0 },
        }
    }

    fn dead_end_map() -> Map {
        // U-shaped pocket around (0, 0) facing +X: walls at x = 0.65, y = ±0.65, open behind (-X).
        let mut obstacles = Vec::new();
        for i in -6..=6 {
            let y = i as f64 * 0.1;
            obstacles.push(Vec2 { x: 0.65, y });
        }
        for i in -6..=6 {
            let x = i as f64 * 0.1;
            obstacles.push(Vec2 { x, y: 0.65 });
            obstacles.push(Vec2 { x, y: -0.65 });
        }
        Map { obstacles, free: Vec::new(), resolution: 0.1, revision: 1 }
    }

    #[test]
    fn backs_out_of_a_dead_end() {
        let vehicle = test_vehicle();
        let map = dead_end_map();
        let state = State {
            t: 0.0,
            pose: Pose2 { x: 0.0, y: 0.0, yaw: 0.0 },
            twist: Twist::STOP,
        };
        let goal = Goal::Pose(PoseGoal { x: -1.8, y: 0.0, yaw: None, tolerance: 0.2 });

        let mut forward_only = HybridAStar {
            resolution: 0.1,
            yaw_bins: 72,
            step: 0.3,
            heuristic_weight: 1.5,
            allow_reverse: false,
            reverse_penalty: 2.0,
        };
        assert!(forward_only.plan(&state, &map, &goal, &vehicle).is_none());

        let mut with_reverse = HybridAStar { allow_reverse: true, ..forward_only };
        let path = with_reverse
            .plan(&state, &map, &goal, &vehicle)
            .expect("should back out of dead-end pocket");
        assert!(path.poses.len() >= 2);
        // First step moves backward (-X) while facing +X.
        assert!(path.poses[1].x < path.poses[0].x);
    }

    #[test]
    fn prefers_forward_in_open_space() {
        let vehicle = test_vehicle();
        let map = Map::default();
        let state = State {
            t: 0.0,
            pose: Pose2 { x: 0.0, y: 0.0, yaw: 0.0 },
            twist: Twist::STOP,
        };
        let goal = Goal::Pose(PoseGoal { x: 2.0, y: 0.0, yaw: None, tolerance: 0.2 });
        let mut planner = HybridAStar {
            resolution: 0.1,
            yaw_bins: 72,
            step: 0.3,
            heuristic_weight: 1.5,
            allow_reverse: true,
            reverse_penalty: 2.0,
        };
        let path = planner.plan(&state, &map, &goal, &vehicle).expect("open-space path");
        assert!(path.poses.windows(2).all(|p| p[1].x >= p[0].x));
    }

    #[test]
    fn plans_through_narrow_corridor_with_rectangular_footprint() {
        // 0.60 m long × 0.30 m wide rover:
        // - bounding circle diameter + 2×ROUTE_MARGIN_M = 0.67 + 0.30 = 0.97 m (blocked!)
        // - rectangular width + 2×ROUTE_MARGIN_M = 0.30 + 0.30 = 0.60 m (fits a 0.70 m corridor!)
        let vehicle = DiffDrive {
            track: 0.28,
            footprint: Footprint { length: 0.60, width: 0.30 },
            limits: Limits { max_wheel_speed: 1.0, max_accel: 1.0, max_yaw_accel: 2.0 },
        };
        let mut obstacles = Vec::new();
        for i in 0..=20 {
            let x = i as f64 * 0.1;
            obstacles.push(Vec2 { x, y: 0.35 });
            obstacles.push(Vec2 { x, y: -0.35 });
        }
        let map = Map { obstacles, free: Vec::new(), resolution: 0.1, revision: 1 };
        let state = State {
            t: 0.0,
            pose: Pose2 { x: 0.0, y: 0.0, yaw: 0.0 },
            twist: Twist::STOP,
        };
        let goal = Goal::Pose(PoseGoal { x: 2.0, y: 0.0, yaw: None, tolerance: 0.2 });
        let mut planner = HybridAStar {
            resolution: 0.1,
            yaw_bins: 72,
            step: 0.3,
            heuristic_weight: 1.5,
            allow_reverse: true,
            reverse_penalty: 2.0,
        };
        let path = planner
            .plan(&state, &map, &goal, &vehicle)
            .expect("rectangular footprint should fit through 0.70 m corridor");
        assert!(path.poses.len() >= 2);
    }
}
