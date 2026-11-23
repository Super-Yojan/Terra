//! Autonomous frontier exploration (`Goal::Explore`).
//!
//! A **frontier** is an observed-free grid cell adjacent to at least one
//! unknown cell (neither observed-free nor occupied). The explorer:
//!
//! 1. Runs a 4-neighbour BFS from the rover's current cell through observed
//!    free space, checking [`Footprint::clearance`] against nearby obstacles
//!    so only reachable cells where the rover fits are considered.
//! 2. Scores each reachable frontier cell by `unknown_gain / distance` (ignoring
//!    candidates closer than `min_distance` or inside a blacklisted region).
//! 3. Drives to the highest-scoring frontier using the task's [`Navigator`]
//!    (e.g. `hybrid_astar` + `dwa`).
//! 4. Blacklists frontiers that become unreachable (`NoRoute`) or make no
//!    progress for [`NO_PROGRESS_S`] seconds.
//! 5. If no reachable frontier is visible (for example, right at startup when
//!    the front-facing LiDAR has only seen a cone in front of the rover),
//!    performs a 360° in-place scan before declaring the area mapped.
use std::collections::{HashSet, VecDeque};
use std::f64::consts::{FRAC_PI_4, TAU};

use crate::estimation::State;
use crate::geometry::{Pose2, Vec2, wrap_angle};
use crate::mapping::Map;
use crate::planning::{Navigator, PoseGoal};
use crate::vehicle::{Footprint, Twist, VehicleModel};

/// Arrival radius for each intermediate frontier waypoint (m).
const FRONTIER_TOLERANCE_M: f64 = 0.35;
/// Seconds without getting `PROGRESS_EPS_M` closer before blacklisting a frontier.
const NO_PROGRESS_S: f64 = 10.0;
/// Distance reduction (m) that resets the frontier no-progress timer.
const PROGRESS_EPS_M: f64 = 0.15;
/// Seconds before a temporarily blacklisted frontier may be retried.
const BLACKLIST_S: f64 = 45.0;
/// Failures at the same spot before it stays blacklisted for the whole run.
const MAX_ATTEMPTS: u32 = 3;
/// Radius around a blacklisted frontier within which new candidates are skipped (m).
const EXCLUDE_RADIUS_M: f64 = 0.8;
/// In-place look-around turn rate (rad/s) when no frontier is visible.
const SCAN_YAW_RATE: f64 = 0.45;
/// Maximum duration of a single in-place scan (s).
const SCAN_TIMEOUT_S: f64 = 18.0;
/// Minimum angle turned (rad) before a scan can exit early on a newly seen frontier.
const SCAN_MIN_TURN_RAD: f64 = 0.6;
/// Minimum distance moved before the rover is allowed to scan again (m).
const RESCAN_MOVE_M: f64 = 0.5;
/// Extra safety margin around obstacles when checking frontier reachability (m).
const FRONTIER_MARGIN_M: f64 = 0.08;

/// Result of one exploration tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ExploreStep {
    /// Keep exploring: drive with `twist` (towards `goal`, or spinning in place when `None`).
    Drive { twist: Twist, goal: Option<PoseGoal> },
    /// No reachable frontiers remain, or the time budget expired.
    Done,
}

#[derive(Clone, Copy, Debug)]
struct BlacklistEntry {
    point: Vec2,
    expires_at: f64,
    attempts: u32,
}

#[derive(Clone, Copy, Debug)]
struct Scan {
    started_at: f64,
    last_yaw: f64,
    turned: f64,
}

/// Stateful frontier explorer owned by an active `Goal::Explore` task.
#[derive(Clone, Debug)]
pub struct Explorer {
    budget_s: Option<f64>,
    started_at: Option<f64>,
    target: Option<PoseGoal>,
    blacklist: Vec<BlacklistEntry>,
    scan: Option<Scan>,
    scanned_at: Option<Vec2>,
    last_progress_at: f64,
    best_dist: f64,
}

impl Explorer {
    pub fn new(budget_s: Option<f64>) -> Self {
        Self {
            budget_s: budget_s.filter(|b| b.is_finite() && *b > 0.0),
            started_at: None,
            target: None,
            blacklist: Vec::new(),
            scan: None,
            scanned_at: None,
            last_progress_at: 0.0,
            best_dist: f64::INFINITY,
        }
    }

    /// Current frontier waypoint, if any.
    pub fn target(&self) -> Option<PoseGoal> {
        self.target
    }

    /// Reset transient timers/scan when resuming after a pause or attention hold.
    pub fn reset_for_resume(&mut self) {
        self.scan = None;
        self.target = None;
    }

    /// Advance exploration by one tick.
    pub fn step(
        &mut self,
        t: f64,
        state: &State,
        map: &Map,
        navigator: &mut Navigator,
        vehicle: &dyn VehicleModel,
    ) -> ExploreStep {
        let started_at = *self.started_at.get_or_insert(t);
        if self.budget_s.is_some_and(|budget| t - started_at >= budget) {
            return ExploreStep::Done;
        }

        self.prune_blacklist(t);
        let footprint = vehicle.footprint();

        // 1. In-place look-around scan (when no frontier was visible).
        if let Some(scan) = &mut self.scan {
            scan.turned += wrap_angle(state.pose.yaw - scan.last_yaw).abs();
            scan.last_yaw = state.pose.yaw;
            let timed_out = t - scan.started_at >= SCAN_TIMEOUT_S;
            let full_turn = scan.turned >= TAU;

            if scan.turned >= SCAN_MIN_TURN_RAD
                && let Some(frontier) = self.pick_frontier(map, state.pose, footprint)
            {
                self.scan = None;
                self.set_target(frontier, state.pose, t);
            } else if full_turn || timed_out {
                self.scan = None;
                self.scanned_at = Some(state.pose.position());
            } else {
                return ExploreStep::Drive {
                    twist: Twist { v: 0.0, w: SCAN_YAW_RATE },
                    goal: None,
                };
            }
        }

        // 2. Check current frontier target for arrival or lack of progress.
        if let Some(goal) = self.target {
            let dist = goal.distance(&state.pose);
            if dist <= goal.tolerance {
                self.target = None;
            } else if dist < self.best_dist - PROGRESS_EPS_M {
                self.best_dist = dist;
                self.last_progress_at = t;
            } else if t - self.last_progress_at > NO_PROGRESS_S {
                self.blacklist_point(goal.position(), t);
                self.target = None;
            }
        }

        // 3. Pick a new frontier if we don't have one.
        if self.target.is_none() {
            if let Some(frontier) = self.pick_frontier(map, state.pose, footprint) {
                self.set_target(frontier, state.pose, t);
            } else {
                let can_scan = self
                    .scanned_at
                    .is_none_or(|p| p.distance(state.pose.position()) > RESCAN_MOVE_M);
                if can_scan {
                    self.scan = Some(Scan {
                        started_at: t,
                        last_yaw: state.pose.yaw,
                        turned: 0.0,
                    });
                    return ExploreStep::Drive {
                        twist: Twist { v: 0.0, w: SCAN_YAW_RATE },
                        goal: None,
                    };
                }
                return ExploreStep::Done;
            }
        }

        // 4. Drive toward the active frontier via the task's Navigator.
        let goal = self.target.expect("target set above");
        match navigator.drive(state, map, &goal, vehicle) {
            Ok(twist) => ExploreStep::Drive { twist, goal: Some(goal) },
            Err(_no_route) => {
                // Unreachable right now: blacklist it and try another on the next tick.
                self.blacklist_point(goal.position(), t);
                self.target = None;
                ExploreStep::Drive { twist: Twist::STOP, goal: None }
            }
        }
    }

    fn set_target(&mut self, point: Vec2, pose: Pose2, t: f64) {
        let goal = PoseGoal {
            x: point.x,
            y: point.y,
            yaw: None,
            tolerance: FRONTIER_TOLERANCE_M,
        };
        self.target = Some(goal);
        self.best_dist = goal.distance(&pose);
        self.last_progress_at = t;
        self.scanned_at = None;
    }

    fn prune_blacklist(&mut self, t: f64) {
        self.blacklist
            .retain(|e| e.attempts >= MAX_ATTEMPTS || t < e.expires_at);
    }

    fn blacklist_point(&mut self, point: Vec2, t: f64) {
        if let Some(existing) = self
            .blacklist
            .iter_mut()
            .find(|e| e.point.distance(point) <= EXCLUDE_RADIUS_M)
        {
            existing.attempts = existing.attempts.saturating_add(1);
            existing.expires_at = t + BLACKLIST_S;
            return;
        }
        self.blacklist.push(BlacklistEntry {
            point,
            expires_at: t + BLACKLIST_S,
            attempts: 1,
        });
    }

    fn pick_frontier(&self, map: &Map, pose: Pose2, footprint: Footprint) -> Option<Vec2> {
        let excluded: Vec<Vec2> = self.blacklist.iter().map(|e| e.point).collect();
        select_frontier(map, pose, footprint, &excluded, EXCLUDE_RADIUS_M)
    }
}

/// Pick the highest-scoring reachable frontier in `map` (world metres).
///
/// Returns `None` when the map has no resolution, no free cells, or no
/// reachable frontier outside `excluded`.
pub fn select_frontier(
    map: &Map,
    pose: Pose2,
    footprint: Footprint,
    excluded: &[Vec2],
    exclude_radius: f64,
) -> Option<Vec2> {
    let res = map.resolution;
    if !res.is_finite() || res <= 0.0 || map.free.is_empty() {
        return None;
    }
    if !pose.x.is_finite() || !pose.y.is_finite() || !pose.yaw.is_finite() {
        return None;
    }

    let to_cell = |p: Vec2| -> (i32, i32) {
        ((p.x / res).floor() as i32, (p.y / res).floor() as i32)
    };
    let to_world = |(ix, iy): (i32, i32)| -> Vec2 {
        Vec2 {
            x: (f64::from(ix) + 0.5) * res,
            y: (f64::from(iy) + 0.5) * res,
        }
    };

    let free_cells: HashSet<(i32, i32)> = map.free.iter().map(|&p| to_cell(p)).collect();
    let occupied_cells: HashSet<(i32, i32)> = map.obstacles.iter().map(|&p| to_cell(p)).collect();

    let start = to_cell(pose.position());
    let seed = if free_cells.contains(&start) {
        start
    } else {
        // Snap to the nearest observed free cell within 0.6 m so exploration
        // still starts even if the LiDAR blind zone sits directly under the rover.
        map.free
            .iter()
            .filter(|&&p| p.distance(pose.position()) <= 0.6)
            .min_by(|a, b| {
                a.distance(pose.position())
                    .total_cmp(&b.distance(pose.position()))
            })
            .map(|&p| to_cell(p))?
    };

    let radius = footprint.radius();
    let min_dist = radius.max(0.6);
    let margin = FRONTIER_MARGIN_M.min(0.25 * footprint.length.min(footprint.width));
    let gain_r = (radius.max(0.4) / res).ceil() as i32;
    let obs_search_r = radius + margin + res;

    // Spatial hash of obstacles (binned at 1 m) for fast clearance checks.
    let mut obs_bins: std::collections::HashMap<(i32, i32), Vec<Vec2>> =
        std::collections::HashMap::new();
    for &o in &map.obstacles {
        let bin = (o.x.floor() as i32, o.y.floor() as i32);
        obs_bins.entry(bin).or_default().push(o);
    }

    let fits = |p: Vec2| -> bool {
        if map.obstacles.is_empty() {
            return true;
        }
        let min_bx = (p.x - obs_search_r).floor() as i32;
        let max_bx = (p.x + obs_search_r).floor() as i32;
        let min_by = (p.y - obs_search_r).floor() as i32;
        let max_by = (p.y + obs_search_r).floor() as i32;

        // Heading-free fit: the rover fits if it has clearance at any of 4 orientations.
        for k in 0..4 {
            let candidate = Pose2 { x: p.x, y: p.y, yaw: k as f64 * FRAC_PI_4 };
            let mut ok = true;
            'bins: for bx in min_bx..=max_bx {
                for by in min_by..=max_by {
                    if let Some(obs) = obs_bins.get(&(bx, by)) {
                        for &o in obs {
                            if footprint.clearance(candidate, o) < margin {
                                ok = false;
                                break 'bins;
                            }
                        }
                    }
                }
            }
            if ok {
                return true;
            }
        }
        false
    };

    let mut visited = HashSet::new();
    let mut queue = VecDeque::new();
    visited.insert(seed);
    queue.push_back(seed);

    let mut best: Option<(f64, Vec2)> = None;

    while let Some(cell @ (ix, iy)) = queue.pop_front() {
        let world = to_world(cell);

        // Allow leaving the start cell even if an obstacle is close, otherwise require fit.
        if cell != seed && !fits(world) {
            continue;
        }

        // Check if this free cell borders at least one unknown 4-neighbour.
        let neighbours = [(ix + 1, iy), (ix - 1, iy), (ix, iy + 1), (ix, iy - 1)];
        let is_frontier = neighbours
            .iter()
            .any(|n| !free_cells.contains(n) && !occupied_cells.contains(n));

        if is_frontier {
            let dist = world.distance(pose.position());
            let clear_of_excluded = excluded
                .iter()
                .all(|&ex| world.distance(ex) > exclude_radius);
            if dist >= min_dist && clear_of_excluded {
                let mut gain = 0u32;
                for dx in -gain_r..=gain_r {
                    for dy in -gain_r..=gain_r {
                        let n = (ix + dx, iy + dy);
                        if !free_cells.contains(&n) && !occupied_cells.contains(&n) {
                            gain += 1;
                        }
                    }
                }
                if gain > 0 {
                    let score = f64::from(gain) / dist.max(0.5);
                    if best.is_none_or(|(best_score, _)| score > best_score) {
                        best = Some((score, world));
                    }
                }
            }
        }

        for n in neighbours {
            if free_cells.contains(&n) && visited.insert(n) {
                queue.push_back(n);
            }
        }
    }

    best.map(|(_, p)| p)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::global::StraightLine;
    use crate::planning::local::DrivePose;
    use crate::vehicle::DiffDrive;

    fn make_free_box(res: f64, x_cells: std::ops::Range<i32>, y_cells: std::ops::Range<i32>) -> Map {
        let mut free = Vec::new();
        for ix in x_cells {
            for iy in y_cells.clone() {
                free.push(Vec2 {
                    x: (f64::from(ix) + 0.5) * res,
                    y: (f64::from(iy) + 0.5) * res,
                });
            }
        }
        Map { obstacles: Vec::new(), free, resolution: res, revision: 1 }
    }

    #[test]
    fn selects_frontier_at_edge_of_observed_free_space() {
        let map = make_free_box(0.2, 0..15, -3..4); // x in [0, 3], y in [-0.6, 0.8]
        let pose = Pose2 { x: 0.3, y: 0.1, yaw: 0.0 };
        let fp = Footprint { length: 0.3, width: 0.2 };

        let frontier = select_frontier(&map, pose, fp, &[], 0.8).expect("should find frontier");
        assert!(frontier.distance(pose.position()) >= 0.6);
    }

    #[test]
    fn enclosed_room_has_no_frontier() {
        let res = 0.2;
        let mut map = make_free_box(res, 0..10, 0..10);
        // Surround the entire free box with walls.
        for i in -1..=10 {
            for &(ix, iy) in &[(i, -1), (i, 10), (-1, i), (10, i)] {
                map.obstacles.push(Vec2 {
                    x: (f64::from(ix) + 0.5) * res,
                    y: (f64::from(iy) + 0.5) * res,
                });
            }
        }
        let pose = Pose2 { x: 1.0, y: 1.0, yaw: 0.0 };
        let fp = Footprint { length: 0.3, width: 0.2 };
        assert_eq!(select_frontier(&map, pose, fp, &[], 0.8), None);
    }

    fn test_vehicle() -> DiffDrive {
        DiffDrive {
            track: 0.3,
            footprint: Footprint { length: 0.35, width: 0.25 },
            limits: crate::vehicle::Limits {
                max_wheel_speed: 0.8,
                max_accel: 1.0,
                max_yaw_accel: 2.0,
            },
        }
    }

    fn test_controller() -> Box<dyn crate::planning::local::LocalPlanner> {
        DrivePose::spec().build(&Default::default()).unwrap().1
    }

    #[test]
    fn explorer_scans_when_no_frontier_then_finishes() {
        let map = Map { obstacles: Vec::new(), free: Vec::new(), resolution: 0.2, revision: 1 };
        let vehicle = test_vehicle();
        let mut nav = Navigator::new(Box::new(StraightLine), test_controller());
        let mut explorer = Explorer::new(None);

        // First tick starts a look-around scan.
        let s0 = State { t: 0.0, pose: Pose2 { x: 0.0, y: 0.0, yaw: 0.0 }, twist: Twist::STOP };
        let step0 = explorer.step(0.0, &s0, &map, &mut nav, &vehicle);
        assert!(matches!(step0, ExploreStep::Drive { twist, goal: None } if twist.w > 0.0));

        // After scan timeout with still no frontiers, exploration finishes cleanly.
        let s1 = State { t: 20.0, pose: Pose2 { x: 0.0, y: 0.0, yaw: 0.0 }, twist: Twist::STOP };
        let step1 = explorer.step(20.0, &s1, &map, &mut nav, &vehicle);
        assert_eq!(step1, ExploreStep::Done);
    }

    #[test]
    fn explorer_respects_time_budget() {
        let map = make_free_box(0.2, 0..15, -3..4);
        let vehicle = test_vehicle();
        let mut nav = Navigator::new(Box::new(StraightLine), test_controller());
        let mut explorer = Explorer::new(Some(5.0));

        let s0 = State { t: 10.0, pose: Pose2 { x: 0.3, y: 0.1, yaw: 0.0 }, twist: Twist::STOP };
        assert!(matches!(
            explorer.step(10.0, &s0, &map, &mut nav, &vehicle),
            ExploreStep::Drive { goal: Some(_), .. }
        ));

        let s1 = State { t: 15.5, pose: Pose2 { x: 0.5, y: 0.1, yaw: 0.0 }, twist: Twist::STOP };
        assert_eq!(explorer.step(15.5, &s1, &map, &mut nav, &vehicle), ExploreStep::Done);
    }
}
