//! Waypoint routing with Hybrid A*, including goals the rover has not seen yet.
//!
//! A [`RouteJob`] is advanced a bounded number of expansions per control tick:
//!
//! 1. A goal off the map moves to the nearest point inside it. A goal whose
//!    body would touch an observed obstacle moves to the nearest point within
//!    `snap` metres where the body fits at some heading (unknown cells allowed);
//!    if there is none, the goal is blocked.
//! 2. The search first stays in known-free space. A quick 2D check rules this
//!    out at once when no known-free connection exists.
//! 3. Otherwise it may cross unknown cells at [`UNKNOWN_COST`] times the cost,
//!    and only the known-free start of that route is returned: the rover drives
//!    to the edge of what it has seen, the map fills in, and the caller replans.
use crate::footprint::{self, Unknown};
use crate::{
    HybridConfig, HybridGoal, HybridSearch, HybridWaypoint, Pose,
};
use terra_mapping::MapSnapshot;

/// Cost multiple for crossing unknown cells; routes prefer observed space but
/// may cross unseen space when it is much shorter.
pub const UNKNOWN_COST: f64 = 3.;

/// A drivable route and whether it ends at the requested goal.
#[derive(Clone, Debug, PartialEq)]
pub struct HybridRoute {
    /// Samples at most one map cell apart, starting at the rover.
    pub waypoints: Vec<HybridWaypoint>,
    /// False when the route stops short: at the edge of known space, at a
    /// snapped goal, or at the map edge toward an off-map goal.
    pub reaches_goal: bool,
}

enum Stage {
    Known(HybridSearch),
    Unknown(HybridSearch),
    Done,
}

/// A route to a waypoint goal, computed a few expansions at a time.
pub struct RouteJob {
    map: MapSnapshot,
    start: Pose,
    goal: HybridGoal,
    config: HybridConfig,
    /// The goal was moved (snapped, or pulled inside the map).
    moved: bool,
    stage: Stage,
}

impl RouteJob {
    /// Start routing from `start` toward `(x, y)`.
    ///
    /// # Errors
    /// - `route_invalid_map`: invalid map, configuration or coordinates.
    /// - `route_start_outside_map`, `route_start_blocked`.
    /// - `route_goal_blocked`: the goal cell is an observed obstacle, or no point
    ///   within `snap` metres of it fits the body clear of observed obstacles.
    /// - `route_no_path`: observed obstacles seal the goal off.
    pub fn new(
        map: &MapSnapshot,
        start: Pose,
        x: f64,
        y: f64,
        config: &HybridConfig,
        snap: f64,
    ) -> Result<Self, &'static str> {
        if !map.valid()
            || !start.finite()
            || !x.is_finite()
            || !y.is_finite()
            || !snap.is_finite()
        {
            return Err("route_invalid_map");
        }
        map.cell(start.x, start.y).ok_or("route_start_outside_map")?;
        let mut moved = false;
        let (mut gx, mut gy) = (x, y);
        match map.at(x, y) {
            None => {
                (gx, gy) = inside_map(map, x, y, config.footprint.circumradius());
                moved = true;
            }
            Some(v) if v >= terra_mapping::OCCUPIED => {
                return Err("route_goal_blocked");
            }
            Some(_) => {}
        }
        if !fits_somewhere(map, config, gx, gy) {
            (gx, gy) = snap_goal(map, config, gx, gy, snap).ok_or("route_goal_blocked")?;
            moved = true;
        }
        let goal = HybridGoal {
            x: gx,
            y: gy,
            yaw: None,
        };
        let stage = match HybridSearch::new(map, start, goal, config, None) {
            Ok(search) => Stage::Known(search),
            Err("route_no_path" | "route_goal_blocked") => Stage::Unknown(HybridSearch::new(
                map,
                start,
                goal,
                config,
                Some(UNKNOWN_COST),
            )?),
            Err(why) => return Err(why),
        };
        Ok(Self {
            map: map.clone(),
            start,
            goal,
            config: config.clone(),
            moved,
            stage,
        })
    }

    /// The goal actually routed to, after any snapping or clamping.
    pub fn goal(&self) -> (f64, f64) {
        (self.goal.x, self.goal.y)
    }

    /// Run up to `budget` expansions. `None` while still searching.
    pub fn advance(&mut self, budget: u32) -> Option<Result<HybridRoute, &'static str>> {
        match &mut self.stage {
            Stage::Known(search) => match search.advance(budget)? {
                Ok(waypoints) => {
                    self.stage = Stage::Done;
                    Some(Ok(HybridRoute {
                        waypoints,
                        reaches_goal: !self.moved,
                    }))
                }
                Err(_) => {
                    match HybridSearch::new(
                        &self.map,
                        self.start,
                        self.goal,
                        &self.config,
                        Some(UNKNOWN_COST),
                    ) {
                        Ok(search) => {
                            self.stage = Stage::Unknown(search);
                            None
                        }
                        Err(why) => {
                            self.stage = Stage::Done;
                            Some(Err(why))
                        }
                    }
                }
            },
            Stage::Unknown(search) => {
                let result = search.advance(budget)?;
                self.stage = Stage::Done;
                Some(result.and_then(|waypoints| {
                    let prefix = known_prefix(&self.map, &self.config, self.start, &waypoints);
                    let length: f64 = prefix
                        .windows(2)
                        .map(|w| (w[1].x - w[0].x).hypot(w[1].y - w[0].y))
                        .sum();
                    if length < self.map.resolution {
                        return Err("route_no_path");
                    }
                    let complete = prefix.len() == waypoints.len();
                    Ok(HybridRoute {
                        waypoints: prefix,
                        reaches_goal: complete && !self.moved,
                    })
                }))
            }
            Stage::Done => Some(Err("route_no_path")),
        }
    }
}

/// Whether the body fits at `(x, y)` at some heading, counting only observed
/// obstacles and the map edge.
fn fits_somewhere(map: &MapSnapshot, config: &HybridConfig, x: f64, y: f64) -> bool {
    footprint::fits_some_heading(map, &config.footprint, x, y, Unknown::Anywhere)
}

/// The nearest cell centre within `snap` metres of `(x, y)` where the body fits.
fn snap_goal(
    map: &MapSnapshot,
    config: &HybridConfig,
    x: f64,
    y: f64,
    snap: f64,
) -> Option<(f64, f64)> {
    let res = map.resolution;
    let reach = (snap / res).ceil() as i64;
    let (c0, r0) = map.cell(x, y)?;
    let mut candidates = Vec::new();
    for dr in -reach..=reach {
        for dc in -reach..=reach {
            let (c, r) = (c0 as i64 + dc, r0 as i64 + dr);
            if c < 0 || r < 0 || c >= map.width as i64 || r >= map.height as i64 {
                continue;
            }
            let cx = map.origin_x + (c as f64 + 0.5) * res;
            let cy = map.origin_y + (r as f64 + 0.5) * res;
            let distance = (cx - x).hypot(cy - y);
            if distance <= snap {
                candidates.push((distance, cx, cy));
            }
        }
    }
    candidates.sort_by(|a, b| a.0.total_cmp(&b.0));
    candidates
        .into_iter()
        .find(|&(_, cx, cy)| fits_somewhere(map, config, cx, cy))
        .map(|(_, cx, cy)| (cx, cy))
}

/// The point inside the map nearest `(x, y)`, kept `inset` metres from the edge.
fn inside_map(map: &MapSnapshot, x: f64, y: f64, inset: f64) -> (f64, f64) {
    let span_x = map.width as f64 * map.resolution;
    let span_y = map.height as f64 * map.resolution;
    let clamp = |v: f64, lo: f64, span: f64| {
        let inset = inset.min(span / 2.);
        v.clamp(lo + inset, lo + span - inset)
    };
    (
        clamp(x, map.origin_x, span_x),
        clamp(y, map.origin_y, span_y),
    )
}

/// Waypoints up to (not including) the first whose body overlaps unknown cells,
/// other than those under the body at `start`.
fn known_prefix(
    map: &MapSnapshot,
    config: &HybridConfig,
    start: Pose,
    route: &[HybridWaypoint],
) -> Vec<HybridWaypoint> {
    route
        .iter()
        .take_while(|w| {
            let pose = Pose {
                x: w.x,
                y: w.y,
                yaw: w.yaw,
            };
            footprint::rejection(map, &config.footprint, pose, 0., Unknown::UnderBody(start))
                .is_none()
        })
        .copied()
        .collect()
}
