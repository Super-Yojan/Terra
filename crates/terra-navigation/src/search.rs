//! Mission-scoped observed map and deterministic routes. No target truth is accepted here.
use crate::{Pose, reachable};
use terra_mapping::MapSnapshot;
#[derive(Clone, Copy, Debug)]
pub struct SearchBounds {
    pub min_x: f64,
    pub min_y: f64,
    pub max_x: f64,
    pub max_y: f64,
}
#[derive(Clone, Debug, PartialEq)]
pub enum SearchNavigationDecision {
    Goal { x: f64, y: f64 },
    Exhausted,
    Blocked { reason: &'static str },
}
pub struct SearchNavigator {
    bounds: SearchBounds,
    radius: f64,
    map: Option<MapSnapshot>,
    last_revision: Option<u64>,
    last_time: Option<f64>,
    goal: Option<(f64, f64)>,
    route: Vec<(f64, f64)>,
    excluded: Vec<(f64, f64, f64)>,
    anchor: Option<(Pose, f64)>,
    failures: u8,
}
impl SearchNavigator {
    pub fn new(bounds: SearchBounds, radius: f64) -> Result<Self, &'static str> {
        if ![
            bounds.min_x,
            bounds.min_y,
            bounds.max_x,
            bounds.max_y,
            radius,
        ]
        .iter()
        .all(|v| v.is_finite())
            || radius <= 0.
            || radius > 5.
            || bounds.max_x - bounds.min_x <= 2. * radius
            || bounds.max_y - bounds.min_y <= 2. * radius
            || bounds.max_x - bounds.min_x > 100.
            || bounds.max_y - bounds.min_y > 100.
        {
            return Err("invalid search geometry");
        }
        Ok(Self {
            bounds,
            radius,
            map: None,
            last_revision: None,
            last_time: None,
            goal: None,
            route: vec![],
            excluded: vec![],
            anchor: None,
            failures: 0,
        })
    }
    pub fn observed_cells(&self) -> usize {
        self.map
            .as_ref()
            .map_or(0, |m| m.occupancy.iter().filter(|&&v| v >= 0).count())
    }
    pub fn replan(&mut self) {
        self.goal = None;
        self.route.clear();
        self.anchor = None;
    }
    pub fn invalidate_goal(&mut self, now: f64) {
        if let Some((x, y)) = self.goal.take() {
            if self.excluded.len() == 256 {
                self.excluded.remove(0);
            }
            self.excluded.push((x, y, now + 30.));
        }
        self.route.clear();
        self.anchor = None;
        self.failures = self.failures.saturating_add(1);
    }
    pub fn record_progress(&mut self, pose: Pose, now: f64) {
        if self
            .anchor
            .is_none_or(|(p, _)| (p.x - pose.x).hypot(p.y - pose.y) >= 0.25)
        {
            self.anchor = Some((pose, now));
        }
    }
    fn merge(&mut self, m: &MapSnapshot, revision: u64) -> Result<(), &'static str> {
        if m.width == 0
            || m.height == 0
            || m.occupancy.len() != m.width as usize * m.height as usize
            || !m.resolution.is_finite()
            || m.resolution <= 0.
            || !m.origin_x.is_finite()
            || !m.origin_y.is_finite()
        {
            return Err("invalid_map");
        }
        if let Some(old) = self.last_revision {
            if revision < old {
                return Err("map_revision_regressed");
            }
        }
        if self.map.is_none() {
            let w = ((self.bounds.max_x - self.bounds.min_x) / m.resolution).ceil() as u32;
            let h = ((self.bounds.max_y - self.bounds.min_y) / m.resolution).ceil() as u32;
            if w as u64 * h as u64 > 250_000 {
                return Err("search_map_capacity");
            }
            self.map = Some(MapSnapshot {
                width: w,
                height: h,
                resolution: m.resolution,
                origin_x: self.bounds.min_x,
                origin_y: self.bounds.min_y,
                occupancy: vec![-1; w as usize * h as usize],
            });
        }
        let memory = self.map.as_mut().unwrap();
        if (memory.resolution - m.resolution).abs() > 1e-9 {
            return Err("map_frame_changed");
        }
        for y in 0..m.height {
            for x in 0..m.width {
                let value = m.occupancy[(y * m.width + x) as usize];
                if value < 0 {
                    continue;
                }
                let px = m.origin_x + (x as f64 + 0.5) * m.resolution;
                let py = m.origin_y + (y as f64 + 0.5) * m.resolution;
                let ix = ((px - memory.origin_x) / memory.resolution).floor() as isize;
                let iy = ((py - memory.origin_y) / memory.resolution).floor() as isize;
                if ix >= 0 && iy >= 0 && ix < memory.width as isize && iy < memory.height as isize {
                    memory.occupancy[iy as usize * memory.width as usize + ix as usize] = value;
                }
            }
        }
        self.last_revision = Some(revision);
        Ok(())
    }
    pub fn next_goal(
        &mut self,
        m: &MapSnapshot,
        pose: Pose,
        revision: u64,
        now: f64,
    ) -> SearchNavigationDecision {
        if !now.is_finite() || now < 0. || !pose.finite() || self.last_time.is_some_and(|t| now < t)
        {
            return SearchNavigationDecision::Blocked {
                reason: "invalid_time_or_pose",
            };
        }
        self.last_time = Some(now);
        if let Err(reason) = self.merge(m, revision) {
            return SearchNavigationDecision::Blocked { reason };
        }
        if pose.x - self.radius < self.bounds.min_x
            || pose.x + self.radius > self.bounds.max_x
            || pose.y - self.radius < self.bounds.min_y
            || pose.y + self.radius > self.bounds.max_y
        {
            return SearchNavigationDecision::Blocked {
                reason: "outside_search_bounds",
            };
        }
        self.record_progress(pose, now);
        if self.goal.is_some() && self.anchor.is_some_and(|(_, t)| now - t >= 10.) {
            self.invalidate_goal(now);
        }
        if self.failures >= 3 {
            return SearchNavigationDecision::Blocked {
                reason: "search_recovery_exhausted",
            };
        }
        self.excluded.retain(|p| p.2 > now);
        if let Some(&(x, y)) = self.route.first() {
            if (pose.x - x).hypot(pose.y - y) < 0.3 {
                self.route.remove(0);
            } else if !reachable(m, pose, self.radius, x, y) {
                self.invalidate_goal(now);
            }
        }
        if !self.route.is_empty() {
            let (x, y) = self.route[0];
            return SearchNavigationDecision::Goal { x, y };
        }
        if let Some((x, y)) = self.goal.take() {
            self.excluded.push((x, y, now + 30.));
        }
        let memory = self.map.as_ref().unwrap();
        let excluded: Vec<_> = self.excluded.iter().map(|p| (p.0, p.1)).collect();
        let Some(target) = crate::frontier(memory, pose, self.radius, &excluded) else {
            return if !excluded.is_empty() {
                SearchNavigationDecision::Blocked {
                    reason: "frontier_cooldown",
                }
            } else {
                SearchNavigationDecision::Exhausted
            };
        };
        self.route = match crate::astar_route(memory, pose, self.radius, target.0, target.1) {
            Ok(route) => approach_waypoints(&route),
            Err(_) => Vec::new(),
        };
        // Only hand the follower a waypoint reachable in the current observed map.
        if let Some(&(x, y)) = self.route.first() {
            if reachable(m, pose, self.radius, x, y) {
                self.goal = Some(target);
                if self.anchor.is_none() {
                    self.anchor = Some((pose, now));
                }
                return SearchNavigationDecision::Goal { x, y };
            }
        }
        SearchNavigationDecision::Blocked {
            reason: "route_outside_local_map",
        }
    }
}
/// Sample the compacted polyline into steps the search follower can consume
/// one at a time. The first sample is ahead of the rover, matching the old
/// cell-center route that omitted the starting pose.
fn approach_waypoints(route: &[[f64; 2]]) -> Vec<(f64, f64)> {
    let mut waypoints = Vec::new();
    for segment in route.windows(2) {
        let distance = (segment[1][0] - segment[0][0]).hypot(segment[1][1] - segment[0][1]);
        let steps = (distance / 0.25).ceil().max(1.0) as usize;
        for step in 1..=steps {
            let fraction = step as f64 / steps as f64;
            waypoints.push((
                segment[0][0] + (segment[1][0] - segment[0][0]) * fraction,
                segment[0][1] + (segment[1][1] - segment[0][1]) * fraction,
            ));
        }
    }
    waypoints
}
