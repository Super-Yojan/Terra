//! Mission-scoped observed map and deterministic routes. No target truth is accepted here.
use crate::{InformationGainScorer, select_frontier};
use terra_mapping::MapSnapshot;
use serde::{Deserialize, Serialize};
use terra_navigation::{Footprint, Pose, reachable};
/// Longest side of a search area, in metres.
pub const MAX_SEARCH_SIDE_M: f64 = 100.;
/// Axis-aligned search rectangle in world metres; valid sides are positive
/// and at most [`MAX_SEARCH_SIDE_M`]. The whole rover body stays inside it.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SearchBounds {
    /// Inclusive minimum world x coordinate, in metres.
    pub min_x: f64,
    /// Inclusive minimum world y coordinate, in metres.
    pub min_y: f64,
    /// Inclusive maximum world x coordinate, in metres.
    pub max_x: f64,
    /// Inclusive maximum world y coordinate, in metres.
    pub max_y: f64,
}
impl SearchBounds {
    /// Whether all coordinates are finite and each side is positive and at
    /// most [`MAX_SEARCH_SIDE_M`].
    pub fn valid(self) -> bool {
        [self.min_x, self.min_y, self.max_x, self.max_y]
            .iter()
            .all(|v| v.is_finite())
            && self.max_x > self.min_x
            && self.max_y > self.min_y
            && self.max_x - self.min_x <= MAX_SEARCH_SIDE_M
            && self.max_y - self.min_y <= MAX_SEARCH_SIDE_M
    }
    /// Whether finite world-metre coordinates lie inside the rectangle, including its edges.
    pub fn contains(self, x: f64, y: f64) -> bool {
        x.is_finite()
            && y.is_finite()
            && x >= self.min_x
            && x <= self.max_x
            && y >= self.min_y
            && y <= self.max_y
    }
    /// Whether the body rectangle at the pose's heading lies inside the bounds.
    pub fn contains_body(self, footprint: &Footprint, pose: Pose) -> bool {
        let (a, b) = footprint.half_extents(0.);
        let (sin, cos) = pose.yaw.sin_cos();
        let reach_x = a * cos.abs() + b * sin.abs();
        let reach_y = a * sin.abs() + b * cos.abs();
        pose.x - reach_x >= self.min_x
            && pose.x + reach_x <= self.max_x
            && pose.y - reach_y >= self.min_y
            && pose.y + reach_y <= self.max_y
    }
}
/// Next action from the mission's accumulated observations and recovery state.
#[derive(Clone, Debug, PartialEq)]
pub enum SearchNavigationDecision {
    /// Next route waypoint in world metres, rather than necessarily the final frontier.
    Goal {
        /// World x coordinate of the waypoint in metres.
        x: f64,
        /// World y coordinate of the waypoint in metres.
        y: f64,
    },
    /// No frontier remains and no excluded target is waiting for its cooldown.
    Exhausted,
    /// Navigation cannot proceed; `reason` identifies the failed check or recovery limit.
    Blocked {
        /// Machine-readable failure code; see [`SearchNavigator::next_goal`].
        reason: &'static str,
    },
}
/// Accumulates observations across local map windows and follows routes to frontiers.
///
/// Construct a new navigator for each mission to reset map memory and recovery failures.
/// Known cells in incoming windows overwrite previous observations; unknown cells
/// leave memory intact. All windows must share world coordinates and resolution.
///
/// # Examples
///
/// ```
/// use terra_exploration::{SearchBounds, SearchNavigator};
/// use terra_navigation::Footprint;
///
/// let bounds = SearchBounds {
///     min_x: -10.0, min_y: -10.0, max_x: 10.0, max_y: 10.0,
/// };
/// let mut navigator = SearchNavigator::new(bounds, Footprint::default())?;
/// assert_eq!(navigator.observed_cells(), 0);
/// navigator.replan(); // Observations and recovery history would be retained.
/// # Ok::<(), &'static str>(())
/// ```
pub struct SearchNavigator {
    bounds: SearchBounds,
    footprint: Footprint,
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
    /// Create a mission for a rover with this body footprint.
    ///
    /// Each bounds dimension must exceed the diameter of the circle around the
    /// body (room to turn at any heading) and be at most [`MAX_SEARCH_SIDE_M`].
    ///
    /// # Errors
    ///
    /// Returns `invalid search geometry` when a boundary is nonfinite, the
    /// footprint is invalid, or either bounds dimension is invalid.
    /// Map memory is allocated later by [`Self::next_goal`], using the first map's resolution.
    pub fn new(bounds: SearchBounds, footprint: Footprint) -> Result<Self, &'static str> {
        let radius = footprint.circumradius();
        if !bounds.valid()
            || !footprint.valid()
            || bounds.max_x - bounds.min_x <= 2. * radius
            || bounds.max_y - bounds.min_y <= 2. * radius
        {
            return Err("invalid search geometry");
        }
        Ok(Self {
            bounds,
            footprint,
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
    /// Count known cells in accumulated mission memory, including occupied cells.
    pub fn observed_cells(&self) -> usize {
        self.map
            .as_ref()
            .map_or(0, |m| m.occupancy.iter().filter(|&&v| v >= 0).count())
    }
    /// Discard the active route and progress anchor, retaining observations and failures.
    pub fn replan(&mut self) {
        self.goal = None;
        self.route.clear();
        self.anchor = None;
    }
    /// Abandon the route, exclude its target for 30 s, and increment recovery failures.
    ///
    /// `now` uses the same seconds-based clock as [`Self::next_goal`].
    /// Supply a finite nonnegative time. Failures accumulate even if there is no
    /// active target, and [`Self::replan`] does not reset the failure count.
    pub fn invalidate_goal(&mut self, now: f64) {
        if let Some((x, y)) = self.goal.take() {
            if self.excluded.len() == 256 {
                self.excluded.remove(0);
            }
            self.excluded
                .push((x, y, now + terra_types::timing::SEARCH_BLACKLIST_SECONDS));
        }
        self.route.clear();
        self.anchor = None;
        self.failures = self.failures.saturating_add(1);
    }
    /// Reset the stall timer after at least 0.25 m of displacement from its anchor.
    ///
    /// Establishes an anchor on the first call. Supply a finite pose and a finite,
    /// nonnegative `now` on the same monotonic seconds-based clock as [`Self::next_goal`].
    /// This method does not validate its inputs; `next_goal` calls it after validation.
    pub fn record_progress(&mut self, pose: Pose, now: f64) {
        if self
            .anchor
            .is_none_or(|(p, _)| (p.x - pose.x).hypot(p.y - pose.y) >= 0.25)
        {
            self.anchor = Some((pose, now));
        }
    }
    fn merge(&mut self, m: &MapSnapshot, revision: u64) -> Result<(), &'static str> {
        if !m.valid() {
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
            if w as u64 * h as u64 > terra_mapping::MAX_PLANNING_CELLS as u64 {
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
                // Missing observations in a new window must not erase mission memory.
                if value < 0 {
                    continue;
                }
                let (px, py) = m.centre(x as usize, y as usize);
                if let Some((ix, iy)) = memory.cell(px, py) {
                    memory.occupancy[iy * memory.width as usize + ix] = value;
                }
            }
        }
        self.last_revision = Some(revision);
        Ok(())
    }
    /// Merge a local observation and return the next reachable search waypoint.
    ///
    /// Revisions must not decrease; `now` must be finite, nonnegative, and monotonic
    /// in seconds. A target is invalidated after 10 s without recorded progress,
    /// and three invalidations exhaust recovery for this navigator.
    ///
    /// # Decisions
    ///
    /// [`SearchNavigationDecision::Goal`] contains the next waypoint, which may
    /// precede the selected frontier. [`SearchNavigationDecision::Exhausted`]
    /// means no eligible frontier remains and no target cooldown is pending.
    /// A blocked decision has one of these reason codes:
    ///
    /// - `invalid_time_or_pose`: invalid pose/time, or time moved backward.
    /// - `invalid_map`: malformed map storage or geometry.
    /// - `map_revision_regressed`: revision decreased since the last merge.
    /// - `search_map_capacity`: mission memory would exceed 250,000 cells.
    /// - `map_frame_changed`: resolution differs from the first accepted map.
    /// - `outside_search_bounds`: the rover footprint extends outside mission bounds.
    /// - `search_recovery_exhausted`: at least three goal invalidations accumulated.
    /// - `frontier_cooldown`: no eligible frontier was found while exclusions remain.
    /// - `route_outside_local_map`: no usable first waypoint could be certified locally.
    ///
    /// The method can update time, map memory, and recovery state before returning
    /// a blocked decision; a blocked result does not roll those updates back.
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
        if !self.bounds.contains_body(&self.footprint, pose) {
            return SearchNavigationDecision::Blocked {
                reason: "outside_search_bounds",
            };
        }
        self.record_progress(pose, now);
        if self.goal.is_some()
            && self
                .anchor
                .is_some_and(|(_, t)| now - t >= terra_types::timing::SEARCH_STUCK_SECONDS)
        {
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
            } else if m.cell(x, y).is_some() && !reachable(m, pose, &self.footprint, x, y) {
                self.invalidate_goal(now);
            }
        }
        if !self.route.is_empty() {
            let (x, y) = self.route[0];
            return SearchNavigationDecision::Goal { x, y };
        }
        if let Some((x, y)) = self.goal.take() {
            self.excluded
                .push((x, y, now + terra_types::timing::SEARCH_BLACKLIST_SECONDS));
        }
        // Choose frontiers globally from memory; validate follower waypoints locally below.
        let memory = self.map.as_ref().unwrap();
        let excluded: Vec<_> = self.excluded.iter().map(|p| (p.0, p.1)).collect();
        let Some(target) = select_frontier(
            memory,
            pose,
            &self.footprint,
            &excluded,
            1.,
            &InformationGainScorer,
        )
        .map(|c| (c.x, c.y)) else {
            return if !excluded.is_empty() {
                SearchNavigationDecision::Blocked {
                    reason: "frontier_cooldown",
                }
            } else {
                SearchNavigationDecision::Exhausted
            };
        };
        // The follower routes to the frontier itself (Hybrid A* in the local map,
        // heading toward it when it lies beyond). Hand it over only if the body
        // can reach it through the remembered map.
        if !reachable(memory, pose, &self.footprint, target.0, target.1) {
            return SearchNavigationDecision::Blocked {
                reason: "route_outside_local_map",
            };
        }
        self.route = vec![target];
        self.goal = Some(target);
        if self.anchor.is_none() {
            self.anchor = Some((pose, now));
        }
        SearchNavigationDecision::Goal {
            x: target.0,
            y: target.1,
        }
    }
}
