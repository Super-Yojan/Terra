//! Global routing over observed cells. Four-connected edges prevent diagonal corner cutting.
use crate::{Pose, cell, clear, footprint_rejection, valid_map};
use std::cmp::Reverse;
use std::collections::{BinaryHeap, VecDeque};
use terra_mapping::MapSnapshot;

/// Check the swept circular footprint along a straight route segment.
///
/// Endpoints use world metres. Samples are at most half a cell apart; unknown
/// cells are allowed only within the footprint at `pose`, as for local planning.
/// Invalid input or a segment requiring more than 10,000 steps returns `false`.
/// `radius` must be finite, positive, and at most 5 m. Both endpoints are checked,
/// including when `from == to`.
pub fn route_segment_clear(
    map: &MapSnapshot,
    pose: Pose,
    radius: f64,
    from: [f64; 2],
    to: [f64; 2],
) -> bool {
    if !valid_map(map)
        || !pose.finite()
        || !radius.is_finite()
        || !(0.0..=5.0).contains(&radius)
        || radius == 0.
        || !from.into_iter().chain(to).all(f64::is_finite)
    {
        return false;
    }
    let distance = (to[0] - from[0]).hypot(to[1] - from[1]);
    let steps = (distance / (map.resolution * 0.5)).ceil() as usize;
    if steps > 10_000 {
        return false;
    }
    (0..=steps).all(|i| {
        let fraction = i as f64 / steps.max(1) as f64;
        clear(
            map,
            from[0] + (to[0] - from[0]) * fraction,
            from[1] + (to[1] - from[1]) * fraction,
            radius,
            Some(pose),
        )
    })
}

/// A* to a certifiable goal. The goal cell must lie on the map and its footprint
/// must be known-free. A path whose final centimetres clip an obstacle is trimmed
/// to the last clear point instead of being discarded.
///
/// Returns a compact world-coordinate polyline beginning at `pose`; the final
/// point can fall short of `(x, y)` when the last segment needs trimming.
///
/// # Errors
///
/// - `route_invalid_map`: invalid map, nonfinite pose, radius outside `(0, 5]` m,
///   or more than 250,000 map cells.
/// - `route_start_outside_map`: the starting position is outside the map.
/// - `route_goal_outside_map`: the goal is outside the map or has nonfinite coordinates.
/// - `route_goal_blocked`: the goal footprint touches occupied, unknown, or off-map cells.
/// - `route_no_path`: no connected route exists or trimming leaves no usable segment.
///
/// # Examples
///
/// ```
/// use terra_mapping::MapSnapshot;
/// use terra_navigation::{astar_route, route_segment_clear, Pose};
///
/// let map = MapSnapshot {
///     width: 40, height: 40, resolution: 0.25,
///     origin_x: -5.0, origin_y: -5.0,
///     occupancy: vec![0; 40 * 40],
/// };
/// let pose = Pose::default();
/// let route = astar_route(&map, pose, 0.65, 2.0, 0.0)?;
/// assert_eq!(route.first(), Some(&[pose.x, pose.y]));
/// assert_eq!(route.last(), Some(&[2.0, 0.0]));
/// assert!(route.windows(2).all(|edge|
///     route_segment_clear(&map, pose, 0.65, edge[0], edge[1])
/// ));
/// # Ok::<(), &'static str>(())
/// ```
pub fn astar_route(
    map: &MapSnapshot,
    pose: Pose,
    radius: f64,
    x: f64,
    y: f64,
) -> Result<Vec<[f64; 2]>, &'static str> {
    let grid = FootprintGrid::build(map, pose, radius)?;
    exact_route(&grid, pose, x, y)
}

/// Extra cost of a step whose footprint is not yet known to be free. Routes
/// prefer observed space, but may cross unseen space when it is much shorter.
const UNKNOWN_STEP_COST: usize = 3;

/// Route toward an operator goal that may sit in space the rover has not seen,
/// such as behind an obstacle (issue #109).
///
/// 1. When the goal footprint is known free and connected, this is [`astar_route`].
/// 2. Otherwise the search treats unknown cells as passable at a higher cost,
///    finds the cheapest route to the goal, and returns the part of that route
///    that is known free. The rover drives to the edge of what it has seen, the
///    map fills in, and the caller replans. Only that known prefix is returned,
///    so every returned edge passes [`route_segment_clear`].
/// 3. A goal whose footprint touches an observed obstacle (for example just
///    behind a box) is moved to the nearest cell within `snap` metres whose
///    footprint clears every observed obstacle. Pass the follower's arrival
///    radius less a margin so that reaching the moved goal counts as arriving.
/// 4. A goal outside the map routes to the reachable cell nearest to it.
///
/// A successful route begins at `pose` and uses the same compact polyline format
/// as [`astar_route`]. Its endpoint may fall short of `(x, y)`, so success does
/// not imply that the requested goal was reached.
///
/// # Errors
///
/// Uses the error strings documented by [`astar_route`]:
/// - `route_goal_blocked`: the goal cell is occupied, or no cell within `snap`
///   of it fits the rover clear of observed obstacles.
/// - `route_no_path`: observed obstacles seal the goal off, even when unseen
///   space is allowed, or no known-free progress is possible from here.
/// - `route_goal_outside_map`: an off-map goal with no reachable cell closer
///   to it than the starting pose.
pub fn astar_progress_route(
    map: &MapSnapshot,
    pose: Pose,
    radius: f64,
    x: f64,
    y: f64,
    snap: f64,
) -> Result<Vec<[f64; 2]>, &'static str> {
    let grid = FootprintGrid::build(map, pose, radius)?;
    let goal = match exact_route(&grid, pose, x, y) {
        Ok(route) => return Ok(route),
        Err("route_goal_outside_map") => return progress_route(&grid, pose, x, y),
        Err("route_start_outside_map") => return Err("route_start_outside_map"),
        Err(_) => cell(map, x, y).ok_or("route_goal_outside_map")?,
    };
    if map.occupancy[goal.1 * map.width as usize + goal.0] >= 65 {
        return Err("route_goal_blocked");
    }
    // Treat unseen cells as free; observed obstacles and the map edge still block.
    let mut open_map = map.clone();
    for value in &mut open_map.occupancy {
        if *value < 0 {
            *value = 0;
        }
    }
    let open = FootprintGrid::build(&open_map, pose, radius)?;
    // The search enters the goal through its cell, so that cell must be open too.
    let target = if open.open(goal.1 * map.width as usize + goal.0)
        && footprint_rejection(&open_map, x, y, radius, None).is_none()
    {
        [x, y]
    } else {
        snap_goal(&open, x, y, snap).ok_or("route_goal_blocked")?
    };
    let route = weighted_route(&open, &grid, pose, target)?;
    match known_prefix(&grid, &route) {
        Some(prefix) => Ok(prefix),
        // Nothing known lies along the cheapest route (for example the rover
        // faces away from it); fall back to the nearest reachable progress.
        None => progress_route(&grid, pose, x, y).map_err(|_| "route_no_path"),
    }
}

/// Cached clearance for cell centers, with explicit segment checks near obstacles.
struct FootprintGrid<'a> {
    map: &'a MapSnapshot,
    pose: Pose,
    radius: f64,
    limit_sq: f64,
    slack_sq: f64,
    /// Squared cell-index distance from each cell center to the nearest blocked cell.
    dist_sq: Vec<u64>,
}

impl<'a> FootprintGrid<'a> {
    fn build(map: &'a MapSnapshot, pose: Pose, radius: f64) -> Result<Self, &'static str> {
        // Search maps can be up to the terra-navigation search capacity.
        if !valid_map(map)
            || map.occupancy.len() > 250_000
            || !pose.finite()
            || !radius.is_finite()
            || radius <= 0.
            || radius > 5.
        {
            return Err("route_invalid_map");
        }
        // Match the conservative cell-corner allowance in `footprint_rejection`.
        let reach = radius + map.resolution * 0.7072;
        let cells = map.occupancy.len();
        let width = map.width as usize;
        let height = map.height as usize;
        // A blocked border makes distance queries account for leaving the observed map.
        let pad = 1usize;
        let padded_width = width + pad * 2;
        let padded_height = height + pad * 2;
        let mut padded = vec![1u64 << 50; padded_width * padded_height];
        for row in 0..padded_height {
            for col in 0..padded_width {
                let inside = col >= pad && row >= pad && col < width + pad && row < height + pad;
                let blocked = if !inside {
                    true
                } else {
                    let index = (row - pad) * width + (col - pad);
                    cell_blocked(map, pose, reach, index)
                };
                if blocked {
                    padded[row * padded_width + col] = 0;
                }
            }
        }
        distance_transform(&mut padded, padded_width, padded_height);
        let mut dist_sq = vec![0u64; cells];
        for row in 0..height {
            for col in 0..width {
                dist_sq[row * width + col] = padded[(row + pad) * padded_width + (col + pad)];
            }
        }
        let cells_per_metre = 1.0 / map.resolution;
        let limit = reach * cells_per_metre;
        Ok(Self {
            map,
            pose,
            radius,
            limit_sq: limit * limit,
            slack_sq: (limit + 0.5) * (limit + 0.5),
            dist_sq,
        })
    }

    fn width(&self) -> usize {
        self.map.width as usize
    }

    fn center(&self, index: usize) -> [f64; 2] {
        let width = self.width();
        [
            self.map.origin_x
                + (index % width) as f64 * self.map.resolution
                + self.map.resolution * 0.5,
            self.map.origin_y
                + (index / width) as f64 * self.map.resolution
                + self.map.resolution * 0.5,
        ]
    }

    fn open(&self, index: usize) -> bool {
        self.dist_sq[index] as f64 > self.limit_sq
    }

    /// Adjacent cell-center edges are equivalent to `route_segment_clear`:
    /// distance is 1-Lipschitz, and samples fall on the endpoints and the midpoint.
    fn edge(&self, from: usize, to: usize, from_pose: bool) -> bool {
        if !self.open(to) {
            return false;
        }
        let to_point = self.center(to);
        // The initial pose need not coincide with its cell center.
        if from_pose {
            return route_segment_clear(
                self.map,
                self.pose,
                self.radius,
                [self.pose.x, self.pose.y],
                to_point,
            );
        }
        if !self.open(from) {
            return false;
        }
        if self.dist_sq[from] as f64 > self.slack_sq && self.dist_sq[to] as f64 > self.slack_sq {
            return true;
        }
        route_segment_clear(
            self.map,
            self.pose,
            self.radius,
            self.center(from),
            to_point,
        )
    }
}

fn cell_blocked(map: &MapSnapshot, pose: Pose, reach: f64, index: usize) -> bool {
    let occupancy = map.occupancy[index];
    if occupancy >= 65 {
        return true;
    }
    if occupancy < 0 {
        let width = map.width as usize;
        let x = map.origin_x + (index % width) as f64 * map.resolution + map.resolution * 0.5;
        let y = map.origin_y + (index / width) as f64 * map.resolution + map.resolution * 0.5;
        return (x - pose.x).hypot(y - pose.y) > reach;
    }
    false
}

fn exact_route(
    grid: &FootprintGrid<'_>,
    pose: Pose,
    x: f64,
    y: f64,
) -> Result<Vec<[f64; 2]>, &'static str> {
    let start = cell(grid.map, pose.x, pose.y).ok_or("route_start_outside_map")?;
    let goal = cell(grid.map, x, y).ok_or("route_goal_outside_map")?;
    if footprint_rejection(grid.map, x, y, grid.radius, None).is_some() {
        return Err("route_goal_blocked");
    }
    let width = grid.width();
    let index = |c: (usize, usize)| c.1 * width + c.0;
    let start_index = index(start);
    let goal_index = index(goal);
    if start_index == goal_index {
        return commit(grid, vec![[pose.x, pose.y], [x, y]]);
    }
    // Manhattan distance matches unit-cost, four-connected motion. Heap tuples also
    // break equal-priority ties by cost and row-major cell index.
    let heuristic = |i: usize| (i % width).abs_diff(goal.0) + (i / width).abs_diff(goal.1);
    let mut costs = vec![usize::MAX; grid.map.occupancy.len()];
    let mut previous = vec![usize::MAX; grid.map.occupancy.len()];
    let mut open = BinaryHeap::new();
    costs[start_index] = 0;
    open.push(Reverse((heuristic(start_index), 0usize, start_index)));
    while let Some(Reverse((_, cost, current))) = open.pop() {
        if cost != costs[current] {
            continue;
        }
        if current == goal_index {
            let mut route = vec![[x, y]];
            let mut cursor = current;
            while cursor != start_index {
                route.push(grid.center(cursor));
                let parent = previous[cursor];
                if parent == usize::MAX {
                    return Err("route_no_path");
                }
                cursor = parent;
            }
            route.push([pose.x, pose.y]);
            route.reverse();
            return commit(grid, route);
        }
        let from_pose = current == start_index;
        for (dx, dy) in [(1isize, 0isize), (-1, 0), (0, 1), (0, -1)] {
            let col = (current % width) as isize + dx;
            let row = (current / width) as isize + dy;
            if col < 0
                || row < 0
                || col >= grid.map.width as isize
                || row >= grid.map.height as isize
            {
                continue;
            }
            let next = row as usize * width + col as usize;
            if cost + 1 >= costs[next] || !grid.edge(current, next, from_pose) {
                continue;
            }
            previous[next] = current;
            costs[next] = cost + 1;
            open.push(Reverse((cost + 1 + heuristic(next), cost + 1, next)));
        }
    }
    Err("route_no_path")
}

/// Nearest cell center within `snap` metres of `(x, y)` whose footprint is open.
fn snap_goal(open: &FootprintGrid<'_>, x: f64, y: f64, snap: f64) -> Option<[f64; 2]> {
    if !snap.is_finite() || snap <= 0. {
        return None;
    }
    let map = open.map;
    let (column, row) = cell(map, x, y)?;
    let reach = (snap / map.resolution).ceil() as isize;
    let mut best: Option<([f64; 2], f64)> = None;
    for dy in -reach..=reach {
        for dx in -reach..=reach {
            let c = column as isize + dx;
            let r = row as isize + dy;
            if c < 0 || r < 0 || c >= map.width as isize || r >= map.height as isize {
                continue;
            }
            let index = r as usize * map.width as usize + c as usize;
            let center = open.center(index);
            let distance = (center[0] - x).hypot(center[1] - y);
            if distance <= snap && open.open(index) && best.is_none_or(|(_, d)| distance < d) {
                best = Some((center, distance));
            }
        }
    }
    best.map(|(center, _)| center)
}

/// A* over `open` (unseen space passable) where each step into a cell that is
/// not open in `known` costs [`UNKNOWN_STEP_COST`]. Ends exactly at `goal`.
fn weighted_route(
    open: &FootprintGrid<'_>,
    known: &FootprintGrid<'_>,
    pose: Pose,
    goal: [f64; 2],
) -> Result<Vec<[f64; 2]>, &'static str> {
    let start = cell(open.map, pose.x, pose.y).ok_or("route_start_outside_map")?;
    let target = cell(open.map, goal[0], goal[1]).ok_or("route_goal_outside_map")?;
    let width = open.width();
    let start_index = start.1 * width + start.0;
    let goal_index = target.1 * width + target.0;
    if start_index == goal_index {
        return commit(open, vec![[pose.x, pose.y], goal]);
    }
    let heuristic = |i: usize| (i % width).abs_diff(target.0) + (i / width).abs_diff(target.1);
    let mut costs = vec![usize::MAX; open.map.occupancy.len()];
    let mut previous = vec![usize::MAX; open.map.occupancy.len()];
    let mut heap = BinaryHeap::new();
    costs[start_index] = 0;
    heap.push(Reverse((heuristic(start_index), 0usize, start_index)));
    while let Some(Reverse((_, cost, current))) = heap.pop() {
        if cost != costs[current] {
            continue;
        }
        if current == goal_index {
            let mut route = vec![goal];
            let mut cursor = current;
            while cursor != start_index {
                route.push(open.center(cursor));
                cursor = previous[cursor];
                if cursor == usize::MAX {
                    return Err("route_no_path");
                }
            }
            route.push([pose.x, pose.y]);
            route.reverse();
            return commit(open, route);
        }
        let from_pose = current == start_index;
        for (dx, dy) in [(1isize, 0isize), (-1, 0), (0, 1), (0, -1)] {
            let col = (current % width) as isize + dx;
            let row = (current / width) as isize + dy;
            if col < 0
                || row < 0
                || col >= open.map.width as isize
                || row >= open.map.height as isize
            {
                continue;
            }
            let next = row as usize * width + col as usize;
            let step = if known.open(next) {
                1
            } else {
                UNKNOWN_STEP_COST
            };
            if cost + step >= costs[next] || !open.edge(current, next, from_pose) {
                continue;
            }
            previous[next] = current;
            costs[next] = cost + step;
            heap.push(Reverse((cost + step + heuristic(next), cost + step, next)));
        }
    }
    Err("route_no_path")
}

/// The leading part of `route` that is known free, cut at the last clear point
/// of the first segment that enters unseen space. `None` when that part is
/// shorter than one map cell.
fn known_prefix(known: &FootprintGrid<'_>, route: &[[f64; 2]]) -> Option<Vec<[f64; 2]>> {
    let mut prefix = vec![*route.first()?];
    for edge in route.windows(2) {
        if route_segment_clear(known.map, known.pose, known.radius, edge[0], edge[1]) {
            prefix.push(edge[1]);
            continue;
        }
        let cut = last_clear_point(known, edge[0], edge[1]);
        if (cut[0] - edge[0][0]).hypot(cut[1] - edge[0][1]) > 1e-3 {
            prefix.push(cut);
        }
        break;
    }
    let length: f64 = prefix
        .windows(2)
        .map(|p| (p[1][0] - p[0][0]).hypot(p[1][1] - p[0][1]))
        .sum();
    (prefix.len() >= 2 && length >= known.map.resolution).then_some(prefix)
}

/// Search all reachable cells for the closest center that improves on the current pose.
fn progress_route(
    grid: &FootprintGrid<'_>,
    pose: Pose,
    x: f64,
    y: f64,
) -> Result<Vec<[f64; 2]>, &'static str> {
    let start = cell(grid.map, pose.x, pose.y).ok_or("route_start_outside_map")?;
    let width = grid.width();
    let start_index = start.1 * width + start.0;
    let pose_distance = (pose.x - x).hypot(pose.y - y);
    let mut costs = vec![usize::MAX; grid.map.occupancy.len()];
    let mut previous = vec![usize::MAX; grid.map.occupancy.len()];
    let mut queue = VecDeque::new();
    costs[start_index] = 0;
    queue.push_back(start_index);
    let mut best: Option<usize> = None;
    let mut best_distance = f64::MAX;
    while let Some(current) = queue.pop_front() {
        let center = grid.center(current);
        let distance = (center[0] - x).hypot(center[1] - y);
        if grid.open(current) && distance + 1e-3 < pose_distance && distance < best_distance {
            best = Some(current);
            best_distance = distance;
        }
        let from_pose = current == start_index;
        for (dx, dy) in [(1isize, 0isize), (-1, 0), (0, 1), (0, -1)] {
            let col = (current % width) as isize + dx;
            let row = (current / width) as isize + dy;
            if col < 0
                || row < 0
                || col >= grid.map.width as isize
                || row >= grid.map.height as isize
            {
                continue;
            }
            let next = row as usize * width + col as usize;
            if costs[next] != usize::MAX || !grid.edge(current, next, from_pose) {
                continue;
            }
            costs[next] = costs[current] + 1;
            previous[next] = current;
            queue.push_back(next);
        }
    }
    let Some(best) = best else {
        return Err(if cell(grid.map, x, y).is_none() {
            "route_goal_outside_map"
        } else {
            "route_goal_blocked"
        });
    };
    let mut route = vec![grid.center(best)];
    let mut cursor = best;
    while cursor != start_index {
        cursor = previous[cursor];
        if cursor == usize::MAX {
            return Err("route_no_path");
        }
        if cursor != start_index {
            route.push(grid.center(cursor));
        }
    }
    route.push([pose.x, pose.y]);
    route.reverse();
    commit(grid, route)
}

/// Shorten a clipped final segment to its last clear sample. Earlier edges were
/// checked while searching, so a bad tail does not discard the path.
fn commit(
    grid: &FootprintGrid<'_>,
    mut route: Vec<[f64; 2]>,
) -> Result<Vec<[f64; 2]>, &'static str> {
    if route.len() < 2 {
        return Err("route_no_path");
    }
    let last = route.len() - 1;
    let previous = route[last - 1];
    let point = route[last];
    if !route_segment_clear(grid.map, grid.pose, grid.radius, previous, point) {
        let trimmed = last_clear_point(grid, previous, point);
        if (trimmed[0] - previous[0]).hypot(trimmed[1] - previous[1]) > 1e-3 {
            route[last] = trimmed;
        } else {
            route.pop();
        }
    }
    if route.len() < 2 {
        return Err("route_no_path");
    }
    // Remove redundant points on axis-aligned runs, retaining turns and endpoints.
    let mut compact: Vec<[f64; 2]> = Vec::new();
    for point in route {
        if compact.len() >= 2 {
            let a = compact[compact.len() - 2];
            let b = compact[compact.len() - 1];
            if (a[0] == b[0] && b[0] == point[0]) || (a[1] == b[1] && b[1] == point[1]) {
                compact.pop();
            }
        }
        compact.push(point);
    }
    Ok(compact)
}

fn last_clear_point(grid: &FootprintGrid<'_>, from: [f64; 2], to: [f64; 2]) -> [f64; 2] {
    let mut lo = 0.0;
    let mut hi = 1.0;
    for _ in 0..24 {
        let mid = (lo + hi) * 0.5;
        let point = [
            from[0] + (to[0] - from[0]) * mid,
            from[1] + (to[1] - from[1]) * mid,
        ];
        if route_segment_clear(grid.map, grid.pose, grid.radius, from, point) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    [
        from[0] + (to[0] - from[0]) * lo,
        from[1] + (to[1] - from[1]) * lo,
    ]
}

/// Squared Euclidean distance transform. `grid` values are 0 on blocked cells
/// and a large sentinel on free cells; they are replaced by squared index distance.
fn distance_transform(grid: &mut [u64], width: usize, height: usize) {
    let mut column = vec![0u64; height];
    let mut transformed = vec![0u64; height];
    for col in 0..width {
        for row in 0..height {
            column[row] = grid[row * width + col];
        }
        distance_transform_1d(&column, &mut transformed);
        for row in 0..height {
            grid[row * width + col] = transformed[row];
        }
    }
    let mut row_values = vec![0u64; width];
    let mut transformed = vec![0u64; width];
    for row in 0..height {
        let start = row * width;
        row_values.copy_from_slice(&grid[start..start + width]);
        distance_transform_1d(&row_values, &mut transformed);
        grid[start..start + width].copy_from_slice(&transformed);
    }
}

fn distance_transform_1d(values: &[u64], out: &mut [u64]) {
    // Each source index defines a parabola: values[i] + (q - i)^2. Maintain
    // their lower envelope, then evaluate it in one forward pass.
    let n = values.len();
    let mut vertices = vec![0usize; n];
    let mut bounds = vec![0.0; n + 1];
    let mut k = 0usize;
    bounds[0] = f64::NEG_INFINITY;
    bounds[1] = f64::INFINITY;
    for q in 1..n {
        let mut cursor = k;
        let mut intersection = parabola_intersection(values, vertices[cursor], q);
        while intersection <= bounds[cursor] {
            cursor -= 1;
            intersection = parabola_intersection(values, vertices[cursor], q);
        }
        k = cursor + 1;
        vertices[k] = q;
        bounds[k] = intersection;
        bounds[k + 1] = f64::INFINITY;
    }
    k = 0;
    for q in 0..n {
        while bounds[k + 1] < q as f64 {
            k += 1;
        }
        let delta = q as i64 - vertices[k] as i64;
        out[q] = (delta * delta) as u64 + values[vertices[k]];
    }
}

fn parabola_intersection(values: &[u64], index: usize, q: usize) -> f64 {
    let left = index as f64;
    let right = q as f64;
    ((values[q] as f64 + right * right) - (values[index] as f64 + left * left))
        / (2.0 * right - 2.0 * left)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Cached cell-center clearance must match the direct footprint check for
    /// occupied, unknown, and free cells, including the starting-footprint exception.
    #[test]
    fn cached_clearance_matches_the_footprint_test() {
        let mut map = MapSnapshot {
            width: 36,
            height: 24,
            resolution: 0.1,
            origin_x: -1.8,
            origin_y: -1.2,
            occupancy: vec![0; 36 * 24],
        };
        // A fixed seed makes the mixed occupancy fixture reproducible on every run.
        let mut seed = 9_u64;
        for cell in &mut map.occupancy {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            let roll = (seed >> 33) % 100;
            *cell = if roll < 8 {
                100
            } else if roll < 20 {
                -1
            } else {
                0
            };
        }
        let pose = Pose {
            x: 0.05,
            y: 0.05,
            yaw: 0.4,
        };
        let grid = FootprintGrid::build(&map, pose, 0.65).unwrap();
        for index in 0..map.occupancy.len() {
            let center = grid.center(index);
            assert_eq!(
                grid.open(index),
                clear(&map, center[0], center[1], 0.65, Some(pose)),
                "cell {index}"
            );
        }
    }
}
