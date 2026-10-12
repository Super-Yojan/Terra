//! Global routing over observed cells. Four-connected edges prevent diagonal corner cutting.
use crate::{Pose, cell, clear, valid_map};
use std::cmp::Reverse;
use std::collections::BinaryHeap;
use terra_mapping::MapSnapshot;

/// Check the swept circular footprint along a straight route segment.
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

/// A* with Manhattan heuristic and deterministic tie breaking. No unknown-space fallback.
/// Returned points are world metres, including the exact starting pose and destination.
pub fn astar_route(
    map: &MapSnapshot,
    pose: Pose,
    radius: f64,
    x: f64,
    y: f64,
) -> Result<Vec<[f64; 2]>, &'static str> {
    if !valid_map(map)
        || map.occupancy.len() > 100_000
        || !pose.finite()
        || !radius.is_finite()
        || radius <= 0.
        || radius > 5.
    {
        return Err("route_invalid_map");
    }
    let start = cell(map, pose.x, pose.y).ok_or("route_start_outside_map")?;
    let goal = cell(map, x, y).ok_or("route_goal_outside_map")?;
    if !clear(map, x, y, radius, None) {
        return Err("route_goal_blocked");
    }
    let width = map.width as usize;
    let index = |c: (usize, usize)| c.1 * width + c.0;
    let center = |i: usize| {
        [
            map.origin_x + (i % width) as f64 * map.resolution + map.resolution * 0.5,
            map.origin_y + (i / width) as f64 * map.resolution + map.resolution * 0.5,
        ]
    };
    let heuristic = |i: usize| (i % width).abs_diff(goal.0) + (i / width).abs_diff(goal.1);
    let mut costs = vec![usize::MAX; map.occupancy.len()];
    let mut previous = vec![usize::MAX; map.occupancy.len()];
    let mut open = BinaryHeap::new();
    let start_index = index(start);
    let goal_index = index(goal);
    costs[start_index] = 0;
    open.push(Reverse((heuristic(start_index), 0, start_index)));
    let mut traversable = vec![None; map.occupancy.len()];
    while let Some(Reverse((_, cost, current))) = open.pop() {
        if cost != costs[current] {
            continue;
        }
        if current == goal_index {
            let mut route = vec![[x, y]];
            let mut cursor = current;
            while cursor != start_index {
                route.push(center(cursor));
                cursor = previous[cursor];
            }
            route.push([pose.x, pose.y]);
            route.reverse();
            if route
                .windows(2)
                .all(|p| route_segment_clear(map, pose, radius, p[0], p[1]))
            {
                // Remove collinear cell centers without cutting across corners.
                let mut compact: Vec<[f64; 2]> = Vec::new();
                for point in route {
                    if compact.len() >= 2 {
                        let a = compact[compact.len() - 2];
                        let b = compact[compact.len() - 1];
                        if (a[0] == b[0] && b[0] == point[0]) || (a[1] == b[1] && b[1] == point[1])
                        {
                            compact.pop();
                        }
                    }
                    compact.push(point);
                }
                if compact.len() > 512 {
                    return Err("route_too_complex");
                }
                return Ok(compact);
            }
            return Err("route_no_path");
        }
        let from = if current == start_index {
            [pose.x, pose.y]
        } else {
            center(current)
        };
        for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
            let col = (current % width) as isize + dx;
            let row = (current / width) as isize + dy;
            if col < 0 || row < 0 || col >= map.width as isize || row >= map.height as isize {
                continue;
            }
            let next = row as usize * width + col as usize;
            if cost + 1 >= costs[next] {
                continue;
            }
            let to = center(next);
            let safe = *traversable[next]
                .get_or_insert_with(|| clear(map, to[0], to[1], radius, Some(pose)));
            if !safe || !route_segment_clear(map, pose, radius, from, to) {
                continue;
            }
            previous[next] = current;
            costs[next] = cost + 1;
            open.push(Reverse((cost + 1 + heuristic(next), cost + 1, next)));
        }
    }
    Err("route_no_path")
}
