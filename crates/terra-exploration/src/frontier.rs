use crate::{FrontierCandidate, GoalScorer};
use std::collections::VecDeque;
use terra_mapping::MapSnapshot;
use terra_navigation::{Pose, footprint_clear, grid_cell};
/// Observed free area in square metres, and the fraction of cells that are no longer unknown.
pub fn map_coverage(map: &MapSnapshot) -> Option<(f64, f64)> {
    let cells = map.width as usize * map.height as usize;
    if map.width == 0
        || map.height == 0
        || map.width > 1000
        || map.height > 1000
        || map.occupancy.len() != cells
        || !map.resolution.is_finite()
        || map.resolution < 0.01
        || !map.origin_x.is_finite()
        || !map.origin_y.is_finite()
    {
        return None;
    }
    let mut known = 0u32;
    let mut free = 0u32;
    for value in &map.occupancy {
        if *value >= 0 {
            known += 1;
            if *value < 65 {
                free += 1;
            }
        }
    }
    let area = free as f64 * map.resolution * map.resolution;
    Some((area, known as f64 / cells as f64))
}
/// Pick the highest-scoring reachable frontier. `None` means the known free space has no unknown neighbour.
///
/// Ordering matches `terra_navigation::frontier` when `scorer` is [`crate::InformationGainScorer`]
/// and `exclude_radius` is 1: right, left, up, down, and a strictly better score replaces the current best.
pub fn select_frontier(
    map: &MapSnapshot,
    pose: Pose,
    radius: f64,
    excluded: &[(f64, f64)],
    exclude_radius: f64,
    scorer: &dyn GoalScorer,
) -> Option<FrontierCandidate> {
    if !pose.finite() || !radius.is_finite() || radius <= 0. || radius > 5. {
        return None;
    }
    let exclude_radius = if exclude_radius.is_finite() && exclude_radius > 0. {
        exclude_radius
    } else {
        1.
    };
    let (start_x, start_y) = grid_cell(map, pose.x, pose.y)?;
    let mut seen = vec![false; map.occupancy.len()];
    let mut queue = VecDeque::from([(start_x, start_y, 0usize)]);
    let mut best: Option<FrontierCandidate> = None;
    while let Some((x, y, travel)) = queue.pop_front() {
        let index = y * map.width as usize + x;
        if seen[index] {
            continue;
        }
        seen[index] = true;
        let px = map.origin_x + (x as f64 + 0.5) * map.resolution;
        let py = map.origin_y + (y as f64 + 0.5) * map.resolution;
        if !footprint_clear(map, px, py, radius, Some(pose)) {
            continue;
        }
        let reach = (radius / map.resolution).ceil() as isize + 2;
        let mut gain = 0u32;
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
                    gain += 1;
                }
            }
        }
        let distance = (px - pose.x).hypot(py - pose.y);
        if footprint_clear(map, px, py, radius, None)
            && gain > 0
            && distance > radius.max(0.8)
            && excluded
                .iter()
                .all(|&(a, b)| (px - a).hypot(py - b) > exclude_radius)
        {
            let mut candidate = FrontierCandidate {
                x: px,
                y: py,
                information_gain: gain,
                travel_cells: travel,
                distance_m: distance,
                score: 0.,
            };
            let score = scorer.score(&candidate);
            if score.is_finite() && best.as_ref().is_none_or(|current| score > current.score) {
                candidate.score = score;
                best = Some(candidate);
            }
        }
        for (dx, dy) in [(1isize, 0isize), (-1, 0), (0, 1), (0, -1)] {
            let a = x as isize + dx;
            let b = y as isize + dy;
            if a >= 0 && b >= 0 && a < map.width as isize && b < map.height as isize {
                queue.push_back((a as usize, b as usize, travel + 1));
            }
        }
    }
    best
}
