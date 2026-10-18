//! Connected components, detector tracks, and frontier viewpoints.
//!
//! IDs are assigned after geometry is fixed: buckets `B`, people `P`, balls `N`,
//! frontiers `F`, other occupied components `O`, numbered by increasing x then y.

use terra_mapping::MapSnapshot;
use terra_navigation::{Pose, astar_route, frontier, reachable};

use crate::geom::{self, cell_center, cell_index};
use crate::scene::{CameraModel, FOOTPRINT_RADIUS, SceneObject};

#[derive(Clone, Copy, Debug)]
pub struct Detection {
    pub class: &'static str,
    pub x: f64,
    pub y: f64,
    pub confidence: f64,
    pub age_s: f64,
    pub carried: bool,
}

struct Component {
    centers: Vec<(f64, f64)>,
    x: f64,
    y: f64,
    min_x: f64,
    min_y: f64,
    max_x: f64,
    max_y: f64,
}

pub fn build_objects(
    map: &MapSnapshot,
    pose: Pose,
    detections: &[Detection],
) -> Result<Vec<SceneObject>, String> {
    let mut objects = Vec::new();
    let components = occupied_components(map);
    let mut used = vec![false; components.len()];
    let mut pending: Vec<&Detection> = detections.iter().filter(|det| !det.carried).collect();
    pending.sort_by(|a, b| {
        b.confidence
            .total_cmp(&a.confidence)
            .then(a.x.total_cmp(&b.x))
            .then(a.y.total_cmp(&b.y))
            .then(a.class.cmp(b.class))
    });
    for detection in pending {
        let mut best: Option<(usize, f64)> = None;
        for (index, component) in components.iter().enumerate() {
            if used[index] {
                continue;
            }
            let distance = component
                .centers
                .iter()
                .map(|(x, y)| (x - detection.x).hypot(y - detection.y))
                .fold(f64::MAX, f64::min);
            if distance <= 0.55 && best.is_none_or(|(_, best_distance)| distance < best_distance) {
                best = Some((index, distance));
            }
        }
        let Some((index, _)) = best else {
            return Err(format!(
                "detection {} at ({:.2}, {:.2}) matched no occupied component",
                detection.class, detection.x, detection.y
            ));
        };
        used[index] = true;
        let component = &components[index];
        objects.push(object_from_component(
            map,
            pose,
            component,
            detection.class,
            detection.confidence,
            detection.age_s,
            false,
            eligible_class(detection.class),
        ));
    }
    for detection in detections.iter().filter(|det| det.carried) {
        objects.push(SceneObject {
            id: String::new(),
            class: detection.class.to_string(),
            x: detection.x,
            y: detection.y,
            approach_x: detection.x,
            approach_y: detection.y,
            min_x: detection.x,
            min_y: detection.y,
            max_x: detection.x,
            max_y: detection.y,
            cell_count: 0,
            confidence: detection.confidence,
            age_s: detection.age_s,
            goal_eligible: false,
            reachable: false,
            carried: true,
        });
    }
    for (index, component) in components.iter().enumerate() {
        if used[index] {
            continue;
        }
        objects.push(object_from_component(
            map, pose, component, "obstacle", 1.0, 0.0, false, false,
        ));
    }
    objects.extend(frontier_objects(map, pose)?);
    assign_ids(&mut objects);
    Ok(objects)
}

fn eligible_class(class: &str) -> bool {
    class == "bucket"
}

fn object_from_component(
    map: &MapSnapshot,
    pose: Pose,
    component: &Component,
    class: &str,
    confidence: f64,
    age_s: f64,
    carried: bool,
    goal_eligible: bool,
) -> SceneObject {
    let approach = if goal_eligible {
        approach_point(map, pose, [component.x, component.y])
    } else {
        None
    };
    let (approach_x, approach_y, reachable_goal) = match approach {
        Some(point) => (tidy(point[0]), tidy(point[1]), true),
        None => (component.x, component.y, false),
    };
    let (x, y) = if class == "obstacle" {
        component
            .centers
            .iter()
            .max_by(|left, right| {
                let left_d = (left.0 - component.x).hypot(left.1 - component.y);
                let right_d = (right.0 - component.x).hypot(right.1 - component.y);
                left_d.total_cmp(&right_d)
            })
            .copied()
            .unwrap_or((component.x, component.y))
    } else {
        (component.x, component.y)
    };
    SceneObject {
        id: String::new(),
        class: class.to_string(),
        x,
        y,
        approach_x,
        approach_y,
        min_x: component.min_x,
        min_y: component.min_y,
        max_x: component.max_x,
        max_y: component.max_y,
        cell_count: component.centers.len() as u32,
        confidence,
        age_s,
        goal_eligible,
        reachable: reachable_goal,
        carried,
    }
}

fn approach_point(map: &MapSnapshot, pose: Pose, target: [f64; 2]) -> Option<[f64; 2]> {
    let mut candidates = Vec::new();
    for (ring_index, ring) in [1.05_f64, 1.2, 1.35].into_iter().enumerate() {
        for step in 0..12 {
            let angle = step as f64 * std::f64::consts::TAU / 12.0;
            let point = [
                target[0] + ring * angle.cos(),
                target[1] + ring * angle.sin(),
            ];
            if geom::occupancy_at(map, point[0], point[1])
                .is_some_and(|value| (0..65).contains(&value))
            {
                candidates.push((ring_index, step, point));
            }
        }
    }
    candidates.sort_by(|a, b| {
        let da = (a.2[0] - pose.x).hypot(a.2[1] - pose.y);
        let db = (b.2[0] - pose.x).hypot(b.2[1] - pose.y);
        da.total_cmp(&db).then(a.0.cmp(&b.0)).then(a.1.cmp(&b.1))
    });
    candidates.into_iter().find_map(|(_, _, point)| {
        if !reachable(map, pose, FOOTPRINT_RADIUS, point[0], point[1]) {
            return None;
        }
        astar_route(map, pose, FOOTPRINT_RADIUS, point[0], point[1])
            .ok()
            .map(|_| point)
    })
}

fn occupied_components(map: &MapSnapshot) -> Vec<Component> {
    let mut seen = vec![false; map.occupancy.len()];
    let mut seeds = Vec::new();
    for (index, value) in map.occupancy.iter().enumerate() {
        if *value >= 65 {
            let col = (index % map.width as usize) as i32;
            let row = (index / map.width as usize) as i32;
            seeds.push((col, row));
        }
    }
    seeds.sort();
    let mut components = Vec::new();
    for (col, row) in seeds {
        let start = cell_index(map, col, row).expect("seed");
        if seen[start] {
            continue;
        }
        let mut stack = vec![(col, row)];
        seen[start] = true;
        let mut centers = Vec::new();
        while let Some((cx, cy)) = stack.pop() {
            let (x, y) = cell_center(map, cx, cy);
            centers.push((x, y));
            for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
                let nx = cx + dx;
                let ny = cy + dy;
                let Some(index) = cell_index(map, nx, ny) else {
                    continue;
                };
                if seen[index] || map.occupancy[index] < 65 {
                    continue;
                }
                seen[index] = true;
                stack.push((nx, ny));
            }
        }
        if centers.is_empty() {
            continue;
        }
        let n = centers.len() as f64;
        let x = centers.iter().map(|p| p.0).sum::<f64>() / n;
        let y = centers.iter().map(|p| p.1).sum::<f64>() / n;
        let half = map.resolution * 0.5;
        components.push(Component {
            min_x: centers.iter().map(|p| p.0).fold(f64::MAX, f64::min) - half,
            min_y: centers.iter().map(|p| p.1).fold(f64::MAX, f64::min) - half,
            max_x: centers.iter().map(|p| p.0).fold(f64::MIN, f64::max) + half,
            max_y: centers.iter().map(|p| p.1).fold(f64::MIN, f64::max) + half,
            x,
            y,
            centers,
        });
    }
    components
}

fn frontier_objects(map: &MapSnapshot, pose: Pose) -> Result<Vec<SceneObject>, String> {
    let cells = frontier_cells(map);
    let clusters = cluster_cells(&cells);
    let verified = verified_frontier_viewpoint(map, pose);
    let mut objects = Vec::new();
    for cluster in clusters {
        if cluster.len() < 8 {
            continue;
        }
        let half = map.resolution * 0.5;
        let min_x = cluster.iter().map(|p| p.0).fold(f64::MAX, f64::min) - half;
        let min_y = cluster.iter().map(|p| p.1).fold(f64::MAX, f64::min) - half;
        let max_x = cluster.iter().map(|p| p.0).fold(f64::MIN, f64::max) + half;
        let max_y = cluster.iter().map(|p| p.1).fold(f64::MIN, f64::max) + half;
        let span = {
            let cx = cluster.iter().map(|p| p.0).sum::<f64>() / cluster.len() as f64;
            let cy = cluster.iter().map(|p| p.1).sum::<f64>() / cluster.len() as f64;
            cluster
                .iter()
                .map(|p| (p.0 - cx).hypot(p.1 - cy))
                .fold(0.0_f64, f64::max)
        };
        let approach = verified.and_then(|point| {
            let close = cluster
                .iter()
                .any(|cell| (cell.0 - point[0]).hypot(cell.1 - point[1]) <= span.max(1.5));
            close.then_some(point)
        });
        let Some(approach) = approach.or_else(|| local_viewpoint(map, pose, &cluster)) else {
            continue;
        };
        let (glyph_x, glyph_y) = cluster
            .iter()
            .min_by(|a, b| {
                let da = (a.0 - approach[0]).hypot(a.1 - approach[1]);
                let db = (b.0 - approach[0]).hypot(b.1 - approach[1]);
                da.total_cmp(&db)
            })
            .copied()
            .unwrap_or((approach[0], approach[1]));
        objects.push(SceneObject {
            id: String::new(),
            class: "frontier".to_string(),
            x: glyph_x,
            y: glyph_y,
            approach_x: tidy(approach[0]),
            approach_y: tidy(approach[1]),
            min_x,
            min_y,
            max_x,
            max_y,
            cell_count: cluster.len() as u32,
            confidence: 1.0,
            age_s: 0.0,
            goal_eligible: true,
            reachable: true,
            carried: false,
        });
    }
    // One verified viewpoint must survive even when clustering attaches it poorly.
    if let Some(point) = verified {
        let listed = objects
            .iter()
            .any(|object| (object.approach_x - point[0]).hypot(object.approach_y - point[1]) < 0.2);
        if !listed {
            objects.push(SceneObject {
                id: String::new(),
                class: "frontier".to_string(),
                x: point[0],
                y: point[1],
                approach_x: tidy(point[0]),
                approach_y: tidy(point[1]),
                min_x: point[0],
                min_y: point[1],
                max_x: point[0],
                max_y: point[1],
                cell_count: 1,
                confidence: 1.0,
                age_s: 0.0,
                goal_eligible: true,
                reachable: true,
                carried: false,
            });
        }
    }
    Ok(objects)
}

pub fn verified_frontier_viewpoint(map: &MapSnapshot, pose: Pose) -> Option<[f64; 2]> {
    // Prefer a viewpoint ahead of the rover so the frontier sits in the camera wedge.
    let mut x = pose.x + 0.8;
    while x < pose.x + 8.5 {
        let point = [tidy(x), tidy(pose.y)];
        if geom::near_unknown(map, point[0], point[1], 1.0)
            && reachable(map, pose, FOOTPRINT_RADIUS, point[0], point[1])
            && astar_route(map, pose, FOOTPRINT_RADIUS, point[0], point[1]).is_ok()
        {
            return Some(point);
        }
        x += 0.1;
    }
    if let Some((x, y)) = frontier(map, pose, FOOTPRINT_RADIUS, &[]) {
        if astar_route(map, pose, FOOTPRINT_RADIUS, x, y).is_ok() {
            return Some([tidy(x), tidy(y)]);
        }
    }
    None
}

fn tidy(value: f64) -> f64 {
    let rounded = (value * 1_000.0).round() / 1_000.0;
    if rounded.abs() < 5e-4 { 0.0 } else { rounded }
}

fn local_viewpoint(map: &MapSnapshot, pose: Pose, cluster: &[(f64, f64)]) -> Option<[f64; 2]> {
    let mut sx = 0.0;
    let mut sy = 0.0;
    let mut n = 0.0;
    for cell in cluster {
        for (dx, dy) in [(0.1, 0.0), (-0.1, 0.0), (0.0, 0.1), (0.0, -0.1)] {
            let x = cell.0 + dx;
            let y = cell.1 + dy;
            if geom::occupancy_at(map, x, y).is_some_and(|value| (0..65).contains(&value)) {
                sx += x;
                sy += y;
                n += 1.0;
            }
        }
    }
    if n == 0.0 {
        return None;
    }
    let mean = [sx / n, sy / n];
    let cx = cluster.iter().map(|p| p.0).sum::<f64>() / cluster.len() as f64;
    let cy = cluster.iter().map(|p| p.1).sum::<f64>() / cluster.len() as f64;
    let mut dir_x = mean[0] - cx;
    let mut dir_y = mean[1] - cy;
    let norm = dir_x.hypot(dir_y);
    if norm < 1e-6 {
        return None;
    }
    dir_x /= norm;
    dir_y /= norm;
    for extra in [0.15_f64, 0.4, 0.7, 1.0] {
        let point = [mean[0] + dir_x * extra, mean[1] + dir_y * extra];
        if reachable(map, pose, FOOTPRINT_RADIUS, point[0], point[1])
            && astar_route(map, pose, FOOTPRINT_RADIUS, point[0], point[1]).is_ok()
        {
            return Some(point);
        }
    }
    None
}

fn frontier_cells(map: &MapSnapshot) -> Vec<(f64, f64)> {
    let mut cells = Vec::new();
    for row in 0..map.height as i32 {
        for col in 0..map.width as i32 {
            let index = cell_index(map, col, row).expect("in range");
            if map.occupancy[index] >= 0 {
                continue;
            }
            let touches_free = [(1, 0), (-1, 0), (0, 1), (0, -1)]
                .into_iter()
                .any(|(dx, dy)| {
                    cell_index(map, col + dx, row + dy)
                        .is_some_and(|neighbor| (0..65).contains(&map.occupancy[neighbor]))
                });
            if touches_free {
                cells.push(cell_center(map, col, row));
            }
        }
    }
    cells.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)));
    cells
}

fn cluster_cells(cells: &[(f64, f64)]) -> Vec<Vec<(f64, f64)>> {
    let mut seen = vec![false; cells.len()];
    let mut clusters = Vec::new();
    for start in 0..cells.len() {
        if seen[start] {
            continue;
        }
        seen[start] = true;
        let mut stack = vec![start];
        let mut group = Vec::new();
        while let Some(index) = stack.pop() {
            group.push(cells[index]);
            for (other, center) in cells.iter().enumerate() {
                if seen[other] {
                    continue;
                }
                if (center.0 - cells[index].0).hypot(center.1 - cells[index].1) <= 0.16 {
                    seen[other] = true;
                    stack.push(other);
                }
            }
        }
        group.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)));
        clusters.push(group);
    }
    clusters.sort_by(|a, b| a[0].0.total_cmp(&b[0].0).then(a[0].1.total_cmp(&b[0].1)));
    clusters
}

fn assign_ids(objects: &mut [SceneObject]) {
    for (class, prefix) in [
        ("bucket", "B"),
        ("person", "P"),
        ("ball", "N"),
        ("frontier", "F"),
        ("obstacle", "O"),
    ] {
        let mut indexes: Vec<usize> = objects
            .iter()
            .enumerate()
            .filter(|(_, object)| object.class == class)
            .map(|(index, _)| index)
            .collect();
        indexes.sort_by(|&a, &b| {
            objects[a]
                .x
                .total_cmp(&objects[b].x)
                .then(objects[a].y.total_cmp(&objects[b].y))
                .then(objects[a].approach_x.total_cmp(&objects[b].approach_x))
        });
        for (number, index) in indexes.into_iter().enumerate() {
            objects[index].id = format!("{prefix}{}", number + 1);
        }
    }
}

pub fn object_by_id<'a>(objects: &'a [SceneObject], id: &str) -> Option<&'a SceneObject> {
    objects.iter().find(|object| object.id == id)
}

pub fn in_camera_wedge(
    camera_x: f64,
    camera_y: f64,
    yaw: f64,
    camera: &CameraModel,
    x: f64,
    y: f64,
) -> bool {
    let dx = x - camera_x;
    let dy = y - camera_y;
    let distance = dx.hypot(dy);
    if distance < 0.2 || distance > camera.range_m {
        return false;
    }
    let mut delta = dy.atan2(dx) - yaw;
    while delta > std::f64::consts::PI {
        delta -= std::f64::consts::TAU;
    }
    while delta < -std::f64::consts::PI {
        delta += std::f64::consts::TAU;
    }
    delta.abs() <= camera.half_fov() && dx * yaw.cos() + dy * yaw.sin() > 0.0
}
