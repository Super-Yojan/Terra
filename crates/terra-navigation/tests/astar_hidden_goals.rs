//! Issue #109: L3 goals placed behind a mapped obstacle. The space behind an
//! obstacle is unseen, so these maps copy what the phone builds from one view:
//! the obstacle's front face is occupied, the shadow behind it is unknown, and
//! everything else in the camera's view is known free.

use terra_mapping::MapSnapshot;
use terra_navigation::{Pose, astar_progress_route, route_segment_clear};

/// Terra's circle planner radius (0.61 m square body plus clearance).
const RADIUS: f64 = 0.65;
/// L3 arrival radius minus a margin, as the arbiter passes it.
const SNAP: f64 = 0.65;

/// 20 m window of 0.1 m cells centred on the rover, as on the phone.
fn view_with_box(face_x: f64, half_width: f64) -> MapSnapshot {
    let mut map = MapSnapshot {
        width: 200,
        height: 200,
        resolution: 0.1,
        origin_x: -10.,
        origin_y: -10.,
        occupancy: vec![-1; 40_000],
    };
    for row in 0..200 {
        for col in 0..200 {
            let x = map.origin_x + (col as f64 + 0.5) * 0.1;
            let y = map.origin_y + (row as f64 + 0.5) * 0.1;
            let i = row * 200 + col;
            // Seen: the half-disc ahead of the rover out to 6 m, plus its own patch.
            let seen = (x > -1. && x.hypot(y) < 6.) || x.hypot(y) < 1.;
            if !seen {
                continue;
            }
            // The box face, 0.2 m deep.
            if (face_x..face_x + 0.2).contains(&x) && y.abs() <= half_width {
                map.occupancy[i] = 100;
                continue;
            }
            // Shadow cast by the box from the camera at the origin.
            let shadow = x >= face_x + 0.2 && y.abs() <= half_width * x / face_x;
            map.occupancy[i] = if shadow { -1 } else { 0 };
        }
    }
    map
}

fn rover() -> Pose {
    Pose {
        x: 0.,
        y: 0.,
        yaw: 0.,
    }
}

fn assert_safe(map: &MapSnapshot, route: &[[f64; 2]]) {
    assert_safe_from(map, rover(), route);
}

fn assert_safe_from(map: &MapSnapshot, pose: Pose, route: &[[f64; 2]]) {
    assert!(route.len() >= 2, "{route:?}");
    assert_eq!(route[0], [pose.x, pose.y]);
    assert!(
        route
            .windows(2)
            .all(|p| route_segment_clear(map, pose, RADIUS, p[0], p[1])),
        "route leaves known-free space: {route:?}"
    );
}

/// V6: a goal just behind the box. Its footprint touches the box face and the
/// unseen shadow, so the old planner refused it with `route_goal_blocked`.
#[test]
fn goal_just_behind_a_box_routes_around_it() {
    let map = view_with_box(2.0, 0.6);
    let goal = [2.6, 0.0];
    let route = astar_progress_route(&map, rover(), RADIUS, goal[0], goal[1], SNAP)
        .expect("goal behind a box must give a route");
    assert_safe(&map, &route);
    // The route goes around the side of the box, not into its face.
    let end = *route.last().unwrap();
    assert!(
        end[1].abs() > 0.6 + RADIUS,
        "should head round a side: {route:?}"
    );
}

/// V6, second half: once the rover has driven up to the box face, no cell is
/// nearer the goal than where it stands, so the old planner held with
/// `route_goal_blocked` and the rover never moved again.
#[test]
fn rover_at_the_box_face_still_routes_around() {
    let map = view_with_box(2.0, 0.6);
    let at_face = Pose {
        x: 1.25,
        y: 0.,
        yaw: 0.,
    };
    let route = astar_progress_route(&map, at_face, RADIUS, 2.6, 0.0, SNAP)
        .expect("rover parked at the box face must still get a route");
    assert_safe_from(&map, at_face, &route);
    let end = *route.last().unwrap();
    assert!(end[1].abs() > 0.6, "should head round a side: {route:?}");
}

/// A goal further back in the shadow, clear of the box itself.
#[test]
fn goal_deep_in_the_shadow_routes_around_the_box() {
    let map = view_with_box(2.0, 0.6);
    let route = astar_progress_route(&map, rover(), RADIUS, 4.0, 0.0, SNAP)
        .expect("goal in the shadow must give a route");
    assert_safe(&map, &route);
    let end = *route.last().unwrap();
    assert!(
        end[1].abs() > 0.6 + RADIUS,
        "should head round a side: {route:?}"
    );
}

/// A goal on the box itself is still refused.
#[test]
fn goal_on_the_box_is_refused() {
    let map = view_with_box(2.0, 0.6);
    assert_eq!(
        astar_progress_route(&map, rover(), RADIUS, 2.1, 0.0, SNAP),
        Err("route_goal_blocked")
    );
}

/// A known-free goal that can only be reached through unseen space gets a
/// route to the edge of what is known, instead of `route_no_path`.
#[test]
fn known_goal_reached_through_unknown_space() {
    // Known free strip around the rover and known free room at x 4..8, with an
    // unseen 1 m band between them.
    let mut map = MapSnapshot {
        width: 200,
        height: 200,
        resolution: 0.1,
        origin_x: -10.,
        origin_y: -10.,
        occupancy: vec![0; 40_000],
    };
    for row in 0..200 {
        for col in 0..200 {
            let x = map.origin_x + (col as f64 + 0.5) * 0.1;
            if (3.0..4.0).contains(&x) {
                map.occupancy[row * 200 + col] = -1;
            }
        }
    }
    let route = astar_progress_route(&map, rover(), RADIUS, 6.0, 0.0, SNAP)
        .expect("known goal through unseen space must give a route");
    assert_safe(&map, &route);
    let end = *route.last().unwrap();
    assert!(
        end[0] > 2.0,
        "should drive up to the unseen band: {route:?}"
    );
}

/// A known goal walled in on every side (all walls seen) is refused.
#[test]
fn enclosed_goal_is_refused() {
    let mut map = MapSnapshot {
        width: 200,
        height: 200,
        resolution: 0.1,
        origin_x: -10.,
        origin_y: -10.,
        occupancy: vec![-1; 40_000],
    };
    for row in 0..200 {
        for col in 0..200 {
            let x = map.origin_x + (col as f64 + 0.5) * 0.1;
            let y = map.origin_y + (row as f64 + 0.5) * 0.1;
            let i = row * 200 + col;
            if x.hypot(y) < 2. {
                map.occupancy[i] = 0;
            }
            // A closed 3 m square room at x 4..7, y -1.5..1.5, wall 0.2 m.
            let inside = (4.0..7.0).contains(&x) && (-1.5..1.5).contains(&y);
            let wall = (3.8..7.2).contains(&x) && (-1.7..1.7).contains(&y) && !inside;
            if wall {
                map.occupancy[i] = 100;
            } else if inside {
                map.occupancy[i] = 0;
            }
        }
    }
    let result = astar_progress_route(&map, rover(), RADIUS, 5.5, 0.0, SNAP);
    assert_eq!(result, Err("route_no_path"), "{result:?}");
}

/// Review of #109: a goal in known-free space whose footprint touches the box
/// (seen from behind earlier). The old code returned a plain
/// `route_goal_blocked` with no fallback; now the goal moves to where the
/// rover fits, within the arrival radius, and the route reaches it.
#[test]
fn known_free_goal_touching_the_box_snaps_to_where_the_rover_fits() {
    let mut map = MapSnapshot {
        width: 200,
        height: 200,
        resolution: 0.1,
        origin_x: -10.,
        origin_y: -10.,
        occupancy: vec![0; 40_000],
    };
    for row in 0..200 {
        for col in 0..200 {
            let x = map.origin_x + (col as f64 + 0.5) * 0.1;
            let y = map.origin_y + (row as f64 + 0.5) * 0.1;
            if (2.0..2.2).contains(&x) && y.abs() <= 0.6 {
                map.occupancy[row * 200 + col] = 100;
            }
        }
    }
    let goal = [2.5, 0.0];
    let route = astar_progress_route(&map, rover(), RADIUS, goal[0], goal[1], SNAP)
        .expect("known-free goal near the box must give a route");
    assert_safe(&map, &route);
    let end = *route.last().unwrap();
    let miss = (end[0] - goal[0]).hypot(end[1] - goal[1]);
    assert!(miss <= SNAP, "end {end:?} is {miss:.2} m from the goal");
    assert!(end[0] > 2.2, "end should be behind the box: {route:?}");
}
