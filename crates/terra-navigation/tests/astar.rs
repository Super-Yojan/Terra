//! Routing regressions for footprint clearance, partial progress, and search cost.

use std::time::Instant;
use terra_mapping::MapSnapshot;
use terra_navigation::{Pose, astar_progress_route, astar_route, route_segment_clear};
/// Build an observed, obstacle-free 7.5 m square with 0.25 m cells.
fn grid() -> MapSnapshot {
    MapSnapshot {
        width: 30,
        height: 30,
        resolution: 0.25,
        origin_x: 0.,
        origin_y: 0.,
        occupancy: vec![0; 900],
    }
}
/// A finite wall requires a detour that clears the rover's entire footprint.
/// Every returned edge must remain clear, and the route must still reach the goal.
#[test]
fn routes_around_wall_with_footprint_clearance() {
    let mut map = grid();
    for row in 0..20 {
        map.occupancy[row * 30 + 15] = 100;
    }
    let pose = Pose {
        x: 2.,
        y: 2.,
        yaw: 0.,
    };
    let route = astar_route(&map, pose, 0.3, 5., 2.).unwrap();
    assert!(route.iter().any(|p| p[1] > 5.));
    assert!(
        route
            .windows(2)
            .all(|p| route_segment_clear(&map, pose, 0.3, p[0], p[1]))
    );
    assert_eq!(route.last(), Some(&[5., 2.]));
}
/// Neither a wall spanning the map nor unobserved space may produce an exact route.
#[test]
fn sealed_and_unknown_routes_hold() {
    let mut map = grid();
    for row in 0..30 {
        map.occupancy[row * 30 + 15] = 100;
    }
    let pose = Pose {
        x: 2.,
        y: 2.,
        yaw: 0.,
    };
    assert!(astar_route(&map, pose, 0.3, 5., 2.).is_err());
    map.occupancy.fill(-1);
    assert!(astar_route(&map, pose, 0.3, 5., 2.).is_err());
}

/// Two diagonally touching free cells do not form a traversable passage through
/// the occupied cells surrounding their shared corner.
#[test]
fn diagonal_gap_does_not_cut_an_occupied_corner() {
    let map = MapSnapshot {
        width: 4,
        height: 4,
        resolution: 1.,
        origin_x: 0.,
        origin_y: 0.,
        occupancy: (0..16)
            .map(|i| if i == 5 || i == 10 { 0 } else { 100 })
            .collect(),
    };
    assert!(
        astar_route(
            &map,
            Pose {
                x: 1.5,
                y: 1.5,
                yaw: 0.
            },
            0.1,
            2.5,
            2.5
        )
        .is_err()
    );
}

/// Progress routing may approach off-map or unknown goals through observed space,
/// while exact routing rejects them and occupied goals remain errors.
#[test]
fn outside_and_unknown_goals_route_toward_the_frontier() {
    let map = MapSnapshot {
        width: 200,
        height: 200,
        resolution: 0.1,
        origin_x: -10.,
        origin_y: -10.,
        occupancy: vec![0; 40_000],
    };
    let pose = Pose {
        x: 0.,
        y: 0.,
        yaw: 0.,
    };
    assert_eq!(
        astar_route(&map, pose, 0.65, 16., 0.),
        Err("route_goal_outside_map")
    );
    // Report timing for this larger window without imposing a timing assertion here.
    let started = Instant::now();
    let route = astar_progress_route(&map, pose, 0.65, 16., 0.).unwrap();
    eprintln!(
        "20 m window progress route: {:.2} ms",
        started.elapsed().as_secs_f64() * 1000.
    );
    let end = route.last().copied().unwrap();
    assert!(
        (end[0] - 16.).hypot(end[1]) < 16.,
        "approach should reduce distance, ended at {end:?}"
    );
    assert_ne!(end, [16., 0.]);
    assert!(
        route
            .windows(2)
            .all(|p| route_segment_clear(&map, pose, 0.65, p[0], p[1]))
    );

    // Reveal only a local patch: progress must stop short of the unknown goal.
    let mut unknown = map.clone();
    unknown.occupancy.fill(-1);
    for row in 90..110 {
        for col in 90..120 {
            unknown.occupancy[row * 200 + col] = 0;
        }
    }
    let toward_unknown = astar_progress_route(&unknown, pose, 0.65, 5., 0.).unwrap();
    let end = toward_unknown.last().copied().unwrap();
    assert!((end[0] - 5.).hypot(end[1]) < 5.);
    assert_eq!(
        astar_route(&unknown, pose, 0.65, 5., 0.),
        Err("route_goal_blocked")
    );

    // An explicitly occupied target must not trigger the partial-progress fallback.
    unknown.occupancy[100 * 200 + 140] = 100;
    assert!(astar_progress_route(&unknown, pose, 0.65, 4.05, 0.05).is_err());
}

/// Clear endpoints do not guarantee a clear connecting segment. Retain the safe
/// prefix when a same-cell route grazes an inflated neighboring obstacle.
#[test]
fn clipped_final_segment_keeps_the_clear_prefix() {
    let mut map = MapSnapshot {
        width: 8,
        height: 8,
        resolution: 1.,
        origin_x: 0.,
        origin_y: 0.,
        occupancy: vec![0; 64],
    };
    // The pose is in the same cell as the goal. Both are clear, but the
    // straight segment between them crosses the inflated neighbour.
    map.occupancy[1 * 8] = 100;
    let pose = Pose {
        x: 1.45,
        y: 1.1,
        yaw: 0.,
    };
    let radius = 0.30;
    let goal = [1.45, 1.85];
    assert!(route_segment_clear(&map, pose, radius, [pose.x, pose.y], goal) == false);
    let route = astar_route(&map, pose, radius, goal[0], goal[1]).unwrap();
    assert!(route.len() >= 2, "{route:?}");
    assert_ne!(route.last(), Some(&goal));
    let end = route.last().unwrap();
    assert!(end[1] > pose.y && end[1] < goal[1], "{route:?}");
    assert!(
        route
            .windows(2)
            .all(|p| route_segment_clear(&map, pose, radius, p[0], p[1]))
    );
}

/// A long zigzag corridor must remain routable even when its compacted polyline
/// needs more than 512 points; corner count alone is not a reason to reject it.
#[test]
fn routes_are_not_rejected_for_corner_count() {
    let width = 3usize;
    let height = 1_000usize;
    let mut occupancy = vec![100i8; width * height];
    // Repeat a connected left-to-right-to-left pattern to force frequent turns.
    for row in 0..height {
        let free: &[usize] = match row % 6 {
            0 => &[0],
            1 => &[0, 1],
            2 => &[1, 2],
            3 => &[2],
            4 => &[2, 1],
            _ => &[1, 0],
        };
        for col in free {
            occupancy[row * width + col] = 0;
        }
    }
    let map = MapSnapshot {
        width: width as u32,
        height: height as u32,
        resolution: 1.,
        origin_x: 0.,
        origin_y: 0.,
        occupancy,
    };
    let route = astar_route(
        &map,
        Pose {
            x: 0.5,
            y: 0.5,
            yaw: 0.,
        },
        0.05,
        2.5,
        999.5,
    )
    .unwrap();
    assert!(route.len() > 512, "corners {}", route.len());
    assert_eq!(route.last(), Some(&[2.5, 999.5]));
}

/// Cross an observed 18 × 12 m map within the existing 30 ms timing budget,
/// guarding the clearance cache against regressions to repeated footprint scans.
#[test]
fn pitch_corner_to_corner_stays_on_the_control_budget() {
    let map = MapSnapshot {
        width: 180,
        height: 120,
        resolution: 0.1,
        origin_x: 0.,
        origin_y: 0.,
        occupancy: vec![0; 180 * 120],
    };
    let pose = Pose {
        x: 1.,
        y: 1.,
        yaw: 0.,
    };
    let started = Instant::now();
    let route = astar_route(&map, pose, 0.65, 17., 11.).unwrap();
    let elapsed_ms = started.elapsed().as_secs_f64() * 1000.;
    eprintln!(
        "18x12 corner-to-corner: {elapsed_ms:.2} ms, {} points",
        route.len()
    );
    assert_eq!(route.last(), Some(&[17., 11.]));
    // Release builds are several times faster; this guards a debug regression
    // back to the old per-edge footprint scan.
    assert!(elapsed_ms < 30., "{elapsed_ms:.2} ms");
}
