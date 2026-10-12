use std::time::Instant;
use terra_mapping::MapSnapshot;
use terra_navigation::{Pose, astar_progress_route, astar_route, route_segment_clear};
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

    unknown.occupancy[100 * 200 + 140] = 100;
    assert!(astar_progress_route(&unknown, pose, 0.65, 4.05, 0.05).is_err());
}

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
    // Same cell as the goal. The pose and the goal are each clear, but the
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

#[test]
fn routes_are_not_rejected_for_corner_count() {
    let width = 3usize;
    let height = 1_000usize;
    let mut occupancy = vec![100i8; width * height];
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
