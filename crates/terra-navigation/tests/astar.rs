use terra_mapping::MapSnapshot;
use terra_navigation::{Pose, astar_route, route_segment_clear};
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
