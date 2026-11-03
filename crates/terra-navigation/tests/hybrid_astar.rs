//! Hybrid A*: drivable routes for a rectangular rover that never turns in place.
use std::f64::consts::PI;
use terra_mapping::MapSnapshot;
use terra_navigation::{
    HybridConfig, HybridGoal, HybridWaypoint, Pose, astar_route, hybrid_astar_route,
};

const RES: f64 = 0.1;

/// Free 8 m × 6 m map centred on the origin.
fn open_map() -> MapSnapshot {
    MapSnapshot {
        width: 80,
        height: 60,
        resolution: RES,
        origin_x: -4.,
        origin_y: -3.,
        occupancy: vec![0; 80 * 60],
    }
}

/// A 0.2 m thick wall across the map at x = 0..0.2, open only where |y| < gap / 2.
fn wall_with_gap(gap: f64) -> MapSnapshot {
    let mut map = open_map();
    for row in 0..map.height as usize {
        let y = map.origin_y + (row as f64 + 0.5) * RES;
        if y.abs() < gap / 2. {
            continue;
        }
        for column in 40..42 {
            map.occupancy[row * map.width as usize + column] = 100;
        }
    }
    map
}

/// Independent check: dense points over the body rectangle all land on free cells.
fn body_fits(map: &MapSnapshot, p: &HybridWaypoint, config: &HybridConfig) -> bool {
    let (a, b) = (config.footprint.length / 2., config.footprint.width / 2.);
    let (sin, cos) = p.yaw.sin_cos();
    let steps = 40;
    (0..=steps).all(|i| {
        (0..=steps).all(|j| {
            let u = -a + 2. * a * i as f64 / steps as f64;
            let v = -b + 2. * b * j as f64 / steps as f64;
            let (x, y) = (p.x + u * cos - v * sin, p.y + u * sin + v * cos);
            let column = ((x - map.origin_x) / map.resolution).floor();
            let row = ((y - map.origin_y) / map.resolution).floor();
            column >= 0.
                && row >= 0.
                && column < map.width as f64
                && row < map.height as f64
                && (0..65)
                    .contains(&map.occupancy[row as usize * map.width as usize + column as usize])
        })
    })
}

/// The route never turns tighter than the minimum radius, and the body always fits.
fn assert_drivable(map: &MapSnapshot, route: &[HybridWaypoint], config: &HybridConfig) {
    let kappa = 1. / config.min_turn_radius;
    for p in route {
        assert!(
            p.curvature.abs() <= kappa + 1e-9,
            "curvature {}",
            p.curvature
        );
        assert!(body_fits(map, p, config), "body collides at {p:?}");
    }
    for pair in route.windows(2) {
        let distance = (pair[1].x - pair[0].x).hypot(pair[1].y - pair[0].y);
        assert!(distance > 1e-6, "no turning in place: {pair:?}");
        assert!(distance <= RES + 1e-9, "samples at most one cell apart");
        let turn = (pair[1].yaw - pair[0].yaw + PI).rem_euclid(2. * PI) - PI;
        // On a circle of curvature kappa, a chord of length d spans a turn of 2·asin(kappa·d/2).
        let max_turn = 2. * (kappa * distance / 2.).min(1.).asin();
        assert!(
            turn.abs() <= max_turn + 1e-6,
            "turn {turn} over chord {distance}"
        );
    }
}

fn arrives(route: &[HybridWaypoint], x: f64, y: f64, config: &HybridConfig) {
    let end = route.last().unwrap();
    assert!(
        (end.x - x).hypot(end.y - y) <= config.goal_tolerance,
        "ends at {end:?}"
    );
}

#[test]
fn drives_straight_across_an_open_map() {
    let map = open_map();
    let config = HybridConfig::default();
    let start = Pose {
        x: -3.,
        y: 0.,
        yaw: 0.,
    };
    let goal = HybridGoal {
        x: 3.,
        y: 0.,
        yaw: None,
    };
    let route = hybrid_astar_route(&map, start, goal, &config).unwrap();
    assert_drivable(&map, &route, &config);
    arrives(&route, 3., 0., &config);
    assert!(route.iter().all(|p| !p.reverse));
    let length: f64 = route
        .windows(2)
        .map(|w| (w[1].x - w[0].x).hypot(w[1].y - w[0].y))
        .sum();
    assert!(length < 6.2, "length {length}");
}

#[test]
fn lines_up_to_pass_a_gap_the_circle_planner_rejects() {
    // 0.85 m: the 0.61 m body plus margins fits only within about 8 degrees of
    // square to the gap, and the 1.3 m circle never fits.
    let map = wall_with_gap(0.85);
    let config = HybridConfig::default();
    let start = Pose {
        x: -3.,
        y: 1.5,
        yaw: 0.,
    };
    assert!(astar_route(&map, start, 0.65, 3., 0.).is_err());

    let goal = HybridGoal {
        x: 3.,
        y: 0.,
        yaw: None,
    };
    let route = hybrid_astar_route(&map, start, goal, &config).unwrap();
    assert_drivable(&map, &route, &config);
    arrives(&route, 3., 0., &config);
    let in_gap: Vec<_> = route
        .iter()
        .filter(|p| (-0.1..0.3).contains(&p.x))
        .collect();
    assert!(!in_gap.is_empty());
    for p in in_gap {
        let off_axis = (p.yaw + PI / 2.).rem_euclid(PI / 2.) - PI / 4.;
        assert!(
            (PI / 4. - off_axis.abs()) < 0.16,
            "square to the gap: {p:?}"
        );
    }
}

#[test]
fn arrives_facing_the_requested_heading() {
    let map = open_map();
    let config = HybridConfig::default();
    let start = Pose {
        x: -2.,
        y: -1.,
        yaw: 0.,
    };
    let goal = HybridGoal {
        x: 2.,
        y: 1.5,
        yaw: Some(PI / 2.),
    };
    let route = hybrid_astar_route(&map, start, goal, &config).unwrap();
    assert_drivable(&map, &route, &config);
    arrives(&route, 2., 1.5, &config);
    let end = route.last().unwrap();
    assert!(
        (end.yaw - PI / 2.).abs() <= config.goal_heading_tolerance,
        "{end:?}"
    );
}

#[test]
fn forward_only_loops_round_to_a_goal_behind() {
    let map = open_map();
    let config = HybridConfig {
        allow_reverse: false,
        ..HybridConfig::default()
    };
    let start = Pose {
        x: 0.5,
        y: 0.,
        yaw: 0.,
    };
    let goal = HybridGoal {
        x: -1.5,
        y: 0.,
        yaw: None,
    };
    let route = hybrid_astar_route(&map, start, goal, &config).unwrap();
    assert_drivable(&map, &route, &config);
    arrives(&route, -1.5, 0., &config);
    assert!(route.iter().all(|p| !p.reverse));
}

#[test]
fn a_sealed_wall_or_a_gap_narrower_than_the_body_has_no_route() {
    let config = HybridConfig::default();
    let start = Pose {
        x: -3.,
        y: 0.,
        yaw: 0.,
    };
    let goal = HybridGoal {
        x: 3.,
        y: 0.,
        yaw: None,
    };
    assert_eq!(
        hybrid_astar_route(&wall_with_gap(0.), start, goal, &config),
        Err("route_no_path")
    );
    assert_eq!(
        hybrid_astar_route(&wall_with_gap(0.7), start, goal, &config),
        Err("route_no_path")
    );
}

#[test]
fn a_blocked_start_or_goal_is_reported() {
    let map = wall_with_gap(0.);
    let config = HybridConfig::default();
    let goal = HybridGoal {
        x: 3.,
        y: 0.,
        yaw: None,
    };
    let on_wall = Pose {
        x: 0.1,
        y: 1.,
        yaw: 0.,
    };
    assert_eq!(
        hybrid_astar_route(&map, on_wall, goal, &config),
        Err("route_start_blocked")
    );
    let start = Pose {
        x: -3.,
        y: 0.,
        yaw: 0.,
    };
    let goal_on_wall = HybridGoal {
        x: 0.1,
        y: 1.,
        yaw: Some(0.),
    };
    assert_eq!(
        hybrid_astar_route(&map, start, goal_on_wall, &config),
        Err("route_goal_blocked")
    );
}
