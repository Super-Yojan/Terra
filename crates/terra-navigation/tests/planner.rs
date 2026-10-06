use terra_mapping::MapSnapshot;
use terra_navigation::*;
fn map() -> MapSnapshot {
    MapSnapshot {
        width: 40,
        height: 40,
        resolution: 0.25,
        origin_x: -5.,
        origin_y: -5.,
        occupancy: vec![0; 1600],
    }
}
#[test]
fn clear_space_and_blocked_braking() {
    let mut m = map();
    let p = LocalPlanner::default();
    let pose = Pose::default();
    let intent = Twist {
        linear: 1.,
        angular: 0.,
    };
    assert!(p.plan(&m, pose, Twist::default(), intent, 0.1).twist.linear > 0.);
    for y in 0..40 {
        m.occupancy[y * 40 + 22] = 100;
    }
    let result = p.plan(
        &m,
        pose,
        Twist {
            linear: 1.,
            angular: 0.,
        },
        intent,
        0.1,
    );
    assert!(result.twist.linear < 1.);
    assert!(!p.admissible(&m, pose, intent));
}
#[test]
fn unknown_space_and_frontier() {
    let mut m = map();
    let p = LocalPlanner::default();
    m.occupancy.fill(-1);
    assert!(!p.admissible(
        &m,
        Pose::default(),
        Twist {
            linear: 1.,
            angular: 0.
        }
    ));
    assert!(frontier(&m, Pose::default(), 0.35, &[]).is_none());
    for y in 15..25 {
        for x in 15..25 {
            m.occupancy[y * 40 + x] = 0;
        }
    }
    let f = frontier(&m, Pose::default(), 0.35, &[]).unwrap();
    assert!(f.0.is_finite());
    assert!(f.1.is_finite());
}
#[test]
fn determinism_reverse_turning_and_bad_inputs() {
    let m = map();
    let p = LocalPlanner::default();
    let pose = Pose::default();
    let desired = Twist {
        linear: -1.,
        angular: 0.5,
    };
    let a = p.plan(&m, pose, Twist::default(), desired, 0.1);
    let b = p.plan(&m, pose, Twist::default(), desired, 0.1);
    assert_eq!(a.twist, b.twist);
    assert!(a.twist.linear < 0.);
    assert!(a.twist.angular <= 0.2);
    assert_eq!(
        p.plan(
            &m,
            pose,
            Twist::default(),
            Twist {
                linear: f64::NAN,
                angular: 0.
            },
            0.1
        )
        .twist,
        Twist::default()
    );
    assert!(!p.admissible(
        &m,
        Pose {
            x: f64::NAN,
            ..pose
        },
        desired
    ));
}
#[test]
fn known_current_footprint_allows_stationary_hold_in_unknown_map() {
    let mut m = map();
    m.occupancy.fill(-1);
    assert!(LocalPlanner::default().admissible(&m, Pose::default(), Twist::default()));
}
#[test]
fn frontier_can_leave_known_robot_footprint_when_grid_under_body_is_unobserved() {
    let mut m = map();
    m.occupancy.fill(-1);
    for y in 14..27 {
        for x in 14..35 {
            m.occupancy[y * 40 + x] = 0;
        }
    }
    for y in 0..40 {
        for x in 0..40 {
            let px = -5. + (x as f64 + 0.5) * 0.25;
            let py = -5. + (y as f64 + 0.5) * 0.25;
            if px.hypot(py) < 0.65 {
                m.occupancy[y * 40 + x] = -1;
            }
        }
    }
    let pose = Pose::default();
    let target = frontier(&m, pose, 0.65, &[]).expect("reachable observed region ahead");
    assert!(reachable(&m, pose, 0.65, target.0, target.1));
}
