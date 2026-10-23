//! Local planning regressions for braking, unknown space, determinism, and telemetry.

use terra_mapping::MapSnapshot;
use terra_navigation::*;
/// Build an observed, obstacle-free 10 m square centered on the default pose.
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
/// Free space permits forward acceleration, but a wall ahead must reduce the
/// command and reject the requested constant velocity with its braking tail.
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
    // Place a full-height wall 0.5 m ahead of the pose, inside the default footprint.
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
/// Unknown space cannot support forward motion or a frontier proposal on its own;
/// revealing a free patch creates a finite frontier candidate.
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
/// Repeated reverse-turn requests select identical commands within the sampled
/// yaw acceleration bound; nonfinite intent or pose must fail closed.
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
/// The starting-footprint exception permits holding still when underbody cells
/// are unknown, without needing to classify the surrounding map as free.
#[test]
fn known_current_footprint_allows_stationary_hold_in_unknown_map() {
    let mut m = map();
    m.occupancy.fill(-1);
    assert!(LocalPlanner::default().admissible(&m, Pose::default(), Twist::default()));
}
/// Unknown underbody cells must not trap the rover when an observed region is
/// available ahead; both frontier selection and reachability share this exception.
#[test]
fn frontier_can_leave_known_robot_footprint_when_grid_under_body_is_unobserved() {
    let mut m = map();
    m.occupancy.fill(-1);
    for y in 14..27 {
        for x in 14..35 {
            m.occupancy[y * 40 + x] = 0;
        }
    }
    // Remove only observations under the rover, preserving the free region ahead.
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

/// Recording diagnostics must preserve the chosen command, identify its candidate,
/// retain rejection reasons, and keep sampled paths and serialized telemetry bounded.
#[test]
fn trace_matches_decision_and_records_rejected_candidates() {
    let planner = LocalPlanner::default();
    let pose = Pose::default();
    let intent = Twist {
        linear: 1.0,
        angular: 0.0,
    };
    let free = map();
    let expected = planner.plan(&free, pose, Twist::default(), intent, 0.1);
    let (decision, trace) = planner.plan_with_trace(&free, pose, Twist::default(), intent, 0.1);
    assert_eq!(decision.twist, expected.twist);
    assert_eq!(trace.candidates.len(), 121);
    let selected = trace.selected.unwrap();
    assert_eq!(trace.candidates[selected].velocity, decision.twist);
    assert!(trace.candidates[selected].rejection.is_none());
    assert!(
        trace
            .candidates
            .iter()
            .all(|c| !c.points_mm.is_empty() && c.points_mm.len() <= 16)
    );
    // With every cell occupied, all candidates fail and no selected index is valid.
    let mut blocked = free;
    blocked.occupancy.fill(100);
    let (decision, trace) = planner.plan_with_trace(&blocked, pose, Twist::default(), intent, 0.1);
    assert_eq!(decision.reason, "obstacle_blocked");
    assert!(trace.selected.is_none());
    assert!(
        trace
            .candidates
            .iter()
            .all(|c| c.rejection.as_deref() == Some("occupied"))
    );
    assert!(serde_json::to_vec(&trace).unwrap().len() < 55_000);
}

#[test]
fn managed_reverse_checks_unknown_rear_and_marks_trace() {
    let planner = LocalPlanner::default();
    let limiter = terra_control::TurningLimiter::new(Default::default()).unwrap();
    let (out, trace) = planner.plan_with_turning(
        &map(),
        Pose::default(),
        Twist::default(),
        Twist {
            linear: -0.2,
            angular: 0.3,
        },
        0.1,
        &limiter,
        0.,
    );
    assert!(out.twist.linear < 0.);
    let selected = &trace.candidates[trace.selected.unwrap()];
    assert_eq!(selected.velocity, out.twist);
    assert_eq!(selected.direction, "reverse");
    assert!(out.twist.linear >= -planner.config.max_reverse);
    let mut rear = map();
    rear.occupancy[20 * 40 + 17] = -1; // behind, inside rear footprint: no unknown exemption
    assert!(!planner.admissible(&rear, Pose::default(), out.twist));
    rear.occupancy[20 * 40 + 17] = 100;
    assert!(!planner.admissible(&rear, Pose::default(), out.twist));
}
#[test]
fn multipoint_turn_precedes_hold_and_prefers_forward_when_clear() {
    let planner = LocalPlanner::default();
    let c = terra_control::TurningConfig::default();
    let segments = planner
        .heading_maneuver(&map(), Pose::default(), std::f64::consts::PI, &c)
        .unwrap();
    assert!(segments.iter().any(|s| s.direction == "reverse"));
    assert!(segments.iter().any(|s| s.direction == "forward"));
    for segment in &segments {
        assert!(planner.admissible(&map(), segment.start, segment.velocity));
    }
    let (out, _) = planner.plan_with_turning(
        &map(),
        Pose::default(),
        Twist::default(),
        Twist {
            linear: 0.2,
            angular: 0.,
        },
        0.1,
        &terra_control::TurningLimiter::new(c).unwrap(),
        0.,
    );
    assert!(out.twist.linear > 0.);
}

#[test]
fn forward_heading_arc_remains_available_when_reverse_unknown() {
    let mut map = terra_mapping::MapSnapshot {
        width: 80,
        height: 80,
        resolution: 0.1,
        origin_x: -4.,
        origin_y: -4.,
        occupancy: vec![0; 6400],
    };
    map.occupancy[40 * 80 + 38] = -1;
    let planner = terra_navigation::LocalPlanner::default();
    let pose = terra_navigation::Pose::default();
    assert!(planner.admissible(
        &map,
        pose,
        terra_navigation::Twist {
            linear: 0.15,
            angular: 0.3
        }
    ));
    assert!(
        planner
            .heading_maneuver(&map, pose, 1.1, &Default::default())
            .is_some()
    );
}
#[test]
fn measured_overspeed_does_not_expand_candidate_limits() {
    let map = terra_mapping::MapSnapshot {
        width: 100,
        height: 100,
        resolution: 0.1,
        origin_x: -5.,
        origin_y: -5.,
        occupancy: vec![0; 10000],
    };
    let mut planner = terra_navigation::LocalPlanner::default();
    planner.config.max_linear = 0.3;
    planner.config.max_angular = 0.2;
    let (plan, _) = planner.plan_with_trace(
        &map,
        terra_navigation::Pose::default(),
        terra_navigation::Twist {
            linear: 1.,
            angular: 1.,
        },
        terra_navigation::Twist {
            linear: 1.,
            angular: 1.,
        },
        0.1,
    );
    assert!(
        plan.twist.linear <= 0.3 && plan.twist.angular.abs() <= 0.2,
        "{:?}",
        plan.twist
    );
}
