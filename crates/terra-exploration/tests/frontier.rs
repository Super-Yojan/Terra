//! Frontier selection on partly observed maps (moved from navigation's planner tests
//! when the duplicate frontier search there was removed).

use terra_exploration::{InformationGainScorer, select_frontier};
use terra_mapping::MapSnapshot;
use terra_navigation::{Footprint, Pose, reachable};

fn frontier(m: &MapSnapshot, pose: Pose, body: &Footprint) -> Option<(f64, f64)> {
    select_frontier(m, pose, body, &[], 1., &InformationGainScorer).map(|c| (c.x, c.y))
}
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
/// Unknown space alone offers no frontier; revealing a free patch creates a finite one.
#[test]
fn unknown_space_and_frontier() {
    let mut m = map();
    m.occupancy.fill(-1);
    let body = Footprint {
        length: 0.495,
        width: 0.495,
        margin: 0.,
    };
    assert!(frontier(&m, Pose::default(), &body).is_none());
    for y in 15..25 {
        for x in 15..25 {
            m.occupancy[y * 40 + x] = 0;
        }
    }
    let f = frontier(&m, Pose::default(), &body).unwrap();
    assert!(f.0.is_finite());
    assert!(f.1.is_finite());
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
            // Under the 0.919 m square body (half 0.46 m) at heading 0.
            if px.abs() < 0.46 && py.abs() < 0.46 {
                m.occupancy[y * 40 + x] = -1;
            }
        }
    }
    let pose = Pose::default();
    let body = Footprint {
        length: 0.9192,
        width: 0.9192,
        margin: 0.,
    };
    let target = frontier(&m, pose, &body).expect("reachable observed region ahead");
    assert!(reachable(&m, pose, &body, target.0, target.1));
}
