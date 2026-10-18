//! Mission search regressions for observation memory, waypoint validity, and recovery.

use terra_mapping::MapSnapshot;
use terra_navigation::{Pose, SearchBounds, SearchNavigationDecision, SearchNavigator};
/// Observe a central 5 m square, leaving unknown space around it to form frontiers.
fn map() -> MapSnapshot {
    let mut m = MapSnapshot {
        width: 40,
        height: 40,
        resolution: 0.25,
        origin_x: -5.,
        origin_y: -5.,
        occupancy: vec![-1; 1600],
    };
    for y in 10..30 {
        for x in 10..30 {
            m.occupancy[y * 40 + x] = 0;
        }
    }
    m
}
/// Keep the 0.3 m footprint within an 8 m square mission area.
fn nav() -> SearchNavigator {
    SearchNavigator::new(
        SearchBounds {
            min_x: -4.,
            min_y: -4.,
            max_x: 4.,
            max_y: 4.,
        },
        0.3,
    )
    .unwrap()
}
/// The first proposed waypoint must be locally reachable and leave room for the
/// full footprint inside mission bounds.
#[test]
fn goals_are_reachable_and_contained() {
    let mut n = nav();
    let m = map();
    let SearchNavigationDecision::Goal { x, y } = n.next_goal(&m, Pose::default(), 1, 0.) else {
        panic!("goal required")
    };
    assert!(terra_navigation::reachable(&m, Pose::default(), 0.3, x, y));
    assert!(x.abs() + 0.3 <= 4. && y.abs() + 0.3 <= 4.);
}
/// Moving the local observation window must not erase previously observed cells
/// from the navigator's fixed mission map.
#[test]
fn map_shifts_do_not_forget_observed_cells() {
    let mut n = nav();
    let m = map();
    n.next_goal(&m, Pose::default(), 1, 0.);
    let count = n.observed_cells();
    let mut shifted = m.clone();
    shifted.origin_x += 1.;
    n.next_goal(&shifted, Pose::default(), 2, 1.);
    assert!(n.observed_cells() >= count);
}
/// Reject a backward clock and a changed cell resolution rather than continuing
/// navigation with inconsistent time or map geometry.
#[test]
fn resolution_change_holds_and_invalid_time_cannot_progress() {
    let mut n = nav();
    let m = map();
    n.next_goal(&m, Pose::default(), 1, 1.);
    assert!(matches!(
        n.next_goal(&m, Pose::default(), 2, 0.),
        SearchNavigationDecision::Blocked { .. }
    ));
    let mut m = m;
    m.resolution = 0.5;
    assert!(matches!(
        n.next_goal(&m, Pose::default(), 2, 2.),
        SearchNavigationDecision::Blocked { .. }
    ));
}
/// Repeated calls at an unchanged pose eventually exhaust stall recovery instead
/// of endlessly choosing replacement goals.
#[test]
fn blocked_routes_have_bounded_recovery() {
    let mut n = nav();
    let m = map();
    // Eleven-second intervals exceed the ten-second progress timeout each time.
    for t in [0., 11., 22., 33.] {
        let d = n.next_goal(&m, Pose::default(), t as u64 + 1, t);
        if t == 33. {
            assert!(matches!(d, SearchNavigationDecision::Blocked { .. }));
        }
    }
}
/// With no unknown cells there are no exploration frontiers. Exhaustion describes
/// map coverage, without asserting whether a mission target exists.
#[test]
fn fully_observed_area_exhausts_without_claiming_target_absence() {
    let mut n = nav();
    let mut m = map();
    m.occupancy.fill(0);
    assert_eq!(
        n.next_goal(&m, Pose::default(), 1, 0.),
        SearchNavigationDecision::Exhausted
    );
}
/// A rover inside a U-shaped obstacle must receive a reachable intermediate
/// waypoint toward the open side rather than a shortcut through the closed wall.
#[test]
fn u_shaped_obstacle_uses_intermediate_reachable_route() {
    let mut n = nav();
    let mut m = map();
    // Close the right, lower, and upper sides, leaving an escape to the left.
    for y in 14..26 {
        m.occupancy[y * 40 + 23] = 100;
    }
    for x in 16..24 {
        m.occupancy[14 * 40 + x] = 100;
        m.occupancy[25 * 40 + x] = 100;
    }
    let d = n.next_goal(&m, Pose::default(), 1, 0.);
    let SearchNavigationDecision::Goal { x, y } = d else {
        panic!("reachable escape expected")
    };
    assert!(terra_navigation::reachable(&m, Pose::default(), 0.3, x, y));
    assert!(x < 0.5, "route must exit the open side");
}
