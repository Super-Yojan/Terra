use terra_exploration::*;
use terra_mapping::MapSnapshot;
use terra_navigation::Pose;

/// Square body whose circumscribed radius is `r`: the circle these tests were written for.
fn body(r: f64) -> terra_navigation::Footprint {
    let side = r * std::f64::consts::SQRT_2;
    terra_navigation::Footprint {
        length: side,
        width: side,
        margin: 0.,
    }
}
/// Build a 40 × 40 grid at 0.25 m resolution, centred on the origin, filled with the supplied occupancy value.
fn map(fill: i8) -> MapSnapshot {
    MapSnapshot {
        width: 40,
        height: 40,
        resolution: 0.25,
        origin_x: -5.,
        origin_y: -5.,
        occupancy: vec![fill; 1600],
    }
}
/// Build a 4 m square free core in unknown space, or a fully free map when `fill_unknown` is false.
fn with_known_core(fill_unknown: bool) -> MapSnapshot {
    let mut grid = map(if fill_unknown { -1 } else { 0 });
    if fill_unknown {
        for y in 12..28 {
            for x in 12..28 {
                grid.occupancy[y * 40 + x] = 0;
            }
        }
    }
    grid
}
/// Build an origin-pose exploration tick with caller-controlled time, map, blockage, and follower state; distances are metres.
fn tick<'a>(
    now: f64,
    map: Option<&'a MapSnapshot>,
    blocked: bool,
    active: bool,
    arrived: bool,
    x: f64,
    y: f64,
    distance: f64,
) -> ExploreTick<'a> {
    ExploreTick {
        now,
        pose: Pose::default(),
        map,
        planner_blocked: blocked,
        goal_active: active,
        goal_arrived: arrived,
        goal_x: x,
        goal_y: y,
        goal_distance: distance,
    }
}
/// Rank a known core surrounded by unknown cells, then prefer east and exclude that goal.
#[test]
fn frontier_selection_respects_a_scorer() {
    let grid = with_known_core(true);
    let pose = Pose::default();
    let selected =
        select_frontier(&grid, pose, &body(0.35), &[], 1., &InformationGainScorer).unwrap();
    assert!(selected.information_gain > 0);
    assert!(selected.score > 0.);
    let prefer_east = select_frontier(
        &grid,
        pose,
        &body(0.35),
        &[],
        1.,
        &FnScorer(|candidate: &FrontierCandidate| candidate.x),
    )
    .unwrap();
    assert!(prefer_east.x > 0.5);
    let skipped = select_frontier(
        &grid,
        pose,
        &body(0.35),
        &[(prefer_east.x, prefer_east.y)],
        1.,
        &InformationGainScorer,
    )
    .unwrap();
    assert!((skipped.x - prefer_east.x).hypot(skipped.y - prefer_east.y) > 1.);
}
/// Wall off an assigned frontier after a short planner hold; verify blacklisting selects another goal or waits for a retry.
#[test]
fn unreachable_frontier_is_blacklisted_and_does_not_stick() {
    let grid = with_known_core(true);
    let mut explorer = Explorer::default();
    explorer.config_mut().stuck_timeout = 1.;
    explorer.config_mut().blacklist_seconds = 30.;
    explorer.config_mut().max_attempts = 3;
    explorer.config_mut().footprint = body(0.35);
    assert!(explorer.start(60., "run-1"));
    let action = explorer.step(tick(0., Some(&grid), false, false, false, 0., 0., 0.));
    let ExploreAction::Seek { x, y } = action else {
        panic!("expected a frontier, got {action:?}");
    };
    assert!(explorer.status(0.).target.is_some());
    let still = explorer.step(tick(0.2, Some(&grid), true, true, false, x, y, 2.));
    assert_eq!(still, ExploreAction::Seek { x, y });
    assert!(
        explorer
            .take_events()
            .iter()
            .all(|event| event.kind != "frontier_blocked")
    );
    let mut walled = grid.clone();
    for index in 0..walled.occupancy.len() {
        let px = walled.origin_x + (index % 40) as f64 * walled.resolution;
        let py = walled.origin_y + (index / 40) as f64 * walled.resolution;
        if (px - x).hypot(py - y) < 0.4 {
            walled.occupancy[index] = 100;
        }
    }
    let next = explorer.step(tick(0.4, Some(&walled), false, true, false, x, y, 2.));
    let events = explorer.take_events();
    assert!(events.iter().any(|event| event.kind == "frontier_blocked"));
    match next {
        ExploreAction::Seek { x: nx, y: ny } => {
            assert!((nx - x).hypot(ny - y) > 1.);
            assert_eq!(explorer.status(0.4).phase, ExplorePhase::Running);
        }
        ExploreAction::Hold => {
            assert_eq!(explorer.status(0.4).phase, ExplorePhase::Running);
            assert!(explorer.status(0.4).blacklist_count >= 1);
        }
        other => panic!("blocked frontier held the run: {other:?}"),
    }
}
/// Use one permitted attempt and an exclusion radius covering the map; one blockage must exhaust all frontiers.
#[test]
fn repeated_blocks_end_instead_of_holding_forever() {
    let grid = with_known_core(true);
    let mut explorer = Explorer::default();
    explorer.config_mut().stuck_timeout = 0.;
    explorer.config_mut().blacklist_seconds = 10.;
    explorer.config_mut().max_attempts = 1;
    explorer.config_mut().footprint = body(0.35);
    explorer.config_mut().exclude_radius = 50.;
    assert!(explorer.start(60., "run-1"));
    let ExploreAction::Seek { x, y } =
        explorer.step(tick(0., Some(&grid), false, false, false, 0., 0., 0.))
    else {
        panic!("missing frontier");
    };
    let action = explorer.step(tick(0.1, Some(&grid), true, true, false, x, y, 2.));
    assert_eq!(action, ExploreAction::Finished(EndReason::NoFrontiersLeft));
    assert_eq!(
        explorer.status(0.1).end_reason,
        Some(EndReason::NoFrontiersLeft)
    );
}
/// Advance an active frontier through a one-second budget and verify terminal elapsed time stays frozen.
#[test]
fn budget_expiry_stops_a_live_frontier() {
    let grid = with_known_core(true);
    let mut explorer = Explorer::default();
    explorer.config_mut().footprint = body(0.35);
    assert!(explorer.start(1., "run-1"));
    let ExploreAction::Seek { x, y } =
        explorer.step(tick(0., Some(&grid), false, false, false, 0., 0., 0.))
    else {
        panic!("missing frontier");
    };
    assert_eq!(explorer.status(0.).phase, ExplorePhase::Running);
    assert!((explorer.status(0.).remaining_seconds - 1.).abs() < 1e-9);
    let mid = explorer.step(tick(0.4, Some(&grid), false, true, false, x, y, 1.5));
    assert_eq!(mid, ExploreAction::Seek { x, y });
    assert_eq!(explorer.status(0.4).phase, ExplorePhase::Running);
    assert_eq!(
        explorer.step(tick(1., Some(&grid), false, true, false, x, y, 1.2)),
        ExploreAction::Finished(EndReason::BudgetExpired)
    );
    let status = explorer.status(5.);
    assert_eq!(status.phase, ExplorePhase::Ended);
    assert_eq!(status.end_reason, Some(EndReason::BudgetExpired));
    assert!((status.elapsed_seconds - 1.).abs() < 1e-9);
    assert_eq!(status.remaining_seconds, 0.);
    assert_eq!(
        explorer.step(tick(1.2, Some(&grid), false, false, false, 0., 0., 0.)),
        ExploreAction::Finished(EndReason::BudgetExpired)
    );
}
/// Verify external stop reasons are latched per run and a fresh run can end with a different reason.
#[test]
fn stop_and_takeover_end_the_run() {
    let mut explorer = Explorer::default();
    assert!(explorer.start(30., "run-1"));
    explorer.stop(EndReason::OperatorStop);
    assert_eq!(
        explorer.status(0.).end_reason,
        Some(EndReason::OperatorStop)
    );
    explorer.stop(EndReason::OperatorTakeover);
    assert_eq!(
        explorer.status(0.).end_reason,
        Some(EndReason::OperatorStop)
    );
    assert!(explorer.start(30., "run-2"));
    explorer.stop(EndReason::OperatorTakeover);
    assert_eq!(
        explorer.status(1.).end_reason,
        Some(EndReason::OperatorTakeover)
    );
    assert_eq!(explorer.status(1.).phase, ExplorePhase::Ended);
}
/// Supply a fully free map with no unknown cells; exploration must immediately report no remaining frontiers.
#[test]
fn no_frontiers_left_when_the_map_is_fully_known() {
    let grid = map(0);
    let mut explorer = Explorer::default();
    assert!(explorer.start(30., "run-1"));
    assert_eq!(
        explorer.step(tick(0., Some(&grid), false, false, false, 0., 0., 0.)),
        ExploreAction::Finished(EndReason::NoFrontiersLeft)
    );
    assert_eq!(
        explorer.status(0.).end_reason,
        Some(EndReason::NoFrontiersLeft)
    );
}
/// Measure a partly known free core and a fully known map, checking positive area and fractional versus complete coverage.
#[test]
fn coverage_counts_observed_free_cells() {
    let grid = with_known_core(true);
    let (area, ratio) = map_coverage(&grid).unwrap();
    assert!(area > 0.);
    assert!(ratio > 0. && ratio < 1.);
    assert!(map_coverage(&map(0)).unwrap().1 == 1.);
}
/// Free cells a forward depth camera sees at the start of a run: a 60° wedge
/// from 0.35 m to 8 m ahead, with the chassis and its sides still unknown.
fn front_camera_wedge() -> MapSnapshot {
    let (width, resolution) = (200usize, 0.1);
    let mut occupancy = vec![-1i8; width * width];
    for row in 0..width {
        for column in 0..width {
            let x = -10. + (column as f64 + 0.5) * resolution;
            let y = -10. + (row as f64 + 0.5) * resolution;
            if x > 0.35 && x < 8. && y.abs() < x * 30f64.to_radians().tan() {
                occupancy[row * width + column] = 0;
            }
        }
    }
    MapSnapshot {
        width: width as u32,
        height: width as u32,
        resolution,
        origin_x: -10.,
        origin_y: -10.,
        occupancy,
    }
}
/// Known ring around the rover after a full look-around: everything from 0.35 m out.
fn after_look_around() -> MapSnapshot {
    let mut grid = front_camera_wedge();
    for row in 0..200 {
        for column in 0..200 {
            let x = -10. + (column as f64 + 0.5) * 0.1;
            let y = -10. + (row as f64 + 0.5) * 0.1;
            let d = x.hypot(y);
            if d > 0.35 && d < 6. {
                grid.occupancy[row * 200 + column] = 0;
            }
        }
    }
    grid
}
fn at(now: f64, yaw: f64, map: &MapSnapshot) -> ExploreTick<'_> {
    ExploreTick {
        now,
        pose: Pose { x: 0., y: 0., yaw },
        map: Some(map),
        planner_blocked: false,
        goal_active: false,
        goal_arrived: false,
        goal_x: 0.,
        goal_y: 0.,
        goal_distance: 0.,
    }
}
/// A rover in its own blind zone turns in place instead of ending, then explores.
#[test]
fn a_rover_in_its_blind_zone_looks_around_before_choosing() {
    let wedge = front_camera_wedge();
    assert!(
        select_frontier(
            &wedge,
            Pose::default(),
            &body(0.65),
            &[],
            1.,
            &InformationGainScorer
        )
        .is_none()
    );
    let mut explorer = Explorer::default();
    assert!(explorer.start(60., "wedge"));
    let rate = explorer.config().scan_yaw_rate;
    assert_eq!(
        explorer.step(at(0., 0., &wedge)),
        ExploreAction::Scan { yaw_rate: rate }
    );
    assert!(explorer.status(0.).scanning);
    assert_eq!(
        explorer.step(at(1., rate, &wedge)),
        ExploreAction::Scan { yaw_rate: rate }
    );
    // The camera has now mapped around the rover: a frontier wins mid-turn.
    let known = after_look_around();
    let ExploreAction::Seek { x, y } = explorer.step(at(2., 2. * rate, &known)) else {
        panic!("expected a frontier after the look-around");
    };
    assert!(x.hypot(y) > 0.8);
    assert!(!explorer.status(2.).scanning);
    assert_eq!(explorer.status(2.).phase, ExplorePhase::Running);
}
/// A full turn that reveals nothing reachable ends the run; it does not spin forever.
#[test]
fn a_fruitless_look_around_ends_with_no_frontiers() {
    let wedge = front_camera_wedge();
    let mut explorer = Explorer::default();
    assert!(explorer.start(600., "wedge"));
    let mut yaw: f64 = 0.;
    let mut now = 0.;
    let mut action = explorer.step(at(now, yaw, &wedge));
    let mut ticks = 0;
    while matches!(action, ExploreAction::Scan { .. }) {
        ticks += 1;
        assert!(ticks < 1000, "look-around never finished");
        now += 0.1;
        yaw = (yaw + 0.035 + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
            - std::f64::consts::PI;
        action = explorer.step(at(now, yaw, &wedge));
    }
    assert_eq!(action, ExploreAction::Finished(EndReason::NoFrontiersLeft));
    // About one full turn at 0.35 rad/s.
    assert!((17.0..19.5).contains(&now), "{now}");
    // A rover that cannot turn gives up after the timeout instead.
    let mut stuck = Explorer::default();
    assert!(stuck.start(600., "stuck"));
    let timeout = stuck.config().scan_timeout;
    assert!(matches!(
        stuck.step(at(0., 0., &wedge)),
        ExploreAction::Scan { .. }
    ));
    assert!(matches!(
        stuck.step(at(timeout - 0.1, 0., &wedge)),
        ExploreAction::Scan { .. }
    ));
    assert_eq!(
        stuck.step(at(timeout, 0., &wedge)),
        ExploreAction::Finished(EndReason::NoFrontiersLeft)
    );
}
/// Through unknown space, the camera wedge alone yields a frontier straight ahead.
#[test]
fn through_unknown_finds_a_frontier_in_the_camera_wedge() {
    let wedge = front_camera_wedge();
    let pose = Pose::default();
    let candidate =
        select_frontier_through_unknown(&wedge, pose, &body(0.65), &[], 1., &InformationGainScorer)
            .expect("a frontier through unknown space");
    assert!(candidate.distance_m > 0.8);
    assert!(candidate.information_gain > 0);
    // The goal is a cell the camera saw as free, not a guess in unknown space.
    let (column, row) = wedge.cell(candidate.x, candidate.y).unwrap();
    assert_eq!(wedge.occupancy[row * 200 + column], 0);
    // The explorer seeks it instead of turning in place.
    let mut explorer = Explorer::default();
    explorer.config_mut().through_unknown = true;
    assert!(explorer.start(60., "optimistic"));
    assert!(matches!(
        explorer.step(at(0., 0., &wedge)),
        ExploreAction::Seek { .. }
    ));
}
/// Occupied cells still block: no frontier is chosen beyond or against a wall.
#[test]
fn through_unknown_never_passes_an_occupied_wall() {
    let mut walled = front_camera_wedge();
    for row in 0..200 {
        for column in 0..200 {
            let x = -10. + (column as f64 + 0.5) * 0.1;
            if (1.5..1.7).contains(&x) {
                walled.occupancy[row * 200 + column] = 100;
            }
        }
    }
    let chosen = select_frontier_through_unknown(
        &walled,
        Pose::default(),
        &body(0.65),
        &[],
        1.,
        &InformationGainScorer,
    );
    // The body fits at the candidate at some heading, so at least its inner
    // half-width stays short of the wall.
    assert!(
        chosen.is_none_or(|c| c.x + body(0.65).inradius() < 1.5),
        "{chosen:?}"
    );
    // Unknown cells become free; observed cells and the grid stay as they were.
    let passable = terra_navigation::unknown_as_free(&walled);
    assert!(passable.occupancy.iter().all(|cell| *cell >= 0));
    assert_eq!(
        passable
            .occupancy
            .iter()
            .filter(|cell| **cell >= 65)
            .count(),
        walled.occupancy.iter().filter(|cell| **cell >= 65).count()
    );
    assert_eq!((passable.width, passable.resolution), (200, 0.1));
}
/// Issue #113: a frontier whose surroundings got mapped is done, not blocked.
#[test]
fn a_frontier_that_got_mapped_is_dropped_without_a_blacklist() {
    let grid = with_known_core(true);
    let mut explorer = Explorer::default();
    explorer.config_mut().footprint = body(0.35);
    assert!(explorer.start(60., "run-1"));
    let ExploreAction::Seek { x, y } =
        explorer.step(tick(0., Some(&grid), false, false, false, 0., 0., 0.))
    else {
        panic!("missing frontier");
    };
    explorer.take_events();
    // The camera has now seen everything; the rover is still 1.5 m short.
    let seen = map(0);
    let action = explorer.step(tick(0.5, Some(&seen), false, true, false, x, y, 1.5));
    let events = explorer.take_events();
    assert!(events.iter().any(|e| e.kind == "frontier_mapped"));
    assert!(events.iter().all(|e| e.kind != "frontier_blocked"));
    assert_eq!(explorer.status(0.5).blacklist_count, 0);
    assert_eq!(action, ExploreAction::Finished(EndReason::NoFrontiersLeft));
}
/// Issue #113: one map update that cuts the route off does not drop the goal at once.
#[test]
fn a_cut_off_route_gets_the_stuck_window_before_blacklisting() {
    let grid = with_known_core(true);
    let mut explorer = Explorer::default();
    explorer.config_mut().footprint = body(0.35);
    explorer.config_mut().stuck_timeout = 2.;
    assert!(explorer.start(60., "run-1"));
    let ExploreAction::Seek { x, y } =
        explorer.step(tick(0., Some(&grid), false, false, false, 0., 0., 0.))
    else {
        panic!("missing frontier");
    };
    // Ring of occupied cells around the rover; the frontier itself stays clear.
    let mut boxed = grid.clone();
    for index in 0..boxed.occupancy.len() {
        let px = boxed.origin_x + ((index % 40) as f64 + 0.5) * boxed.resolution;
        let py = boxed.origin_y + ((index / 40) as f64 + 0.5) * boxed.resolution;
        let r = px.hypot(py);
        if (0.5..0.7).contains(&r) {
            boxed.occupancy[index] = 100;
        }
    }
    explorer.take_events();
    let held = explorer.step(tick(0.5, Some(&boxed), false, true, false, x, y, 2.));
    assert_eq!(held, ExploreAction::Seek { x, y });
    assert!(
        explorer
            .take_events()
            .iter()
            .all(|e| e.kind != "frontier_blocked")
    );
    explorer.step(tick(2.6, Some(&boxed), false, true, false, x, y, 2.));
    let events = explorer.take_events();
    let blocked = events
        .iter()
        .find(|e| e.kind == "frontier_blocked")
        .expect("blocked after the stuck window");
    assert!(
        blocked.reason.contains("why=unreachable"),
        "{}",
        blocked.reason
    );
}
/// Issue #113: with every frontier on a short blacklist, retry the best one now
/// instead of parking in `frontier_retry`.
#[test]
fn all_frontiers_blacklisted_retries_instead_of_holding() {
    let grid = with_known_core(true);
    let mut explorer = Explorer::default();
    explorer.config_mut().footprint = body(0.35);
    explorer.config_mut().stuck_timeout = 0.;
    explorer.config_mut().max_attempts = 3;
    explorer.config_mut().exclude_radius = 50.;
    assert!(explorer.start(60., "run-1"));
    let mut actions = Vec::new();
    let ExploreAction::Seek { x, y } =
        explorer.step(tick(0., Some(&grid), false, false, false, 0., 0., 0.))
    else {
        panic!("missing frontier");
    };
    let (mut gx, mut gy) = (x, y);
    for i in 1..=3 {
        let action = explorer.step(tick(
            i as f64 * 0.1,
            Some(&grid),
            true,
            true,
            false,
            gx,
            gy,
            2.,
        ));
        if let ExploreAction::Seek { x, y } = action {
            (gx, gy) = (x, y);
        }
        actions.push(action);
    }
    let events = explorer.take_events();
    assert!(
        matches!(actions[0], ExploreAction::Seek { .. }),
        "{actions:?}"
    );
    assert!(
        matches!(actions[1], ExploreAction::Seek { .. }),
        "{actions:?}"
    );
    let retries: Vec<_> = events
        .iter()
        .filter(|e| e.kind == "frontier_retry_early")
        .map(|e| e.reason.as_str())
        .collect();
    assert!(
        retries[0].contains("attempts=1,max_attempts=3"),
        "{retries:?}"
    );
    assert!(
        retries[1].contains("attempts=2,max_attempts=3"),
        "{retries:?}"
    );
    assert!(
        actions.iter().all(|a| *a != ExploreAction::Hold),
        "{actions:?}"
    );
    // Third failure exhausts the only frontier; the run ends instead of waiting.
    assert_eq!(
        actions[2],
        ExploreAction::Finished(EndReason::NoFrontiersLeft)
    );
}
/// Issue #113: an arc away from the goal (no in-place turn) is not "stuck" after 5 s.
#[test]
fn no_progress_uses_its_own_longer_window() {
    let grid = with_known_core(true);
    let mut explorer = Explorer::default();
    explorer.config_mut().footprint = body(0.35);
    explorer.config_mut().stuck_timeout = 5.;
    explorer.config_mut().no_progress_timeout = 12.;
    assert!(explorer.start(60., "run-1"));
    let ExploreAction::Seek { x, y } =
        explorer.step(tick(0., Some(&grid), false, false, false, 0., 0., 0.))
    else {
        panic!("missing frontier");
    };
    explorer.step(tick(0.1, Some(&grid), false, true, false, x, y, 2.));
    let still = explorer.step(tick(8., Some(&grid), false, true, false, x, y, 2.3));
    assert_eq!(still, ExploreAction::Seek { x, y });
    explorer.take_events();
    explorer.step(tick(12.2, Some(&grid), false, true, false, x, y, 2.3));
    let events = explorer.take_events();
    let blocked = events
        .iter()
        .find(|e| e.kind == "frontier_blocked")
        .unwrap();
    assert!(
        blocked.reason.contains("why=no_progress"),
        "{}",
        blocked.reason
    );
}
