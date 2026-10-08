use terra_exploration::*;
use terra_mapping::MapSnapshot;
use terra_navigation::Pose;
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
#[test]
fn frontier_selection_matches_navigation_and_respects_a_scorer() {
    let grid = with_known_core(true);
    let pose = Pose::default();
    let selected = select_frontier(&grid, pose, 0.35, &[], 1., &InformationGainScorer).unwrap();
    let legacy = terra_navigation::frontier(&grid, pose, 0.35, &[]).unwrap();
    assert_eq!((selected.x, selected.y), legacy);
    assert!(selected.information_gain > 0);
    assert!(selected.score > 0.);
    let prefer_east = select_frontier(
        &grid,
        pose,
        0.35,
        &[],
        1.,
        &FnScorer(|candidate: &FrontierCandidate| candidate.x),
    )
    .unwrap();
    assert!(prefer_east.x > 0.5);
    let skipped = select_frontier(
        &grid,
        pose,
        0.35,
        &[(prefer_east.x, prefer_east.y)],
        1.,
        &InformationGainScorer,
    )
    .unwrap();
    assert!((skipped.x - prefer_east.x).hypot(skipped.y - prefer_east.y) > 1.);
}
#[test]
fn unreachable_frontier_is_blacklisted_and_does_not_stick() {
    let grid = with_known_core(true);
    let mut explorer = Explorer::default();
    explorer.config_mut().stuck_timeout = 1.;
    explorer.config_mut().blacklist_seconds = 30.;
    explorer.config_mut().max_attempts = 3;
    explorer.config_mut().footprint_radius = 0.35;
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
#[test]
fn repeated_blocks_end_instead_of_holding_forever() {
    let grid = with_known_core(true);
    let mut explorer = Explorer::default();
    explorer.config_mut().stuck_timeout = 0.;
    explorer.config_mut().blacklist_seconds = 10.;
    explorer.config_mut().max_attempts = 1;
    explorer.config_mut().footprint_radius = 0.35;
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
#[test]
fn budget_expiry_stops_a_live_frontier() {
    let grid = with_known_core(true);
    let mut explorer = Explorer::default();
    explorer.config_mut().footprint_radius = 0.35;
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
#[test]
fn find_target_stub_stops_on_a_reported_detection() {
    let grid = with_known_core(true);
    let mut explorer = Explorer::default();
    explorer.set_objective(Box::new(FindTargetsObjective::new(["survivor"])));
    assert!(explorer.start(60., "run-1"));
    assert!(matches!(
        explorer.step(tick(0., Some(&grid), false, false, false, 0., 0., 0.)),
        ExploreAction::Seek { .. }
    ));
    explorer.notify_detection(Detection {
        label: "rock".into(),
        x: 1.,
        y: 1.,
        confidence: 0.9,
        observed_at: 0.2,
    });
    assert_eq!(explorer.phase(), ExplorePhase::Running);
    explorer.notify_detection(Detection {
        label: "survivor".into(),
        x: 2.,
        y: -1.,
        confidence: 0.8,
        observed_at: 0.4,
    });
    assert_eq!(
        explorer.step(tick(0.5, Some(&grid), false, true, false, 0., 0., 1.)),
        ExploreAction::Finished(EndReason::ObjectiveComplete)
    );
}
#[test]
fn coverage_counts_observed_free_cells() {
    let grid = with_known_core(true);
    let (area, ratio) = map_coverage(&grid).unwrap();
    assert!(area > 0.);
    assert!(ratio > 0. && ratio < 1.);
    assert!(map_coverage(&map(0)).unwrap().1 == 1.);
}
