use std::f64::consts::FRAC_PI_2;
use terra_map_merge::*;

#[test]
fn priority_rule_occupied_beats_free_beats_unknown() {
    let mut a = Grid::unknown(2, 1, 1.0, 0.0, 0.0);
    a.set(0, 0, 100);
    let b = Grid::filled(2, 1, 1.0, 0.0, 0.0, 10);
    let out = merge(
        &[(&a, Pose2::IDENTITY), (&b, Pose2::IDENTITY)],
        1.0,
        FusionRule::Priority,
    )
    .unwrap();
    assert_eq!(out.cells, vec![100, 0]);
    let c = Grid::unknown(2, 1, 1.0, 0.0, 0.0);
    let out = merge(&[(&c, Pose2::IDENTITY)], 1.0, FusionRule::Priority).unwrap();
    assert_eq!(out.cells, vec![-1, -1]);
}

#[test]
fn log_odds_lets_two_free_votes_outweigh_a_weak_obstacle() {
    let a = Grid::filled(1, 1, 1.0, 0.0, 0.0, 70);
    let b = Grid::filled(1, 1, 1.0, 0.0, 0.0, 20);
    let c = Grid::filled(1, 1, 1.0, 0.0, 0.0, 20);
    let grids = [
        (&a, Pose2::IDENTITY),
        (&b, Pose2::IDENTITY),
        (&c, Pose2::IDENTITY),
    ];
    assert_eq!(
        merge(&grids, 1.0, FusionRule::Priority).unwrap().cells,
        vec![100]
    );
    let fused = merge(&grids, 1.0, FusionRule::LogOdds).unwrap().cells[0];
    assert_eq!(classify(fused), CellClass::Free, "{fused}");
}

#[test]
fn log_odds_agreeing_obstacles_get_more_confident() {
    let a = Grid::filled(1, 1, 1.0, 0.0, 0.0, 70);
    let fused = merge(
        &[(&a, Pose2::IDENTITY), (&a, Pose2::IDENTITY)],
        1.0,
        FusionRule::LogOdds,
    )
    .unwrap();
    assert!(fused.cells[0] > 80, "{}", fused.cells[0]);
}

#[test]
fn pose_round_trips() {
    let p = Pose2::new(2.0, -1.5, 0.4);
    let (wx, wy) = p.apply(0.3, 1.7);
    let (lx, ly) = p.inverse_apply(wx, wy);
    assert!((lx - 0.3).abs() < 1e-12 && (ly - 1.7).abs() < 1e-12);
}

/// A wall seen by two rovers in their own frames lands in one place in the world.
#[test]
fn rotated_and_shifted_maps_line_up_in_world() {
    // Rover A: identity. Obstacle cell centre (1.75, 0.75) in both frames.
    let mut a = Grid::filled(4, 4, 0.5, 0.0, 0.0, 0);
    a.set(3, 1, 100);
    // Rover B spawned at (2, 0) facing +90°. World (1.75, 0.75) is local (0.75, 0.25).
    let mut b = Grid::filled(4, 4, 0.5, 0.0, 0.0, 0);
    b.set(1, 0, 100);
    let pose_b = Pose2::new(2.0, 0.0, FRAC_PI_2);
    let out = merge(
        &[(&a, Pose2::IDENTITY), (&b, pose_b)],
        0.5,
        FusionRule::LogOdds,
    )
    .unwrap();
    assert_eq!(
        classify(out.at_world(1.75, 0.75).unwrap()),
        CellClass::Occupied
    );
    // Both rovers saw the same cell as occupied, so log-odds is more confident than either.
    assert!(out.at_world(1.75, 0.75).unwrap() == 100);
    // Rover B covers world x in [0, 2], y in [0, 2]; A covers [0, 2] too: bounds are 2 m.
    assert_eq!((out.width, out.height), (4, 4));
}

#[test]
fn agreement_counts_errors_and_coverage() {
    let mut truth = Grid::filled(4, 1, 1.0, 0.0, 0.0, 0);
    truth.set(3, 0, 100);
    let mut map = Grid::unknown(4, 1, 1.0, 0.0, 0.0);
    map.set(0, 0, 0); // match
    map.set(1, 0, 100); // false occupied
    map.set(3, 0, 0); // false free
    let a = agreement(&map, &truth);
    assert_eq!(
        (
            a.compared,
            a.matching,
            a.missing,
            a.false_occupied,
            a.false_free
        ),
        (3, 1, 1, 1, 1)
    );
    assert!((a.coverage() - 0.75).abs() < 1e-12);
}

#[test]
fn rejects_bad_input() {
    let g = Grid::unknown(2, 2, 1.0, 0.0, 0.0);
    assert_eq!(
        merge(&[], 1.0, FusionRule::Priority),
        Err(MergeError::Empty)
    );
    assert_eq!(
        merge(&[(&g, Pose2::IDENTITY)], 0.0, FusionRule::Priority),
        Err(MergeError::BadResolution)
    );
    let mut bad = g.clone();
    bad.cells.pop();
    assert_eq!(
        merge(&[(&bad, Pose2::IDENTITY)], 1.0, FusionRule::Priority),
        Err(MergeError::BadGrid(0))
    );
    assert_eq!(
        merge(
            &[(&g, Pose2::new(f64::NAN, 0.0, 0.0))],
            1.0,
            FusionRule::Priority
        ),
        Err(MergeError::BadPose(0))
    );
    let far = Pose2::new(10_000.0, 0.0, 0.0);
    assert_eq!(
        merge(
            &[(&g, Pose2::IDENTITY), (&g, far)],
            0.1,
            FusionRule::Priority
        ),
        Err(MergeError::TooLarge)
    );
}
