use terra_autonomy::*;
/// Exercise strict level, budget, teleop-session, and safety JSON contracts, including unknown fields and invalid primitive types.
#[test]
fn strict_requests_and_correlation() {
    assert!(decode_level(br#"{"level":"teleop","token":"switch-1"}"#).is_some());
    let explore =
        decode_level(br#"{"level":"explore","token":"explore-1","budget_minutes":2}"#).unwrap();
    assert_eq!(explore.resolved_budget_seconds(), Some(120.));
    assert!(
        decode_level(br#"{"level":"explore","token":"bad","budget_seconds":1,"budget_minutes":1}"#)
            .is_none()
    );
    assert!(decode_level(br#"{"level":"explore","token":"bad","budget_seconds":0}"#).is_none());
    for b in [
        br#"{"level":"magic","token":"x"}"#.as_slice(),
        br#"{"level":"teleop","token":""}"#,
        br#"{"level":"teleop","token":"x","extra":1}"#,
    ] {
        assert!(decode_level(b).is_none());
    }
    assert!(
        decode_teleop(
            br#"{"linear":1,"angular":0,"operator_session_id":"session-1","sequence":2}"#
        )
        .is_some()
    );
    assert!(decode_teleop(br#"{"linear":true,"angular":0}"#).is_none());
    assert!(decode_safety(br#"{"action":"stop","token":"stop-1"}"#).is_some());
}

/// Pinned to the same strings as ARGOS `OPERATOR_LEVELS` and the thesis
/// tables (Super-Yojan/ARGOS#112). Change all three together.
#[test]
fn level_labels_match_argos_and_the_thesis() {
    use terra_autonomy::Level;
    let labels: Vec<_> = [
        Level::Teleop,
        Level::AssistedTeleop,
        Level::WaypointDirect,
        Level::Waypoint,
        Level::Explore,
        Level::Task,
    ]
    .into_iter()
    .map(Level::label)
    .collect();
    assert_eq!(
        labels,
        [
            "L0 Manual",
            "L1 Assisted",
            "L2 Direct",
            "L3 Planned",
            "L4 Explore + search",
            "L5 Task",
        ]
    );
    assert_eq!(Level::TargetSearch.label(), "L4 Explore + search");
}
