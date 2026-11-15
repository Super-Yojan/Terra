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
/// Every wire enum's `as_str` matches its serde spelling, and `Level::parse` inverts it.
#[test]
fn wire_names_match_serde() {
    let wire = |v: serde_json::Value| v.as_str().unwrap().to_owned();
    for level in Level::ALL {
        assert_eq!(wire(serde_json::to_value(level).unwrap()), level.as_str());
        assert_eq!(Level::parse(level.as_str()), Some(level));
    }
    for source in [
        Source::None,
        Source::Operator,
        Source::AssistedOperator,
        Source::Waypoint,
        Source::Frontier,
        Source::Explore,
        Source::TargetSearch,
        Source::Task,
        Source::TaskAgentDrive,
        Source::Recovery,
        Source::Dock,
    ] {
        assert_eq!(wire(serde_json::to_value(source).unwrap()), source.as_str());
    }
    for safety in [Safety::Clear, Safety::Hold, Safety::EmergencyStop] {
        assert_eq!(wire(serde_json::to_value(safety).unwrap()), safety.as_str());
    }
    use terra_navigation::PlanReason;
    for plan in [
        PlanReason::Active,
        PlanReason::Assisted,
        PlanReason::ObstacleBlocked,
        PlanReason::InvalidInput,
    ] {
        assert_eq!(wire(serde_json::to_value(plan).unwrap()), plan.as_str());
        assert_eq!(Reason::from(plan).as_str(), plan.as_str());
    }
    for end in [
        EndReason::BudgetExpired,
        EndReason::NoFrontiersLeft,
        EndReason::OperatorStop,
        EndReason::OperatorTakeover,
        EndReason::MapStale,
        EndReason::SafetyHold,
        EndReason::ObjectiveComplete,
    ] {
        assert_eq!(wire(serde_json::to_value(end).unwrap()), end.as_str());
    }
    assert_eq!(Reason::from("route_no_path"), Reason::RouteNoPath);
    assert_eq!(
        Reason::from("dock_aligning"),
        Reason::Other("dock_aligning")
    );
}
