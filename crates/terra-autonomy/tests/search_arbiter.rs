use terra_autonomy::*;
use terra_mapping::MapSnapshot;
use terra_navigation::{Pose, Twist};
/// Build a 10 m square unknown map with a 5 m square observed free core for reachable frontiers.
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
/// Build fresh healthy origin-pose evidence on map revision 1 at the supplied seconds timestamp.
fn input(m: &MapSnapshot, now: f64) -> ArbiterInput<'_> {
    ArbiterInput {
        now,
        pose: Pose::default(),
        pose_time: now,
        map: Some(m),
        map_time: Some(now),
        map_revision: 1,
        measured: Twist::default(),
        healthy: true,
    }
}
/// Build a 60-second survivor search bounded by ±4 m and correlated to the local run.
fn req() -> SearchRequest {
    SearchRequest {
        version: 1,
        run_id: "local".into(),
        search_id: "s".into(),
        token: "start".into(),
        target_class: "survivor".into(),
        bounds: SearchBounds {
            min_x: -4.,
            min_y: -4.,
            max_x: 4.,
            max_y: 4.,
        },
        time_budget_s: 60.,
    }
}
/// Build an arbiter with a survivor detector advertised at time zero and target-search authority selected.
fn arbiter() -> AutonomyArbiter {
    let mut a = AutonomyArbiter::default();
    a.set_detector_capability(vec!["survivor".into()], "sim-v1".into(), 0.)
        .unwrap();
    a.set_level(LevelRequest {
        level: Level::TargetSearch,
        token: "mode".into(),
        budget_seconds: None,
        budget_minutes: None,
    });
    a
}
/// Reject target-search authority without a detector; with a detector but no session, preserve zero motion.
#[test]
fn mode_without_detector_or_request_never_moves() {
    let mut a = AutonomyArbiter::default();
    a.set_level(LevelRequest {
        level: Level::TargetSearch,
        token: "m".into(),
        budget_seconds: None,
        budget_minutes: None,
    });
    assert_eq!(
        a.step(input(&map(), 0.1)).status.result.as_deref(),
        Some("rejected")
    );
    let mut a = arbiter();
    let o = a.step(input(&map(), 0.1));
    assert_eq!(o.twist, Twist::default());
    assert!(!o.status.supported_levels.is_empty());
}
/// Start on a known core without approval, then latch a stop and verify reset does not resurrect the cancelled search.
#[test]
fn search_drives_without_frontier_approval_and_stop_cancels() {
    let mut a = arbiter();
    let m = map();
    assert!(a.start_search(req(), &input(&m, 0.)).is_ok());
    let o = a.step(input(&m, 0.1));
    assert!(o.search.unwrap().active_goal.is_some());
    assert!(o.proposal.is_none());
    a.set_safety(
        SafetyRequest {
            action: SafetyAction::Stop,
            token: "stop".into(),
        },
        true,
    );
    let o = a.step(input(&m, 0.2));
    assert_eq!(o.twist, Twist::default());
    assert_eq!(o.search.unwrap().phase, SearchPhase::Cancelled);
    a.set_safety(
        SafetyRequest {
            action: SafetyAction::Reset,
            token: "reset".into(),
        },
        true,
    );
    assert_eq!(a.step(input(&m, 0.3)).twist, Twist::default());
}
/// Confirm three frames, hold motion, and retain the report until an acknowledgement names the correct run.
#[test]
fn search_completion_holds_and_report_ack_is_correlated() {
    let mut a = arbiter();
    let m = map();
    a.start_search(req(), &input(&m, 0.)).unwrap();
    for (f, t) in [(1, 0.), (2, 0.3), (3, 0.6)] {
        a.set_detector_capability(vec!["survivor".into()], "sim-v1".into(), t)
            .unwrap();
        a.accept_target_observation(
            TargetObservation {
                run_id: "local".into(),
                search_id: "s".into(),
                frame_id: f,
                target_class: "survivor".into(),
                confidence: 0.9,
                world_x: 2.,
                world_y: 1.,
                received_at: t,
                evidence_id: format!("f{f}"),
            },
            t,
        )
        .unwrap();
    }
    let o = a.step(input(&m, 0.7));
    assert_eq!(o.twist, Twist::default());
    assert_eq!(o.search.unwrap().phase, SearchPhase::Completed);
    assert_eq!(o.pending_reports.len(), 1);
    let mut ack = SearchReportAck {
        version: 1,
        run_id: "other".into(),
        search_id: "s".into(),
        report_id: "s:report".into(),
        token: "a".into(),
    };
    assert!(!a.ack_search_report(ack.clone()));
    ack.run_id = "local".into();
    assert!(a.ack_search_report(ack));
    assert!(a.step(input(&m, 0.8)).pending_reports.is_empty());
}
/// Age out detector capability and verify refreshing it alone leaves the search held for explicit resume.
#[test]
fn detector_loss_holds_and_requires_explicit_resume() {
    let mut a = arbiter();
    let m = map();
    a.start_search(req(), &input(&m, 0.)).unwrap();
    assert_eq!(
        a.step(input(&m, 1.)).search.unwrap().phase,
        SearchPhase::NeedsAttention
    );
    a.set_detector_capability(vec!["survivor".into()], "sim-v1".into(), 1.1)
        .unwrap();
    assert_eq!(a.step(input(&m, 1.2)).twist, Twist::default());
}
/// Replay the accepted start token after teleop takeover; acknowledge idempotently while leaving search cancelled.
#[test]
fn repeated_start_cannot_relaunch_after_takeover() {
    let mut a = arbiter();
    let m = map();
    a.start_search(req(), &input(&m, 0.)).unwrap();
    a.set_level(LevelRequest {
        level: Level::Teleop,
        token: "take".into(),
        budget_seconds: None,
        budget_minutes: None,
    });
    a.start_search(req(), &input(&m, 0.1)).unwrap();
    assert_eq!(a.step(input(&m, 0.2)).twist, Twist::default());
    assert_eq!(
        a.step(input(&m, 0.3)).search.unwrap().phase,
        SearchPhase::Cancelled
    );
}
/// Start a frontier search and verify a nonzero target-search intent despite short navigation steps.
#[test]
fn search_route_intent_is_not_swallowed_by_waypoint_arrival_radius() {
    let mut a = arbiter();
    let m = map();
    a.start_search(req(), &input(&m, 0.)).unwrap();
    let o = a.step(input(&m, 0.1));
    assert_ne!(o.intent, Twist::default());
    assert_eq!(o.status.active_source, "target_search");
}
/// Place the rover centre near the search edge so its footprint crosses the boundary; reject the start.
#[test]
fn out_of_bounds_start_is_rejected_before_search_acceptance() {
    let mut a = arbiter();
    let m = map();
    let mut i = input(&m, 0.);
    i.pose.x = 3.9;
    assert!(a.start_search(req(), &i).is_err());
}
/// Cancel a valid session, reject a subsequent observation, and verify the next tick remains motionless.
#[test]
fn cancel_action_and_foreign_observations_never_resurrect_motion() {
    let mut a = arbiter();
    let m = map();
    a.start_search(req(), &input(&m, 0.)).unwrap();
    a.apply_search_action(
        SearchActionRequest {
            version: 1,
            run_id: "local".into(),
            search_id: "s".into(),
            token: "cancel".into(),
            action: SearchAction::Cancel,
        },
        &input(&m, 0.1),
    )
    .unwrap();
    assert!(
        a.accept_target_observation(
            TargetObservation {
                run_id: "local".into(),
                search_id: "s".into(),
                frame_id: 1,
                target_class: "survivor".into(),
                confidence: 0.9,
                world_x: 2.,
                world_y: 1.,
                received_at: 0.2,
                evidence_id: "f".into()
            },
            0.2
        )
        .is_err()
    );
    assert_eq!(a.step(input(&m, 0.3)).twist, Twist::default());
}
/// Confirm a target and immediately start another session; the first report must remain queued for delivery.
#[test]
fn completed_report_survives_new_search_before_next_control_tick() {
    let mut a = arbiter();
    let m = map();
    a.start_search(req(), &input(&m, 0.)).unwrap();
    for (f, t) in [(1, 0.), (2, 0.3), (3, 0.6)] {
        a.set_detector_capability(vec!["survivor".into()], "sim-v1".into(), t)
            .unwrap();
        a.accept_target_observation(
            TargetObservation {
                run_id: "local".into(),
                search_id: "s".into(),
                frame_id: f,
                target_class: "survivor".into(),
                confidence: 0.9,
                world_x: 2.,
                world_y: 1.,
                received_at: t,
                evidence_id: format!("f{f}"),
            },
            t,
        )
        .unwrap();
    }
    let mut next = req();
    next.search_id = "s2".into();
    next.token = "start2".into();
    a.start_search(next, &input(&m, 0.7)).unwrap();
    assert_eq!(a.step(input(&m, 0.8)).pending_reports.len(), 1);
}
/// Confirm a report and acknowledge its exact identifiers twice; both acknowledgements must succeed.
#[test]
fn report_acknowledgements_are_idempotent() {
    let mut a = arbiter();
    let m = map();
    a.start_search(req(), &input(&m, 0.)).unwrap();
    for (f, t) in [(1, 0.), (2, 0.3), (3, 0.6)] {
        a.set_detector_capability(vec!["survivor".into()], "sim-v1".into(), t)
            .unwrap();
        a.accept_target_observation(
            TargetObservation {
                run_id: "local".into(),
                search_id: "s".into(),
                frame_id: f,
                target_class: "survivor".into(),
                confidence: 0.9,
                world_x: 2.,
                world_y: 1.,
                received_at: t,
                evidence_id: format!("f{f}"),
            },
            t,
        )
        .unwrap();
    }
    let r = SearchReportAck {
        version: 1,
        run_id: "local".into(),
        search_id: "s".into(),
        report_id: "s:report".into(),
        token: "ack".into(),
    };
    assert!(a.ack_search_report(r.clone()));
    assert!(a.ack_search_report(r));
}
