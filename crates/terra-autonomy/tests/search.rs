use terra_autonomy::*;
fn request() -> SearchRequest {
    serde_json::from_str(r#"{"version":1,"run_id":"local","search_id":"s","token":"start","target_class":"survivor","bounds":{"min_x":-5,"min_y":-5,"max_x":5,"max_y":5},"time_budget_s":60}"#).unwrap()
}
fn observation(frame: u64, time: f64) -> TargetObservation {
    TargetObservation {
        run_id: "local".into(),
        search_id: "s".into(),
        frame_id: frame,
        target_class: "survivor".into(),
        confidence: 0.9,
        world_x: 2.,
        world_y: 1.,
        received_at: time,
        evidence_id: format!("frame-{frame}"),
    }
}
#[test]
fn distinct_consistent_frames_confirm_once() {
    let mut s = SearchController::start(request(), 0.).unwrap();
    for (f, t) in [(1, 0.), (2, 0.3), (3, 0.6)] {
        s.observe(observation(f, t), t).unwrap();
    }
    let report = s.status(0.6, true).report.unwrap();
    assert_eq!(report.frame_ids, vec![1, 2, 3]);
    assert_eq!(s.status(0.6, true).phase, SearchPhase::Completed);
    assert!(s.observe(observation(4, 0.7), 0.7).is_err());
    assert_eq!(s.status(0.7, true).report.unwrap(), report);
}
#[test]
fn stale_duplicate_foreign_and_wrong_class_never_confirm() {
    let mut s = SearchController::start(request(), 0.).unwrap();
    s.observe(observation(1, 0.), 0.).unwrap();
    assert!(s.observe(observation(1, 0.3), 0.3).is_err());
    let mut o = observation(2, 0.3);
    o.search_id = "other".into();
    assert!(s.observe(o, 0.3).is_err());
    let mut o = observation(2, 0.3);
    o.target_class = "chair".into();
    assert!(s.observe(o, 0.3).is_err());
    assert!(s.observe(observation(2, 0.3), 1.).is_err());
    assert!(s.status(1., true).report.is_none());
}
#[test]
fn low_confidence_or_disagreement_cannot_complete() {
    let mut s = SearchController::start(request(), 0.).unwrap();
    let mut o = observation(1, 0.);
    o.confidence = 0.7;
    assert!(s.observe(o, 0.).is_err());
    s.observe(observation(2, 0.1), 0.1).unwrap();
    let mut o = observation(3, 0.4);
    o.world_x = 4.;
    s.observe(o, 0.4).unwrap();
    s.observe(observation(4, 0.7), 0.7).unwrap();
    assert!(s.status(0.7, true).report.is_none());
}
#[test]
fn paused_budget_expires_and_cancel_is_terminal() {
    let mut s = SearchController::start(request(), 0.).unwrap();
    s.action(SearchAction::Pause, 1.).unwrap();
    s.update_time(60.).unwrap();
    assert_eq!(s.status(60., true).phase, SearchPhase::TimedOut);
    assert!(s.action(SearchAction::Resume, 61.).is_err());
    let mut s = SearchController::start(request(), 0.).unwrap();
    s.action(SearchAction::Cancel, 1.).unwrap();
    assert!(s.observe(observation(1, 2.), 2.).is_err());
    assert!(s.action(SearchAction::Resume, 2.).is_err());
}
#[test]
fn nonmonotonic_time_does_not_extend_search() {
    let mut s = SearchController::start(request(), 1.).unwrap();
    s.update_time(2.).unwrap();
    assert!(s.update_time(1.5).is_err());
    assert_eq!(s.status(2., true).elapsed_s, 1.);
}
#[test]
fn ordinary_pause_resume_does_not_consume_route_failure_budget() {
    let mut s = SearchController::start(request(), 0.).unwrap();
    let mut m = terra_mapping::MapSnapshot {
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
    for n in 0..4 {
        let t = n as f64 * 2.;
        s.tick(&m, terra_navigation::Pose::default(), 1, t).unwrap();
        s.action(SearchAction::Pause, t + 0.1).unwrap();
        s.action(SearchAction::Resume, t + 0.2).unwrap();
        assert!(
            s.tick(&m, terra_navigation::Pose::default(), 1, t + 0.3)
                .unwrap()
                .is_some()
        );
    }
}
