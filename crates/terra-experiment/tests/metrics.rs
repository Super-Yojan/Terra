use terra_experiment::*;
#[test]
fn near_miss_is_an_episode_not_every_tick() {
    let mut n = NearMiss::default();
    assert_eq!(n.update(Some(0.2)), Some("near_miss_start"));
    assert_eq!(n.update(Some(0.21)), None);
    assert_eq!(n.update(Some(0.5)), Some("near_miss_end"));
    assert_eq!(n.update(None), None);
}
#[test]
fn incomplete_records_are_not_complete_runs() {
    let values = vec![
        serde_json::json!({"kind":"manifest"}),
        serde_json::json!({"kind":"control_tick","time":1.,"status":{"active_source":"operator"},"events":[{"kind":"request_accepted","reason":"operator_selection"}]}),
        serde_json::json!({"kind":"run_end","complete":false}),
    ];
    assert!(!summarize(&values).complete);
    assert_eq!(summarize(&values).mode_changes, 1);
}
#[test]
fn mission_outcomes_and_interventions_use_run_exposure() {
    let records = vec![
        serde_json::json!({"kind":"control_tick","time":10,"mission":{"observations":[{"observed_at":5}],"complete":false},"events":[{"kind":"takeover"},{"kind":"request_accepted","reason":"proposal_approve"}]}),
        serde_json::json!({"kind":"control_tick","time":30,"mission":{"complete":true}}),
    ];
    let s = summarize(&records);
    assert_eq!(s.takeovers, 1);
    assert_eq!(s.time_to_first_discovery, Some(5.));
    assert_eq!(s.mission_completion_time, Some(30.));
    assert_eq!(s.interventions_per_minute, 4.);
}
