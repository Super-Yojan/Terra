use terra_experiment::*;
/// Summarize a manifest, accepted mode change, and incomplete terminal record; preserve both failure and event count.
#[test]
fn incomplete_records_are_not_complete_runs() {
    let values = vec![
        serde_json::json!({"kind": "manifest"}),
        serde_json::json!({
            "kind": "control_tick",
            "time": 1.,
            "status": {"active_source": "operator"},
            "events": [{"kind": "request_accepted","reason": "operator_selection"}],
        }),
        serde_json::json!({"kind": "run_end","complete": false}),
    ];
    assert!(!summarize(&values).complete);
    assert_eq!(summarize(&values).mode_changes, 1);
}
/// Summarize two control ticks spanning 30 seconds; verify discovery/completion times and four interventions per minute.
#[test]
fn mission_outcomes_and_interventions_use_run_exposure() {
    let records = vec![
        serde_json::json!({
            "kind": "control_tick",
            "time": 10,
            "mission": {"observations": [{"observed_at": 5}],"complete": false},
            "events": [{"kind": "takeover"},{"kind": "request_accepted","reason": "proposal_approve"}],
        }),
        serde_json::json!({"kind": "control_tick","time": 30,"mission": {"complete": true}}),
    ];
    let s = summarize(&records);
    assert_eq!(s.takeovers, 1);
    assert_eq!(s.time_to_first_discovery, Some(5.));
    assert_eq!(s.mission_completion_time, Some(30.));
    assert_eq!(s.interventions_per_minute, 4.);
}
/// A record built from a runtime tick is the line `summarize` reads.
#[test]
fn control_tick_record_round_trips_through_summarize() {
    let mut runtime =
        terra_runtime::Runtime::new(Default::default(), "run".into()).expect("default runtime");
    let output = runtime.step(2.).unwrap();
    let mut record = ControlTickRecord::new(&output, "run", 1, 12.);
    record.rover_id = Some(3);
    record.extra.insert(
        "mission".into(),
        serde_json::json!({"complete": true, "observations": []}),
    );
    let line = record.to_value();
    assert_eq!(line["kind"], CONTROL_TICK);
    assert_eq!(line["rover_id"], 3);
    assert!(line.get("source_time").is_none());
    let s = summarize(&[line]);
    assert_eq!(s.duration, 12.);
    assert_eq!(s.mission_completion_time, Some(12.));
}
