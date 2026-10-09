use terra_experiment::summarize;
#[test]
fn search_summary_counts_confirmation_once_and_keeps_exhaustion_distinct() {
    let trace = vec![
        serde_json::json!({"kind":"control_tick","time":1,"search":{"search_id":"s","phase":"searching"},"events":[{"kind":"route_failed"}]}),
        serde_json::json!({"kind":"control_tick","time":3,"search":{"search_id":"s","phase":"completed","report":{"confirmed_at":3}},"events":[{"kind":"target_confirmed"}]}),
        serde_json::json!({"kind":"control_tick","time":4,"search":{"search_id":"s","phase":"completed","report":{"confirmed_at":3}}}),
        serde_json::json!({"kind":"control_tick","time":5,"search":{"search_id":"t","phase":"exhausted"}}),
    ];
    let s = summarize(&trace);
    assert_eq!(s.search_confirmations, 1);
    assert_eq!(s.search_exhaustions, 1);
    assert_eq!(s.search_route_failures, 1);
    assert_eq!(s.time_to_target_confirmation, Some(3.));
}
