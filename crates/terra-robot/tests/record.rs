use terra_experiment::summarize;
use terra_robot::{CONTROL_TICK, ControlTickRecord};

/// A record built from a runtime tick is the line `summarize` reads.
#[test]
fn control_tick_record_round_trips_through_summarize() {
    let mut core =
        terra_robot::Core::new(Default::default(), "run".into()).expect("default runtime");
    let output = core.step(2.).unwrap();
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
