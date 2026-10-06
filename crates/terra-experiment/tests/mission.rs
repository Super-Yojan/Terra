use terra_experiment::*;
#[test]
fn deterministic_detection_and_confirmation() {
    let c = MissionConfig::rescue(42);
    assert_eq!(c, MissionConfig::rescue(42));
    let target = c.survivors[0].clone();
    let mut m = Mission::new(c).unwrap();
    assert!(m.observe(0, target.x, target.y, 0., false).is_empty());
    let sightings = m.observe(0, target.x, target.y, 1., true);
    assert_eq!(sightings.len(), 1);
    assert!(m.report(0, target.id, 2.));
    assert!(!m.report(0, target.id, 3.));
    assert_eq!(m.status(3.).confirmed, 1);
    assert!(!m.status(3.).complete);
    assert!(m.observe(0, target.x, target.y, f64::NAN, true).is_empty());
}
#[test]
fn bounded_recorder_flushes_and_identifies_run() {
    let p = std::env::temp_dir().join(format!("terra-log-{}.jsonl", std::process::id()));
    let mut r = RunRecorder::create(&p, serde_json::json!({"run_id":"run-1"}), 8).unwrap();
    r.record(serde_json::json!({"kind":"takeover","time":1.}))
        .unwrap();
    r.close().unwrap();
    let records = read_records(&std::fs::read(&p).unwrap()).unwrap();
    assert_eq!(records.len(), 3);
    assert_eq!(records[0]["run_id"], "run-1");
    std::fs::remove_file(p).unwrap();
}
#[test]
fn confirmation_requires_safe_return_for_completion() {
    let mut c = MissionConfig::rescue(1);
    c.survivors.truncate(1);
    let t = c.survivors[0].clone();
    let mut m = Mission::new(c).unwrap();
    m.observe_target(0, t.id, t.x, t.y, 0.);
    assert!(m.report(0, t.id, 1.));
    assert!(!m.status(1.).complete);
    assert_eq!(m.status(1.).phase, "return");
    m.update_pose(0, 0., 0., 2.);
    assert!(m.status(2.).complete);
}
#[test]
fn seeded_mission_never_exposes_hidden_targets_and_completes_all_objectives() {
    let c = MissionConfig::rescue(42);
    let targets = c.survivors.clone();
    let mut m = Mission::new(c).unwrap();
    assert!(m.status(0.).observations.is_empty());
    for (i, t) in targets.iter().enumerate() {
        let now = (i * 2) as f64;
        m.observe_target(0, t.id, t.x, t.y, now);
        assert!(m.report(0, t.id, now + 1.));
    }
    assert_eq!(m.status(6.).phase, "return");
    m.update_pose(0, 0., 0., 7.);
    assert!(m.status(7.).complete);
}
