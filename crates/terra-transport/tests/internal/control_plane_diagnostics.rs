use super::*;
use zenoh::Wait;
#[test]
fn busy_telemetry_lock_is_unknown_not_stale() {
    let mut config = zenoh::Config::default();
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .unwrap();
    config.insert_json5("listen/endpoints", "[]").unwrap();
    let session = zenoh::open(config).wait().unwrap();
    let service = ControlPlane {
        inbox: Default::default(),
        signaling: Default::default(),
        states: Default::default(),
        session,
        base: "diag/7".into(),
        batches: Default::default(),
        stop: Default::default(),
        failed: Default::default(),
        failure_reason: Default::default(),
        worker: None,
        rover_id: 7,
    };
    let guard = service.states.lock().unwrap();
    let details = service.diagnostics();
    assert_eq!(details["telemetry_fresh"], serde_json::Value::Null);
    assert_eq!(details["telemetry_sample_available"], false);
    drop(guard);
}
