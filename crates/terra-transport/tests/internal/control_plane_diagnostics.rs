use super::*;
#[test]
fn busy_telemetry_lock_is_unknown_not_stale() {
    let service = ControlPlane {
        inbox: Default::default(),
        states: Default::default(),
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
