use super::*;
#[test]
fn busy_telemetry_lock_is_unknown_not_stale() {
    let socket = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let endpoint = format!("tcp/127.0.0.1:{}", socket.local_addr().unwrap().port());
    drop(socket);
    let service = ControlPlane::listen(&endpoint, "diag", 7).unwrap();
    let guard = service.states.lock().unwrap();
    let details = service.diagnostics();
    assert_eq!(details["telemetry_fresh"], serde_json::Value::Null);
    assert_eq!(details["telemetry_sample_available"], false);
    drop(guard);
}
