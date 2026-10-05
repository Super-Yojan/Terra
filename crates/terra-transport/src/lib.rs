//! Leased, latest-command-only Zenoh velocity publisher for Terra.
use std::{
    sync::{Arc, Mutex},
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};
use zenoh::Wait;
const LEASE: Duration = Duration::from_millis(250);
#[derive(Debug)]
pub enum TransportError {
    InvalidSettings,
    InvalidTarget,
    Closed,
    Network(String),
}
impl std::fmt::Display for TransportError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for TransportError {}
#[derive(Default)]
struct State {
    target: Option<(f64, f64, Instant)>,
    shutdown: bool,
    error: Option<String>,
}
pub struct RoverConnection {
    state: Arc<Mutex<State>>,
    worker: Mutex<Option<JoinHandle<()>>>,
}
fn validate(endpoint: &str, prefix: &str) -> Result<(), TransportError> {
    if endpoint.len() > 512
        || !endpoint.starts_with("tcp/")
        || endpoint.contains(['\n', '\r', '#', '?'])
        || endpoint.parse::<zenoh::config::EndPoint>().is_err()
        || prefix.len() > 256
        || prefix.contains('*')
        || zenoh::key_expr::KeyExpr::new(prefix).is_err()
    {
        return Err(TransportError::InvalidSettings);
    }
    Ok(())
}
fn payload(linear: f64, angular: f64) -> Result<String, TransportError> {
    if !linear.is_finite() || !angular.is_finite() || linear.abs() > 2.0 || angular.abs() > 2.0 {
        return Err(TransportError::InvalidTarget);
    }
    Ok(serde_json::json!({"linear":linear,"angular":angular}).to_string())
}
impl RoverConnection {
    /// Opens a direct TCP client. Run off the UI thread; connection timeout is 2 seconds.
    pub fn connect(endpoint: &str, prefix: &str, rover_id: u64) -> Result<Self, TransportError> {
        validate(endpoint, prefix)?;
        let mut config = zenoh::Config::default();
        let values = [
            ("mode", "\"client\"".to_string()),
            (
                "connect/endpoints",
                serde_json::json!([endpoint]).to_string(),
            ),
            ("connect/timeout_ms", "2000".to_string()),
            ("connect/exit_on_failure", "true".to_string()),
            ("scouting/multicast/enabled", "false".to_string()),
        ];
        for (key, value) in values {
            config
                .insert_json5(key, &value)
                .map_err(|e| TransportError::Network(e.to_string()))?;
        }
        let session = zenoh::open(config)
            .wait()
            .map_err(|e| TransportError::Network(e.to_string()))?;
        let state = Arc::new(Mutex::new(State::default()));
        let shared = state.clone();
        let key = format!("{prefix}/{rover_id}/cmd_vel");
        let worker = thread::Builder::new()
            .name("terra-mobile-zenoh".into())
            .spawn(move || {
                loop {
                    let (shutdown, linear, angular) = {
                        let state = shared.lock().unwrap();
                        let target = state.target.filter(|(_, _, t)| t.elapsed() < LEASE);
                        let (linear, angular) = target.map_or((0.0, 0.0), |(v, w, _)| (v, w));
                        (
                            state.shutdown,
                            if state.shutdown { 0.0 } else { linear },
                            if state.shutdown { 0.0 } else { angular },
                        )
                    };
                    let bytes = payload(linear, angular).expect("validated target");
                    if let Err(error) = session
                        .put(key.clone(), bytes)
                        .congestion_control(zenoh::qos::CongestionControl::Drop)
                        .wait()
                    {
                        shared.lock().unwrap().error = Some(error.to_string());
                        break;
                    }
                    if shutdown || session.is_closed() {
                        break;
                    }
                    thread::sleep(Duration::from_millis(50));
                }
                let _ = session.close().wait();
                shared.lock().unwrap().shutdown = true;
            })
            .map_err(|e| TransportError::Network(e.to_string()))?;
        Ok(Self {
            state,
            worker: Mutex::new(Some(worker)),
        })
    }
    /// Refresh the 250 ms command lease. Targets are limited to ±2 m/s and ±2 rad/s.
    pub fn set_target(&self, linear: f64, angular: f64) -> Result<(), TransportError> {
        // Invalid targets revoke the previous lease immediately.
        if let Err(error) = payload(linear, angular) {
            self.state.lock().unwrap().target = None;
            return Err(error);
        }
        let mut state = self.state.lock().unwrap();
        if state.shutdown {
            return Err(TransportError::Closed);
        }
        state.target = Some((linear, angular, Instant::now()));
        Ok(())
    }
    pub fn status(&self) -> String {
        let state = self.state.lock().unwrap();
        if let Some(error) = &state.error {
            format!("Transport error: {error}")
        } else if state.shutdown {
            "Disconnected".into()
        } else {
            "Zenoh session open · commands at 20 Hz".into()
        }
    }
    /// Worker sends zero before closing. Rover-side 500 ms watchdog remains independent.
    pub fn disconnect(&self) {
        self.state.lock().unwrap().shutdown = true;
        if let Some(worker) = self.worker.lock().unwrap().take() {
            let _ = worker.join();
        }
    }
}
impl Drop for RoverConnection {
    fn drop(&mut self) {
        self.disconnect();
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn validates_topic_endpoint_and_target_contract() {
        assert!(validate("tcp/127.0.0.1:7447", "terra/rover").is_ok());
        assert!(validate("udp/127.0.0.1:7447", "terra/rover").is_err());
        assert!(validate("tcp/127.0.0.1:7447", "terra/*").is_err());
        let p: serde_json::Value = serde_json::from_str(&payload(1.0, 0.3).unwrap()).unwrap();
        assert_eq!(p, serde_json::json!({"linear":1.0,"angular":0.3}));
        assert!(payload(f64::NAN, 0.0).is_err());
        assert!(payload(2.1, 0.0).is_err());
    }
    #[test]
    #[ignore = "requires local TCP sockets"]
    fn publishes_selected_rover_expires_lease_and_disconnects_with_zero() {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let endpoint = format!("tcp/127.0.0.1:{}", listener.local_addr().unwrap().port());
        drop(listener);
        let mut config = zenoh::Config::default();
        config.insert_json5("mode", "\"peer\"").unwrap();
        config
            .insert_json5(
                "listen/endpoints",
                &serde_json::json!([endpoint]).to_string(),
            )
            .unwrap();
        config
            .insert_json5("scouting/multicast/enabled", "false")
            .unwrap();
        let server = zenoh::open(config).wait().unwrap();
        let subscriber = server
            .declare_subscriber("terra/rover/*/cmd_vel")
            .wait()
            .unwrap();
        let client = RoverConnection::connect(&endpoint, "terra/rover", 7).unwrap();
        client.set_target(1.0, 0.3).unwrap();
        let deadline = Instant::now() + Duration::from_secs(3);
        let mut moving = false;
        let mut stopped = false;
        while Instant::now() < deadline {
            if let Ok(Some(sample)) = subscriber.recv_timeout(Duration::from_millis(100)) {
                assert_eq!(sample.key_expr().as_str(), "terra/rover/7/cmd_vel");
                let p: serde_json::Value =
                    serde_json::from_slice(&sample.payload().to_bytes()).unwrap();
                if p["linear"] == 1.0 {
                    moving = true;
                } else if moving && p["linear"] == 0.0 {
                    stopped = true;
                    break;
                }
            }
        }
        assert!(
            moving && stopped,
            "must publish motion then expire lease to zero"
        );
        client.set_target(-0.5, 0.0).unwrap();
        thread::sleep(Duration::from_millis(100));
        client.disconnect();
        assert_eq!(client.status(), "Disconnected");
        assert!(client.set_target(1.0, 0.0).is_err());
        let mut last = None;
        while let Ok(Some(sample)) = subscriber.recv_timeout(Duration::from_millis(100)) {
            last = Some(
                serde_json::from_slice::<serde_json::Value>(&sample.payload().to_bytes()).unwrap(),
            );
        }
        assert_eq!(last.unwrap()["linear"], 0.0);
        server.close().wait().unwrap();
    }
}
