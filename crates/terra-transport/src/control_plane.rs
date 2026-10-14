//! Dashboard endpoint or outbound router client. Transport queues intent, never motor authority.
use crate::TransportError;
use std::{
    collections::{BTreeMap, VecDeque},
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};
use zenoh::Wait;
type Inbox = Arc<Mutex<VecDeque<(String, Vec<u8>, Instant)>>>;
type States = Arc<Mutex<BTreeMap<String, (Option<String>, Instant)>>>;
const TELEMETRY_TTL: Duration = Duration::from_millis(500);

fn visualization_topic(kind: &str) -> bool {
    matches!(
        kind,
        "map/occupancy" | "map/occupancy/heartbeat" | "pointcloud" | "pointcloud/heartbeat"
    )
}

// Visualization is sample-driven, unlike replayable control state. Retain the
// receipt timestamp after taking its payload so fleet liveness keeps its TTL.
fn telemetry_snapshot(
    states: &mut BTreeMap<String, (Option<String>, Instant)>,
) -> (bool, Vec<(String, String)>) {
    let mut active = false;
    let mut snapshots = Vec::new();
    for (kind, (value, at)) in states {
        if at.elapsed() >= TELEMETRY_TTL {
            continue;
        }
        active = true;
        let payload = if visualization_topic(kind) {
            value.take()
        } else {
            value.clone()
        };
        if let Some(payload) = payload {
            snapshots.push((kind.clone(), payload));
        }
    }
    (active, snapshots)
}

pub struct ControlPlane {
    inbox: Inbox,
    states: States,
    batches: Arc<Mutex<BTreeMap<String, (Vec<String>, Instant)>>>,
    stop: Arc<AtomicBool>,
    failed: Arc<AtomicBool>,
    worker: Option<JoinHandle<()>>,
    pub rover_id: u64,
}
impl ControlPlane {
    pub fn listen(endpoint: &str, prefix: &str, id: u64) -> Result<Self, TransportError> {
        crate::validate(endpoint, prefix)?;
        if !endpoint.starts_with("tcp/127.0.0.1:") {
            return Err(TransportError::InvalidSettings);
        }
        Self::open(endpoint, prefix, id, false)
    }
    /// Outbound TCP session, suitable for an explicitly configured Tailscale router.
    /// No listeners, multicast discovery, or automatic application command retries.
    pub fn connect(endpoint: &str, prefix: &str, id: u64) -> Result<Self, TransportError> {
        crate::validate(endpoint, prefix)?;
        Self::open(endpoint, prefix, id, true)
    }
    fn open(endpoint: &str, prefix: &str, id: u64, outbound: bool) -> Result<Self, TransportError> {
        let mut config = zenoh::Config::default();
        if outbound {
            for (key, value) in [
                ("mode", "\"client\""),
                ("listen/endpoints", "[]"),
                ("connect/timeout_ms", "3000"),
                ("connect/exit_on_failure", "true"),
            ] {
                config
                    .insert_json5(key, value)
                    .map_err(|e| TransportError::Network(e.to_string()))?;
            }
        }
        config
            .insert_json5(
                if outbound {
                    "connect/endpoints"
                } else {
                    "listen/endpoints"
                },
                &serde_json::json!([endpoint]).to_string(),
            )
            .map_err(|e| TransportError::Network(e.to_string()))?;
        config
            .insert_json5("scouting/multicast/enabled", "false")
            .map_err(|e| TransportError::Network(e.to_string()))?;
        let session = zenoh::open(config)
            .wait()
            .map_err(|e| TransportError::Network(e.to_string()))?;
        let inbox: Inbox = Arc::new(Mutex::new(VecDeque::new()));
        let states: States = Arc::new(Mutex::new(BTreeMap::new()));
        let stop = Arc::new(AtomicBool::new(false));
        let failed = Arc::new(AtomicBool::new(false));
        let base = format!("{prefix}/{id}");
        let queue = inbox.clone();
        let topic = base.clone();
        let subscriber = session
            .declare_subscriber(format!("{base}/**"))
            .callback(move |sample| {
                let Some(kind) = sample
                    .key_expr()
                    .as_str()
                    .strip_prefix(&format!("{topic}/"))
                else {
                    return;
                };
                if !crate::is_control_topic(kind) {
                    return;
                }
                let bytes = sample.payload().to_bytes();
                if bytes.len() > 2048 {
                    return;
                }
                let mut queue = queue.lock().unwrap();
                crate::enqueue_control_action(
                    &mut queue,
                    kind.into(),
                    bytes.to_vec(),
                    Instant::now(),
                );
            })
            .wait()
            .map_err(|e| TransportError::Network(e.to_string()))?;
        let batches: Arc<Mutex<BTreeMap<String, (Vec<String>, Instant)>>> = Default::default();
        let batch_state = batches.clone();
        let state = states.clone();
        let stopped = stop.clone();
        let fault = failed.clone();
        let fleet = format!("{prefix}/fleet/state");
        let worker = thread::spawn(move || {
            let mut last_batch = Instant::now() - Duration::from_secs(1);
            while !stopped.load(Ordering::Acquire) {
                if outbound && session.info().routers_zid().wait().next().is_none() {
                    fault.store(true, Ordering::Release);
                    break;
                }
                let (active, snapshots) = telemetry_snapshot(&mut state.lock().unwrap());
                // A stalled/backgrounded controller must not look freshly observed.
                if !active {
                    thread::sleep(Duration::from_millis(100));
                    continue;
                }
                if session
                    .put(
                        &fleet,
                        serde_json::json!({"count": 1,"max_count": 1,"ids": [id]}).to_string(),
                    )
                    .wait()
                    .is_err()
                {
                    fault.store(true, Ordering::Release);
                    break;
                }
                for (kind, value) in snapshots {
                    if session.put(format!("{base}/{kind}"), value).wait().is_err() {
                        fault.store(true, Ordering::Release);
                    }
                }
                if last_batch.elapsed() >= Duration::from_secs(1) {
                    last_batch = Instant::now();
                    let batches = batch_state.lock().unwrap().clone();
                    for (kind, (values, at)) in batches {
                        if at.elapsed() < TELEMETRY_TTL {
                            for value in values {
                                if session.put(format!("{base}/{kind}"), value).wait().is_err() {
                                    fault.store(true, Ordering::Release);
                                }
                            }
                        }
                    }
                }
                thread::sleep(Duration::from_millis(100));
            }
            drop(subscriber);
            let _ = session.close().wait();
        });
        Ok(Self {
            inbox,
            states,
            batches,
            stop,
            failed,
            worker: Some(worker),
            rover_id: id,
        })
    }
    pub fn take_actions(&self) -> Vec<(String, Vec<u8>, f64)> {
        if self.failed() {
            self.inbox.lock().unwrap().clear();
            return vec![];
        }
        crate::drain_control_actions(&mut self.inbox.lock().unwrap())
            .into_iter()
            .map(|(kind, payload, received)| (kind, payload, received.elapsed().as_secs_f64()))
            .collect()
    }
    pub fn publish(&self, kind: &str, value: String) {
        let _ = self.enqueue_state(kind, value);
    }
    /// Enqueue sample-driven visualization, reporting size/failure rejection.
    /// Acceptance is into the bounded latest-state slot, not network delivery.
    pub fn publish_visualization(&self, kind: &str, value: String) -> bool {
        visualization_topic(kind) && !self.failed() && self.enqueue_state(kind, value)
    }
    fn enqueue_state(&self, kind: &str, value: String) -> bool {
        // Dense visualization snapshots exceed the ordinary state-message budget.
        // Keep action/small telemetry limits unchanged and bound only these two topics higher.
        let limit = match kind {
            "map/occupancy" => 2_000_000,
            "pointcloud" => 262144,
            _ => 65536,
        };
        if value.len() <= limit {
            self.states
                .lock()
                .unwrap()
                .insert(kind.into(), (Some(value), Instant::now()));
            true
        } else {
            false
        }
    }
    /// Fair replay of bounded outstanding records on one contractual topic.
    pub fn publish_batch(&self, kind: &str, values: Vec<String>) {
        if kind == "search/report" && values.len() <= 128 && values.iter().all(|v| v.len() <= 65536)
        {
            self.batches
                .lock()
                .unwrap()
                .insert(kind.into(), (values, Instant::now()));
        }
    }
    pub fn failed(&self) -> bool {
        self.failed.load(Ordering::Acquire)
    }
}
impl Drop for ControlPlane {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(w) = self.worker.take() {
            let _ = w.join();
        }
    }
}
