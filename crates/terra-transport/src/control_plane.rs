//! Loopback-only dashboard endpoint. Transport queues intent, never motor authority.
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
pub struct ControlPlane {
    inbox: Inbox,
    states: Arc<Mutex<BTreeMap<String, String>>>,
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
        let mut config = zenoh::Config::default();
        config
            .insert_json5(
                "listen/endpoints",
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
        let states = Arc::new(Mutex::new(BTreeMap::<String, String>::new()));
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
                if !matches!(
                    kind,
                    "autonomy" | "safety" | "goal" | "goal/decision" | "teleop" | "cmd_vel"
                ) {
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
        let state = states.clone();
        let stopped = stop.clone();
        let fault = failed.clone();
        let fleet = format!("{prefix}/fleet/state");
        let worker = thread::spawn(move || {
            while !stopped.load(Ordering::Acquire) {
                let snapshots = state.lock().unwrap().clone();
                if session
                    .put(
                        &fleet,
                        serde_json::json!({"count":1,"max_count":1,"ids":[id]}).to_string(),
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
                thread::sleep(Duration::from_millis(100));
            }
            drop(subscriber);
            let _ = session.close().wait();
        });
        Ok(Self {
            inbox,
            states,
            stop,
            failed,
            worker: Some(worker),
            rover_id: id,
        })
    }
    pub fn take_actions(&self) -> Vec<(String, Vec<u8>, f64)> {
        crate::drain_control_actions(&mut self.inbox.lock().unwrap())
            .into_iter()
            .map(|(kind, payload, received)| (kind, payload, received.elapsed().as_secs_f64()))
            .collect()
    }
    pub fn publish(&self, kind: &str, value: String) {
        if value.len() <= 65536 {
            self.states.lock().unwrap().insert(kind.into(), value);
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
