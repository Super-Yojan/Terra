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
type SignalingInbox = Arc<Mutex<VecDeque<(String, Vec<u8>)>>>;
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

/// Dashboard intent inbox and TTL-bound telemetry publisher for one rover.
/// Control actions are bounded and drained once; ordinary state is replayed
/// while fresh. Visualization snapshots are consumed once per producer sample.
/// Dropping the service stops and joins its network worker.
pub struct ControlPlane {
    inbox: Inbox,
    signaling: SignalingInbox,
    states: States,
    /// Cloned Zenoh session used for one-shot signaling puts. The worker owns
    /// the session that publishes latched telemetry and closes the link.
    session: zenoh::Session,
    base: String,
    batches: Arc<Mutex<BTreeMap<String, (Vec<String>, Instant)>>>,
    stop: Arc<AtomicBool>,
    failed: Arc<AtomicBool>,
    worker: Option<JoinHandle<()>>,
    /// Rover identity used in topic paths and fleet advertisements.
    pub rover_id: u64,
}
impl ControlPlane {
    /// Open an explicit loopback TCP dashboard endpoint. Requires the literal
    /// `tcp/127.0.0.1:` address form and a valid non-wildcard key prefix.
    /// Returns `InvalidSettings` for rejected configuration or `Network` on open failure.
    pub fn listen(endpoint: &str, prefix: &str, id: u64) -> Result<Self, TransportError> {
        crate::validate(endpoint, prefix)?;
        if !endpoint.starts_with("tcp/127.0.0.1:") {
            return Err(TransportError::InvalidSettings);
        }
        Self::open(endpoint, prefix, id, false, None)
    }
    /// Outbound TCP session, suitable for an explicitly configured Tailscale router.
    /// No listeners, multicast discovery, or automatic application command retries.
    /// A TCP/TLS endpoint and non-wildcard key prefix are required. Outbound open
    /// has a three-second timeout. Returns `InvalidSettings` for rejected settings
    /// or `Network` for configuration, connection, or subscriber failure.
    pub fn connect(endpoint: &str, prefix: &str, id: u64) -> Result<Self, TransportError> {
        crate::validate(endpoint, prefix)?;
        Self::open(endpoint, prefix, id, true, None)
    }
    /// Open the same outbound session as [`Self::connect`] with Zenoh user/password
    /// authentication. An empty `user` omits credentials. No application intent is retried.
    /// Returns `InvalidSettings` for rejected settings or `Network` for open failure.
    pub fn connect_authenticated(
        endpoint: &str,
        prefix: &str,
        id: u64,
        user: &str,
        password: &str,
    ) -> Result<Self, TransportError> {
        crate::validate(endpoint, prefix)?;
        Self::open(endpoint, prefix, id, true, Some((user, password)))
    }
    fn open(
        endpoint: &str,
        prefix: &str,
        id: u64,
        outbound: bool,
        credentials: Option<(&str, &str)>,
    ) -> Result<Self, TransportError> {
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
        if let Some((user, password)) = credentials.filter(|(u, _)| !u.is_empty()) {
            for (key, value) in [
                ("transport/auth/usrpwd/user", user),
                ("transport/auth/usrpwd/password", password),
            ] {
                config
                    .insert_json5(key, &serde_json::json!(value).to_string())
                    .map_err(|e| TransportError::Network(e.to_string()))?;
            }
        }
        let session = zenoh::open(config)
            .wait()
            .map_err(|e| TransportError::Network(e.to_string()))?;
        let inbox: Inbox = Arc::new(Mutex::new(VecDeque::new()));
        let signaling: SignalingInbox = Arc::new(Mutex::new(VecDeque::new()));
        let states: States = Arc::new(Mutex::new(BTreeMap::new()));
        let stop = Arc::new(AtomicBool::new(false));
        let failed = Arc::new(AtomicBool::new(false));
        let base = format!("{prefix}/{id}");
        let queue = inbox.clone();
        let signaling_queue = signaling.clone();
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
                let bytes = sample.payload().to_bytes();
                // Video signaling never enters the control inbox. A large SDP
                // cannot evict a stop, and a webrtc payload cannot be decoded
                // as teleop, hardware, or safety.
                if crate::signaling::webrtc_inbound(kind) {
                    if bytes.is_empty()
                        || bytes.len() > crate::signaling::WEBRTC_SIGNALING_MAX_BYTES
                    {
                        return;
                    }
                    let mut queue = signaling_queue.lock().unwrap();
                    crate::signaling::enqueue_signaling(&mut queue, kind.into(), bytes.to_vec());
                    return;
                }
                if !crate::is_control_topic(kind) {
                    return;
                }
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
        let publication = session.clone();
        let key_base = base.clone();
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
            signaling,
            states,
            session: publication,
            base: key_base,
            batches,
            stop,
            failed,
            worker: Some(worker),
            rover_id: id,
        })
    }
    /// Drain received intent as `(topic suffix, payload bytes, age_seconds)`.
    /// Recognized stops are last so they dominate the tick. A failed session clears
    /// its inbox and returns an empty vector; payload validation belongs to the consumer.
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
    /// Replace the latest telemetry slot and refresh its 500 ms TTL.
    /// Drops oversized values silently: occupancy is capped at 2,000,000 bytes,
    /// point clouds at 262144 bytes, and other suffixes at 65536 bytes.
    /// Ordinary state can replay at 10 Hz; visualization is taken once per update.
    /// Does not parse JSON or acknowledge delivery. WebRTC signaling suffixes are
    /// refused here so an offer is not replayed as latched telemetry.
    pub fn publish(&self, kind: &str, value: String) {
        let _ = self.enqueue_state(kind, value);
    }
    /// Drain dashboard-to-phone WebRTC signaling. Each item is `(topic suffix, payload)`.
    /// A failed session clears the queue and returns nothing. These samples are
    /// not control actions and do not grant motion or hardware authority.
    pub fn take_signaling(&self) -> Vec<(String, Vec<u8>)> {
        if self.failed() {
            self.signaling.lock().unwrap().clear();
            return Vec::new();
        }
        std::mem::take(&mut *self.signaling.lock().unwrap())
            .into_iter()
            .collect()
    }
    /// Put one phone-to-dashboard signaling sample immediately.
    /// Accepts only a valid `webrtc/offer`, `webrtc/ice/phone`, or `webrtc/state`
    /// sample of at most [`crate::WEBRTC_SIGNALING_MAX_BYTES`].
    /// The sample is not stored for telemetry replay. Returns false when the
    /// kind, JSON, size, or link is rejected. Delivery is still best effort.
    /// This does not publish teleop or hardware.
    pub fn publish_signaling(&self, kind: &str, value: &str) -> bool {
        if self.failed() || !crate::signaling::accept_phone_outbound(kind, value) {
            return false;
        }
        self.session
            .put(format!("{}/{kind}", self.base), value)
            .wait()
            .is_ok()
    }
    /// Enqueue sample-driven visualization, reporting size/failure rejection.
    /// Acceptance is into the bounded latest-state slot, not network delivery.
    /// Only occupancy, pointcloud, and their heartbeat suffixes are accepted.
    /// Uses [`Self::publish`] byte caps and returns false for an unsupported topic,
    /// failed link, or oversized value. JSON/schema validation belongs to the caller.
    pub fn publish_visualization(&self, kind: &str, value: String) -> bool {
        visualization_topic(kind) && !self.failed() && self.enqueue_state(kind, value)
    }
    fn enqueue_state(&self, kind: &str, value: String) -> bool {
        if crate::signaling::is_webrtc_topic(kind) {
            return false;
        }
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
    /// Only `search/report` is accepted: at most 128 records, each at most 65536
    /// bytes. Invalid batches are ignored. The latest batch has a 500 ms TTL and
    /// can replay once per second while ordinary producer telemetry keeps the
    /// worker active. Callers refresh unacknowledged reports explicitly.
    pub fn publish_batch(&self, kind: &str, values: Vec<String>) {
        if kind == "search/report" && values.len() <= 128 && values.iter().all(|v| v.len() <= 65536)
        {
            self.batches
                .lock()
                .unwrap()
                .insert(kind.into(), (values, Instant::now()));
        }
    }
    /// Return the latched router-loss or publication-failure flag.
    /// Once set, intent draining discards queued commands; this service does not reconnect.
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
