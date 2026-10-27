//! Automatic rover connection and router-retry policy shared by both phone apps.
//!
//! Ports `TerraConnectionPolicy`, `TerraPairingCandidates`,
//! `TerraBluetoothFailurePolicy` and `TerraRouterSettings` from the iOS app.
//! Rover identifiers are opaque strings: CoreBluetooth UUIDs on iOS, device
//! addresses on Android. The apps keep the radio work and persistence and
//! supply monotonic time in seconds.
use crate::ControllerError;
use std::collections::BTreeSet;
use std::sync::{Arc, Mutex};

/// Back-off for Bluetooth reconnects and router retries, seconds.
const DELAYS: [f64; 5] = [1.0, 2.0, 4.0, 8.0, 15.0];
/// A pairing offer is replayed only while seen within this many seconds.
const CANDIDATE_LIFETIME: f64 = 10.0;

/// A rover heard during discovery.
#[derive(Clone, Debug, PartialEq, Eq, uniffi::Record)]
pub struct DiscoveredRoverInfo {
    /// Platform identifier.
    pub id: String,
    /// Advertised name.
    pub name: String,
    /// The rover is in pairing mode.
    pub pairing: bool,
}

/// What to do with a discovered rover.
#[derive(Clone, Debug, PartialEq, Eq, uniffi::Enum)]
pub enum ConnectionDecision {
    /// Ask the person whether to pair with this rover.
    Offer { id: String },
    /// Connect to the preferred rover now.
    Connect { id: String },
}

#[derive(Default)]
struct Policy {
    generation: u64,
    enabled: bool,
    preferred: Option<String>,
    connected: Option<String>,
    attempting: Option<String>,
    offered: Option<String>,
    declined: BTreeSet<String>,
    deadline: Option<f64>,
    retry_at: Option<f64>,
    ble_retries: usize,
    router_retries: usize,
    awaiting_operation: bool,
}

impl Policy {
    fn stop(&mut self) {
        self.generation = self.generation.wrapping_add(1);
        self.enabled = false;
        self.connected = None;
        self.attempting = None;
        self.offered = None;
        self.deadline = None;
        self.retry_at = None;
        self.awaiting_operation = false;
    }

    fn authenticated(&mut self, id: String) {
        self.preferred = Some(id.clone());
        self.connected = Some(id);
        self.attempting = None;
        self.deadline = None;
        self.retry_at = None;
        self.awaiting_operation = false;
        self.ble_retries = 0;
    }
}

/// Snapshot of the policy for the UI and the radio adapter.
#[derive(Clone, Debug, PartialEq, Eq, uniffi::Record)]
pub struct ConnectionPolicyState {
    /// Changes whenever pending radio callbacks become stale.
    pub generation: u64,
    /// Automatic connection is running.
    pub enabled: bool,
    /// Rover to reconnect to.
    pub preferred: Option<String>,
    /// Authenticated rover.
    pub connected: Option<String>,
    /// Rover being connected.
    pub attempting: Option<String>,
    /// Rover offered for pairing.
    pub offered: Option<String>,
}

/// Automatic Bluetooth connection plus router retry back-off.
/// Calls are serialized by a mutex.
#[derive(uniffi::Object)]
pub struct ConnectionPolicy {
    state: Mutex<Policy>,
}

#[uniffi::export]
impl ConnectionPolicy {
    #[uniffi::constructor]
    /// Disabled, nothing preferred.
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            state: Mutex::new(Policy::default()),
        })
    }

    /// Current state.
    pub fn state(&self) -> Result<ConnectionPolicyState, ControllerError> {
        let p = self.lock()?;
        Ok(ConnectionPolicyState {
            generation: p.generation,
            enabled: p.enabled,
            preferred: p.preferred.clone(),
            connected: p.connected.clone(),
            attempting: p.attempting.clone(),
            offered: p.offered.clone(),
        })
    }

    /// Start automatic connection, reconnecting to `preferred` if given.
    pub fn begin(&self, preferred: Option<String>) -> Result<(), ControllerError> {
        let mut p = self.lock()?;
        p.stop();
        p.enabled = true;
        p.preferred = preferred;
        p.declined.clear();
        p.ble_retries = 0;
        p.router_retries = 0;
        Ok(())
    }

    /// An already-connected link to the preferred rover becomes the connection.
    pub fn adopt_authenticated(&self, id: String) -> Result<bool, ControllerError> {
        let mut p = self.lock()?;
        if !p.enabled || p.preferred.as_deref() != Some(id.as_str()) {
            return Ok(false);
        }
        p.authenticated(id);
        Ok(true)
    }

    /// Stop automatic connection and invalidate pending callbacks.
    pub fn stop(&self) -> Result<(), ControllerError> {
        self.lock()?.stop();
        Ok(())
    }

    /// Connect to the preferred rover, or offer one pairing rover at a time.
    pub fn discover(
        &self,
        rover: DiscoveredRoverInfo,
    ) -> Result<Option<ConnectionDecision>, ControllerError> {
        let mut p = self.lock()?;
        if !p.enabled || p.connected.is_some() || p.attempting.is_some() || p.retry_at.is_some() {
            return Ok(None);
        }
        if p.preferred.as_deref() == Some(rover.id.as_str()) && !rover.pairing {
            p.offered = None;
            p.attempting = Some(rover.id.clone());
            return Ok(Some(ConnectionDecision::Connect { id: rover.id }));
        }
        if p.awaiting_operation
            || !rover.pairing
            || p.declined.contains(&rover.id)
            || p.offered.is_some()
        {
            return Ok(None);
        }
        p.offered = Some(rover.id.clone());
        Ok(Some(ConnectionDecision::Offer { id: rover.id }))
    }

    /// The person declined pairing with `id`.
    pub fn decline(&self, id: String) -> Result<(), ControllerError> {
        let mut p = self.lock()?;
        if p.offered.as_deref() == Some(id.as_str()) {
            p.offered = None;
        }
        p.declined.insert(id);
        Ok(())
    }

    /// The person accepted pairing; the attempt expires after 15 s.
    pub fn accept(&self, id: String, now: f64) -> Result<(), ControllerError> {
        let mut p = self.lock()?;
        p.offered = None;
        p.declined.insert(id.clone());
        p.attempting = Some(id);
        p.deadline = Some(now + 15.0);
        Ok(())
    }

    /// Pairing finished; wait up to 30 s for the rover to come back unpaired.
    pub fn paired(&self, id: String, now: f64) -> Result<(), ControllerError> {
        let mut p = self.lock()?;
        p.preferred = Some(id);
        p.attempting = None;
        p.connected = None;
        p.awaiting_operation = true;
        p.deadline = Some(now + 30.0);
        p.retry_at = None;
        Ok(())
    }

    /// The link authenticated.
    pub fn authenticated(&self, id: String) -> Result<(), ControllerError> {
        self.lock()?.authenticated(id);
        Ok(())
    }

    /// The link failed. Retryable failures back off 1, 2, 4, 8, 15 s; others stop.
    pub fn failed(&self, retryable: bool, now: f64) -> Result<(), ControllerError> {
        let mut p = self.lock()?;
        p.generation = p.generation.wrapping_add(1);
        p.connected = None;
        p.attempting = None;
        p.offered = None;
        p.deadline = None;
        if p.enabled && retryable {
            p.retry_at = Some(now + DELAYS[p.ble_retries.min(4)]);
            p.ble_retries += 1;
        } else {
            p.retry_at = None;
            p.enabled = false;
        }
        Ok(())
    }

    /// True once when the back-off has elapsed.
    pub fn retry_due(&self, now: f64) -> Result<bool, ControllerError> {
        let mut p = self.lock()?;
        match p.retry_at {
            Some(at) if p.enabled && now >= at => {
                p.retry_at = None;
                Ok(true)
            }
            _ => Ok(false),
        }
    }

    /// The current connect or pairing attempt ran out of time.
    pub fn attempt_expired(&self, now: f64) -> Result<bool, ControllerError> {
        let p = self.lock()?;
        Ok(p.enabled && p.deadline.is_some_and(|d| now >= d))
    }

    /// Seconds to wait before the next router (ARGOS) attempt.
    pub fn next_router_delay(&self) -> Result<f64, ControllerError> {
        let mut p = self.lock()?;
        let delay = DELAYS[p.router_retries.min(4)];
        p.router_retries += 1;
        Ok(delay)
    }

    /// The router connected; the next failure starts the back-off again.
    pub fn router_connected(&self) -> Result<(), ControllerError> {
        self.lock()?.router_retries = 0;
        Ok(())
    }
}

impl ConnectionPolicy {
    fn lock(&self) -> Result<std::sync::MutexGuard<'_, Policy>, ControllerError> {
        self.state.lock().map_err(|_| ControllerError::Internal)
    }
}

/// Pairing rovers in discovery order, dropped 10 s after they were last heard.
#[derive(uniffi::Object)]
pub struct PairingCandidates {
    entries: Mutex<Vec<(DiscoveredRoverInfo, f64)>>,
}

#[uniffi::export]
impl PairingCandidates {
    #[uniffi::constructor]
    /// Empty.
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            entries: Mutex::new(Vec::new()),
        })
    }

    /// Record a discovery. A rover heard outside pairing mode is removed.
    pub fn observe(&self, rover: DiscoveredRoverInfo, now: f64) -> Result<(), ControllerError> {
        let mut entries = self.entries.lock().map_err(|_| ControllerError::Internal)?;
        entries.retain(|(_, seen)| now - seen < CANDIDATE_LIFETIME);
        if !rover.pairing {
            entries.retain(|(r, _)| r.id != rover.id);
            return Ok(());
        }
        match entries.iter_mut().find(|(r, _)| r.id == rover.id) {
            Some(entry) => *entry = (rover, now),
            None => entries.push((rover, now)),
        }
        Ok(())
    }

    /// Rovers heard in pairing mode within the last 10 s.
    pub fn available(&self, now: f64) -> Result<Vec<DiscoveredRoverInfo>, ControllerError> {
        let entries = self.entries.lock().map_err(|_| ControllerError::Internal)?;
        Ok(entries
            .iter()
            .filter(|(_, seen)| now >= *seen && now - seen < CANDIDATE_LIFETIME)
            .map(|(r, _)| r.clone())
            .collect())
    }

    /// Forget every candidate.
    pub fn clear(&self) -> Result<(), ControllerError> {
        self.entries
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .clear();
        Ok(())
    }
}

#[uniffi::export]
/// Radio availability and permission failures retry; protocol and
/// authentication failures stop automatic connection.
pub fn bluetooth_failure_retryable(reason: String) -> bool {
    ![
        "Malformed",
        "Encrypted",
        "Invalid",
        "Terra service unavailable",
        "Status unavailable",
    ]
    .iter()
    .any(|marker| reason.contains(marker))
}

/// Validated router settings.
#[derive(Clone, Debug, PartialEq, Eq, uniffi::Record)]
pub struct RouterSettings {
    /// `tcp/host:port` or `tls/host:port`; IPv6 hosts in brackets.
    pub endpoint: String,
    /// Topic prefix without wildcards or leading/trailing slashes.
    pub prefix: String,
    /// Rover number.
    pub rover_id: u64,
}

#[uniffi::export]
/// Trim and validate router settings; `None` when any part is invalid.
pub fn parse_router_settings(
    endpoint: String,
    prefix: String,
    rover_id: String,
) -> Option<RouterSettings> {
    let endpoint = endpoint.trim();
    let prefix = prefix.trim();
    let id = rover_id.trim().parse::<u64>().ok()?;
    let rest = endpoint
        .strip_prefix("tcp/")
        .or_else(|| endpoint.strip_prefix("tls/"))?;
    if endpoint.chars().any(char::is_whitespace) {
        return None;
    }
    let (host, port) = rest.rsplit_once(':')?;
    if !port.chars().all(|c| c.is_ascii_digit()) || port.parse::<u16>().ok()? == 0 {
        return None;
    }
    if prefix.is_empty()
        || prefix.contains('*')
        || prefix.contains('#')
        || prefix.starts_with('/')
        || prefix.ends_with('/')
        || prefix.contains("//")
    {
        return None;
    }
    if host.is_empty() || host.contains('/') {
        return None;
    }
    if host.contains(':') {
        if !(host.starts_with('[') && host.ends_with(']') && host.len() > 2) {
            return None;
        }
    } else if host.contains('[') || host.contains(']') {
        return None;
    }
    Some(RouterSettings {
        endpoint: endpoint.to_owned(),
        prefix: prefix.to_owned(),
        rover_id: id,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rover(id: &str, pairing: bool) -> DiscoveredRoverInfo {
        DiscoveredRoverInfo {
            id: id.into(),
            name: format!("terra-{id}"),
            pairing,
        }
    }
    fn connect(id: &str) -> Option<ConnectionDecision> {
        Some(ConnectionDecision::Connect { id: id.into() })
    }
    fn offer(id: &str) -> Option<ConnectionDecision> {
        Some(ConnectionDecision::Offer { id: id.into() })
    }

    #[test]
    /// Ports TerraConnectionLifecycleTests for the policy and pairing candidates.
    fn connection_lifecycle() {
        let p = ConnectionPolicy::new();
        p.begin(Some("a".into())).unwrap();
        assert_eq!(p.discover(rover("b", false)).unwrap(), None);
        assert_eq!(p.discover(rover("b", true)).unwrap(), offer("b"));
        assert_eq!(p.discover(rover("b", true)).unwrap(), None);
        p.decline("b".into()).unwrap();
        assert_eq!(p.discover(rover("b", true)).unwrap(), None);
        assert_eq!(p.discover(rover("a", false)).unwrap(), connect("a"));
        assert_eq!(p.discover(rover("a", false)).unwrap(), None);
        p.failed(true, 10.0).unwrap();
        assert!(!p.retry_due(10.5).unwrap());
        assert!(p.retry_due(11.0).unwrap());
        let delays: Vec<f64> = (0..6).map(|_| p.next_router_delay().unwrap()).collect();
        assert_eq!(delays, [1.0, 2.0, 4.0, 8.0, 15.0, 15.0]);
        let generation = p.state().unwrap().generation;
        p.stop().unwrap();
        assert!(p.state().unwrap().generation != generation && !p.retry_due(100.0).unwrap());
        p.begin(None).unwrap();
        assert_eq!(p.discover(rover("b", true)).unwrap(), offer("b"));
        p.accept("b".into(), 0.0).unwrap();
        assert!(!p.attempt_expired(14.0).unwrap());
        assert!(p.attempt_expired(15.0).unwrap());
        p.paired("b".into(), 20.0).unwrap();
        assert_eq!(p.state().unwrap().preferred.as_deref(), Some("b"));
        assert_eq!(p.discover(rover("b", true)).unwrap(), None);
        assert_eq!(p.discover(rover("b", false)).unwrap(), connect("b"));
        p.authenticated("b".into()).unwrap();
        assert_eq!(p.state().unwrap().connected.as_deref(), Some("b"));
        p.stop().unwrap();
        assert_eq!(p.discover(rover("a", true)).unwrap(), None);
        p.begin(None).unwrap();
        assert!(!p.adopt_authenticated("a".into()).unwrap());
        assert_eq!(p.state().unwrap().connected, None);
        p.begin(Some("a".into())).unwrap();
        assert!(p.adopt_authenticated("a".into()).unwrap());
        assert_eq!(p.state().unwrap().connected.as_deref(), Some("a"));

        let candidates = PairingCandidates::new();
        candidates.observe(rover("a", true), 0.0).unwrap();
        candidates.observe(rover("b", true), 1.0).unwrap();
        candidates.observe(rover("b", false), 2.0).unwrap();
        let ids: Vec<String> = candidates
            .available(2.0)
            .unwrap()
            .into_iter()
            .map(|r| r.id)
            .collect();
        assert_eq!(ids, ["a"]);
        assert!(candidates.available(11.0).unwrap().is_empty());

        p.begin(None).unwrap();
        p.accept("a".into(), 0.0).unwrap();
        p.stop().unwrap();
        let s = p.state().unwrap();
        assert!(s.attempting.is_none() && !s.enabled && !p.attempt_expired(20.0).unwrap());
        // A non-retryable failure stops automatic connection.
        p.begin(None).unwrap();
        p.failed(false, 0.0).unwrap();
        assert!(!p.state().unwrap().enabled && !p.retry_due(100.0).unwrap());
    }

    #[test]
    /// Failure classes and router settings, as in the Swift tests.
    fn failures_and_router_settings() {
        assert!(bluetooth_failure_retryable("Bluetooth unavailable".into()));
        assert!(bluetooth_failure_retryable(
            "Bluetooth permission required".into()
        ));
        assert!(!bluetooth_failure_retryable(
            "Malformed Bluetooth message".into()
        ));
        assert!(!bluetooth_failure_retryable(
            "Encrypted read/notification failed".into()
        ));
        let parse =
            |e: &str, p: &str, id: &str| parse_router_settings(e.into(), p.into(), id.into());
        assert_eq!(
            parse("tcp/127.0.0.1:7448", "terra/phone", "7")
                .unwrap()
                .rover_id,
            7
        );
        assert!(parse("tcp/[::1]:7448", "terra/phone", "0").is_some());
        assert_eq!(
            parse(" tls/argos.example:7448 ", " terra/rover ", " 3 ")
                .unwrap()
                .endpoint,
            "tls/argos.example:7448"
        );
        for endpoint in [
            "",
            "tcp/:7448",
            "tcp/host:0",
            "tcp/host:7448:",
            "tcp/host:65536",
            "udp/host:7448",
            "tcp/a b:7448",
            "tcp/::1:7448",
        ] {
            assert!(parse(endpoint, "terra/phone", "7").is_none(), "{endpoint}");
        }
        assert!(parse("tcp/host:7448", "terra/*", "7").is_none());
        assert!(parse("tcp/host:7448", "/terra", "7").is_none());
        assert!(parse("tcp/host:7448", "terra/phone", "-1").is_none());
    }
}
