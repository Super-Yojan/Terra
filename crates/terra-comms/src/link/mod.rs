//! The Zenoh link (feature `zenoh`).
//!
//! [`connect`] returns at once; the session is opened on a background
//! thread. Subscriptions asked for before it is open are declared as soon as
//! it is. Network errors are never returned: [`Link::connected`] and
//! `stats()` are the only view into them.
use std::collections::{BTreeMap, HashMap};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread;
use std::time::Duration;

use zenoh::pubsub::Subscriber;
use zenoh::sample::Sample;
use zenoh::{Config, Session, Wait};

use crate::codec::Message;
use crate::config::{CommsConfig, ConfigError};
use crate::topics::{Topic, Topics};

mod base;
mod rover;

pub use base::{BaseLink, Subscription};
pub use rover::RoverLink;

/// Open a Zenoh client session in the background and return at once.
/// Retries forever if the router is down. Fails only on a bad config.
pub fn connect(config: &CommsConfig) -> Result<Link, ConfigError> {
    config.check()?;
    let zenoh_config = zenoh_config(config)?;
    let core = Arc::new(Core {
        topics: Topics::new(&config.prefix)?,
        config: config.clone(),
        state: Mutex::new(State::default()),
        shutdown: AtomicBool::new(false),
        counters: Arc::new(Counters::default()),
        connected: AtomicBool::new(false),
    });
    let opener = core.clone();
    thread::Builder::new()
        .name("terra-comms-open".into())
        .spawn(move || run_session(&opener, zenoh_config))
        .map_err(|e| ConfigError::new(format!("cannot start the comms thread: {e}")))?;
    Ok(Link { handle: Handle(core) })
}

/// A session, before choosing the rover or the base side.
pub struct Link {
    handle: Handle,
}

impl Link {
    /// The rover side. Needs `rover_id` in the config.
    pub fn rover(self) -> Result<RoverLink, ConfigError> {
        let rover_id = self
            .handle
            .0
            .config
            .rover_id
            .ok_or_else(|| ConfigError::new("[comms] rover_id is required on the rover"))?;
        Ok(RoverLink::start(self.handle, rover_id))
    }

    /// The base-station side.
    pub fn base(self) -> BaseLink {
        BaseLink::start(self.handle)
    }

    /// Whether a router is connected. For a status light, not for control.
    pub fn connected(&self) -> bool {
        self.handle.0.connected()
    }

    pub fn stats(&self) -> Stats {
        self.handle.0.counters.snapshot()
    }
}

/// Counts per topic, since `connect`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TopicStats {
    /// Handed to Zenoh while a router was connected.
    pub sent: u64,
    /// Not sent: replaced by a newer message before it went out, no router
    /// connected (commands), or the send failed.
    pub dropped: u64,
    /// Received and decoded.
    pub received: u64,
    /// Received but not decodable (or under a foreign key); dropped.
    pub bad_payload: u64,
}

pub type Stats = BTreeMap<Topic, TopicStats>;

// ------------------------------------------------------------------ internals

/// Owns the core: dropping it closes the session.
pub(crate) struct Handle(pub(crate) Arc<Core>);

impl Drop for Handle {
    fn drop(&mut self) {
        self.0.shutdown();
    }
}

pub(crate) struct Core {
    pub(crate) config: CommsConfig,
    pub(crate) topics: Topics,
    pub(crate) counters: Arc<Counters>,
    state: Mutex<State>,
    shutdown: AtomicBool,
    /// Cached router connection, refreshed every `CONNECTION_POLL`.
    connected: AtomicBool,
}

/// How often the connection state is refreshed.
const CONNECTION_POLL: Duration = Duration::from_millis(250);

/// One received sample.
pub(crate) struct Received<'a> {
    pub key: &'a str,
    pub payload: &'a [u8],
    /// How long the message had existed when the rover sent it (rover →
    /// base topics; see `age_attachment`). Zero if absent.
    pub age: Duration,
}

pub(crate) type RawCallback = Arc<dyn Fn(Received<'_>) + Send + Sync>;

/// The rover attaches the message's age, as little-endian `u64`
/// microseconds, to everything it sends: time since the control tick
/// published it. A repeat carries its real age, so the base (even one that
/// just started) can tell it from a fresh message without comparing clocks.
pub(crate) fn age_attachment(age: Duration) -> Vec<u8> {
    (age.as_micros().min(u64::MAX as u128) as u64).to_le_bytes().to_vec()
}

fn parse_age(attachment: Option<&[u8]>) -> Duration {
    attachment
        .and_then(|a| <[u8; 8]>::try_from(a).ok())
        .map_or(Duration::ZERO, |b| Duration::from_micros(u64::from_le_bytes(b)))
}

#[derive(Default)]
struct State {
    session: Option<Session>,
    subs: HashMap<u64, Sub>,
    next_id: u64,
}

struct Sub {
    key: String,
    callback: RawCallback,
    /// `None` until declared on the open session (or if declaring failed).
    handle: Option<Subscriber<()>>,
}

impl Core {
    fn lock(&self) -> MutexGuard<'_, State> {
        self.state.lock().unwrap_or_else(|e| e.into_inner())
    }

    pub(crate) fn session(&self) -> Option<Session> {
        self.lock().session.clone()
    }

    /// Cached; cheap enough to call per message.
    pub(crate) fn connected(&self) -> bool {
        self.connected.load(Ordering::Acquire)
    }

    /// Subscribe to `key`, now or once the session is open. Zenoh is never
    /// called with the core lock held.
    pub(crate) fn add_sub(&self, key: String, callback: RawCallback) -> u64 {
        let (id, session) = {
            let mut state = self.lock();
            let id = state.next_id;
            state.next_id += 1;
            state.subs.insert(id, Sub { key: key.clone(), callback: callback.clone(), handle: None });
            // If there's no session yet, `install` will declare it.
            (id, state.session.clone())
        };
        if let Some(session) = session {
            self.attach(id, declare(&session, &key, &callback));
        }
        id
    }

    /// Store a declared subscriber, or drop it if the subscription was
    /// removed meanwhile.
    fn attach(&self, id: u64, handle: Option<Subscriber<()>>) {
        let leftover = {
            let mut state = self.lock();
            match state.subs.get_mut(&id) {
                Some(sub) if sub.handle.is_none() => {
                    sub.handle = handle;
                    None
                }
                _ => handle,
            }
        };
        drop(leftover); // undeclares, outside the lock
    }

    pub(crate) fn remove_sub(&self, id: u64) {
        let sub = self.lock().subs.remove(&id);
        drop(sub); // undeclares, outside the lock
    }

    fn install(&self, session: Session) -> bool {
        let pending: Vec<(u64, String, RawCallback)> = {
            let mut state = self.lock();
            if self.shutdown.load(Ordering::Acquire) {
                drop(state);
                let _ = session.close().wait();
                return false;
            }
            state.session = Some(session.clone());
            state.subs.iter().map(|(id, s)| (*id, s.key.clone(), s.callback.clone())).collect()
        };
        for (id, key, callback) in pending {
            self.attach(id, declare(&session, &key, &callback));
        }
        true
    }

    fn shutdown(&self) {
        self.shutdown.store(true, Ordering::Release);
        self.connected.store(false, Ordering::Release);
        let (session, subs) = {
            let mut state = self.lock();
            (state.session.take(), std::mem::take(&mut state.subs))
        };
        drop(subs);
        if let Some(session) = session {
            let _ = session.close().wait();
        }
    }
}

fn declare(session: &Session, key: &str, callback: &RawCallback) -> Option<Subscriber<()>> {
    let callback = callback.clone();
    let result = session
        .declare_subscriber(key.to_owned())
        .callback(move |sample: Sample| {
            let payload = sample.payload().to_bytes();
            let attachment = sample.attachment().map(|a| a.to_bytes());
            callback(Received {
                key: sample.key_expr().as_str(),
                payload: &payload,
                age: parse_age(attachment.as_deref()),
            });
        })
        .wait();
    match result {
        Ok(subscriber) => Some(subscriber),
        Err(e) => {
            eprintln!("terra-comms: cannot subscribe to {key}: {e}");
            None
        }
    }
}

/// Body of the background thread: open the session (retrying on errors),
/// hand it to the core, then keep the cached connection state fresh until
/// shutdown.
fn run_session(core: &Core, config: Config) {
    let mut backoff = Duration::from_millis(500);
    let session = loop {
        if core.shutdown.load(Ordering::Acquire) {
            return;
        }
        match zenoh::open(config.clone()).wait() {
            Ok(session) => break session,
            Err(e) => {
                eprintln!("terra-comms: cannot open the Zenoh session, retrying in {backoff:?}: {e}");
                thread::sleep(backoff);
                backoff = (backoff * 2).min(Duration::from_secs(10));
            }
        }
    };
    if !core.install(session.clone()) {
        return;
    }
    while !core.shutdown.load(Ordering::Acquire) {
        let up = session.info().routers_zid().wait().next().is_some();
        if !core.shutdown.load(Ordering::Acquire) {
            core.connected.store(up, Ordering::Release);
        }
        thread::sleep(CONNECTION_POLL);
    }
}

/// Fixed settings: client mode, multicast scouting off, keep retrying when
/// the router is down, user/password auth when `user` is set.
fn zenoh_config(c: &CommsConfig) -> Result<Config, ConfigError> {
    let json = |v: &str| serde_json::to_string(v).expect("string to JSON");
    let mut config = Config::default();
    let mut set = |key: &str, value: &str| {
        config
            .insert_json5(key, value)
            .map_err(|e| ConfigError::new(format!("[comms] {key}: {e}")))
    };
    set("mode", r#""client""#)?;
    set("connect/endpoints", &format!("[{}]", json(&c.endpoint)))?;
    set("connect/exit_on_failure", "false")?;
    set("scouting/multicast/enabled", "false")?;
    if let Some(user) = &c.user {
        set("transport/auth/usrpwd/user", &json(user))?;
        set("transport/auth/usrpwd/password", &json(c.password.as_deref().unwrap_or("")))?;
    }
    Ok(config)
}

#[derive(Default)]
pub(crate) struct Counters {
    topics: [TopicCounters; Topic::COUNT],
}

#[derive(Default)]
struct TopicCounters {
    sent: AtomicU64,
    dropped: AtomicU64,
    received: AtomicU64,
    bad_payload: AtomicU64,
}

impl Counters {
    fn get(&self, topic: Topic) -> &TopicCounters {
        &self.topics[topic.index()]
    }
    pub(crate) fn sent(&self, topic: Topic) {
        self.get(topic).sent.fetch_add(1, Ordering::Relaxed);
    }
    pub(crate) fn dropped(&self, topic: Topic) {
        self.get(topic).dropped.fetch_add(1, Ordering::Relaxed);
    }
    pub(crate) fn bad_payload(&self, topic: Topic) {
        self.get(topic).bad_payload.fetch_add(1, Ordering::Relaxed);
    }

    /// Decode, counting the outcome. Bad payloads never reach the caller.
    pub(crate) fn decode<M: Message>(&self, bytes: &[u8]) -> Option<M> {
        match M::decode(bytes) {
            Ok(msg) => {
                self.get(M::TOPIC).received.fetch_add(1, Ordering::Relaxed);
                Some(msg)
            }
            Err(_) => {
                self.bad_payload(M::TOPIC);
                None
            }
        }
    }

    fn snapshot(&self) -> Stats {
        Topic::ALL
            .into_iter()
            .map(|topic| {
                let c = self.get(topic);
                let stats = TopicStats {
                    sent: c.sent.load(Ordering::Relaxed),
                    dropped: c.dropped.load(Ordering::Relaxed),
                    received: c.received.load(Ordering::Relaxed),
                    bad_payload: c.bad_payload.load(Ordering::Relaxed),
                };
                (topic, stats)
            })
            .collect()
    }
}
