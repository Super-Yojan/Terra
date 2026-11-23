//! Base-station side: keeps the latest message of each kind from every
//! rover, and sends commands.
use std::any::Any;
use std::collections::{HashMap, VecDeque};
use std::sync::{Arc, Condvar, Mutex, MutexGuard};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use zenoh::Wait;
use zenoh::qos::{CongestionControl, Priority};

use super::{Core, Handle, RawCallback, Received, Stats};
use crate::codec::{Downlink, Uplink};
use crate::messages::{
    Capabilities, Depth, Estop, MeshChunk, Occupancy, PlannedPath, Pose, RemoteState, Status,
    TaskStatus, WorldMap,
};
use crate::topics::{Topic, Topics};
use crate::{ESTOP_REPEAT, ROVER_SEEN_WINDOW};

pub struct BaseLink {
    handle: Handle,
    state: Arc<BaseState>,
    commands: Arc<Commands>,
    sender: Option<JoinHandle<()>>,
}

impl BaseLink {
    pub(crate) fn start(handle: Handle) -> Self {
        let state = Arc::new(BaseState::default());
        let core = &handle.0;
        // Always watched. `Depth` is pulled only while someone subscribes.
        watch::<Capabilities>(core, &state);
        watch::<TaskStatus>(core, &state);
        watch::<Pose>(core, &state);
        watch::<Status>(core, &state);
        watch::<WorldMap>(core, &state);
        watch::<MeshChunk>(core, &state);
        watch::<Occupancy>(core, &state);
        watch::<PlannedPath>(core, &state);
        watch::<RemoteState>(core, &state);

        let commands = Arc::new(Commands::default());
        let sender = {
            let (core, commands) = (core.clone(), commands.clone());
            thread::Builder::new()
                .name("terra-comms-commands".into())
                .spawn(move || send_commands(&core, &commands))
                .expect("spawn terra-comms command thread")
        };
        Self { handle, state, commands, sender: Some(sender) }
    }

    /// Rovers whose control tick produced a message in the last
    /// `ROVER_SEEN_WINDOW`, sorted. The link's automatic repeats carry their
    /// real age, so a rover whose tick has stopped drops out, even for a
    /// base that just started.
    pub fn rovers(&self) -> Vec<u64> {
        let mut ids: Vec<u64> = lock(&self.state.seen)
            .iter()
            .filter(|(_, stamp)| stamp.age() < ROVER_SEEN_WINDOW)
            .map(|(id, _)| *id)
            .collect();
        ids.sort_unstable();
        ids
    }

    /// Latest message of type `M` from this rover, with its age in seconds:
    /// how long ago the rover's tick published it (network latency
    /// excluded). Repeats don't make it younger. `TaskStatus` and `Status`
    /// are published on change, so a large age is normal for them; use
    /// `latest::<Pose>` (every tick) or [`Self::rovers`] for liveness.
    /// `Depth` is only kept while a `subscribe::<Depth>` is alive.
    pub fn latest<M: Uplink>(&self, rover_id: u64) -> Option<(M, f64)> {
        let latest = lock(&self.state.latest);
        let entry = latest.get(&(rover_id, M::TOPIC))?;
        let msg = entry.value.downcast_ref::<M>()?.clone();
        Some((msg, entry.stamp.age().as_secs_f64()))
    }

    /// Call `f` for every message of type `M` from any rover, including the
    /// link's repeats. Drop the returned handle to unsubscribe (e.g. stop
    /// pulling `Depth`).
    pub fn subscribe<M: Uplink>(&self, f: impl Fn(u64, M) + Send + 'static) -> Subscription {
        let core = &self.handle.0;
        let depth = (M::TOPIC == Topic::Depth).then(|| {
            self.state.pull_depth(core);
            self.state.clone()
        });
        let topics = core.topics.clone();
        let f = Mutex::new(f);
        let callback: RawCallback = Arc::new(move |sample: Received<'_>| {
            // Counting and bad-payload stats are the watcher's job.
            if let Some((rover_id, msg)) = parse::<M>(&topics, &sample) {
                (lock(&f))(rover_id, msg);
            }
        });
        let id = core.add_sub(core.topics.any_rover(M::TOPIC), callback);
        Subscription { core: core.clone(), id, depth }
    }

    /// Send a command to one rover. Never blocks.
    ///
    /// - `Task`, `TaskCommand` and `Estop` are queued for a background
    ///   thread that sends them in order, reliably (`Block`; `Estop` at
    ///   real-time priority). Without a router they are dropped and counted;
    ///   watch `TaskStatus` to see that a task arrived.
    /// - An engaged `Estop` is re-sent to that rover every `ESTOP_REPEAT`
    ///   until this base releases it or is dropped. A release is sent once
    ///   and never repeated, so if two bases disagree, the rover stays
    ///   stopped.
    /// - `Teleop` and `RemoteCmd` go out at once with `Drop` and are never
    ///   re-sent: hold a stick by calling this at 20 Hz.
    pub fn send<M: Downlink>(&self, rover_id: u64, msg: &M) {
        let core = &self.handle.0;
        let topic = M::TOPIC;
        let payload = msg.encode();
        match topic {
            Topic::Teleop | Topic::RemoteCmd => put(core, rover_id, topic, payload),
            _ => {
                let mut queue = lock(&self.commands.state);
                if let Some(estop) = (msg as &dyn Any).downcast_ref::<Estop>() {
                    if estop.engaged {
                        queue.engaged.insert(rover_id, payload.clone());
                    } else {
                        queue.engaged.remove(&rover_id);
                    }
                }
                queue.queue.push_back((rover_id, topic, payload));
                drop(queue);
                self.commands.wake.notify_one();
            }
        }
    }

    pub fn topics(&self) -> &Topics {
        &self.handle.0.topics
    }

    /// Whether a router is connected. For a status light, not for control.
    pub fn connected(&self) -> bool {
        self.handle.0.connected()
    }

    pub fn stats(&self) -> Stats {
        self.handle.0.counters.snapshot()
    }
}

impl Drop for BaseLink {
    fn drop(&mut self) {
        lock(&self.commands.state).closed = true;
        self.commands.wake.notify_all();
        if let Some(sender) = self.sender.take() {
            let _ = sender.join();
        }
        // `handle` then closes the session.
    }
}

/// Unsubscribes when dropped.
#[must_use = "dropping a Subscription unsubscribes"]
pub struct Subscription {
    core: Arc<Core>,
    id: u64,
    /// Set for `Depth`: the last subscription stops the pull.
    depth: Option<Arc<BaseState>>,
}

impl Drop for Subscription {
    fn drop(&mut self) {
        self.core.remove_sub(self.id);
        if let Some(state) = &self.depth {
            state.release_depth(&self.core);
        }
    }
}

// ------------------------------------------------------------------ sending

fn qos(topic: Topic) -> (CongestionControl, Priority) {
    match topic {
        Topic::Estop => (CongestionControl::Block, Priority::RealTime),
        Topic::Task | Topic::TaskCommand => (CongestionControl::Block, Priority::Data),
        Topic::WebrtcRequest
        | Topic::WebrtcAnswer
        | Topic::WebrtcIceDashboard => (CongestionControl::Block, Priority::DataLow),
        _ => (CongestionControl::Drop, Priority::Data),
    }
}

fn put(core: &Core, rover_id: u64, topic: Topic, payload: Vec<u8>) {
    let Some(session) = core.session().filter(|_| core.connected()) else {
        return core.counters.dropped(topic);
    };
    let (congestion, priority) = qos(topic);
    let result = session
        .put(core.topics.key(rover_id, topic), payload)
        .congestion_control(congestion)
        .priority(priority)
        .express(topic == Topic::Estop)
        .wait();
    match result {
        Ok(()) => core.counters.sent(topic),
        Err(_) => core.counters.dropped(topic),
    }
}

#[derive(Default)]
struct Commands {
    state: Mutex<CommandState>,
    wake: Condvar,
}

#[derive(Default)]
struct CommandState {
    queue: VecDeque<(u64, Topic, Vec<u8>)>,
    /// Engaged e-stop payload per rover, repeated until released.
    engaged: HashMap<u64, Vec<u8>>,
    closed: bool,
}

/// Body of the command thread: send queued commands in order, and repeat
/// engaged e-stops every `ESTOP_REPEAT`. All of them go through this one
/// thread, so a repeat can never overtake a newer release. A `Block` put
/// can stall here while the network is congested; the caller never waits.
fn send_commands(core: &Core, commands: &Commands) {
    let mut next_repeat = Instant::now() + ESTOP_REPEAT;
    loop {
        let batch: Vec<(u64, Topic, Vec<u8>)> = {
            let mut state = lock(&commands.state);
            loop {
                if state.closed {
                    return;
                }
                let now = Instant::now();
                if !state.queue.is_empty() || now >= next_repeat {
                    break;
                }
                state = commands.wake.wait_timeout(state, next_repeat - now).unwrap_or_else(|e| e.into_inner()).0;
            }
            let mut batch: Vec<_> = state.queue.drain(..).collect();
            if Instant::now() >= next_repeat {
                next_repeat = Instant::now() + ESTOP_REPEAT;
                batch.extend(state.engaged.iter().map(|(id, p)| (*id, Topic::Estop, p.clone())));
            }
            batch
        };
        for (rover_id, topic, payload) in batch {
            put(core, rover_id, topic, payload);
        }
    }
}

// ---------------------------------------------------------------- receiving

/// When a message was produced, from its age on arrival.
#[derive(Clone, Copy)]
struct Stamp {
    received: Instant,
    age_on_arrival: Duration,
}

impl Stamp {
    fn age(&self) -> Duration {
        self.age_on_arrival + self.received.elapsed()
    }
}

#[derive(Default)]
struct BaseState {
    latest: Mutex<HashMap<(u64, Topic), Latest>>,
    seen: Mutex<HashMap<u64, Stamp>>,
    depth: Mutex<DepthPull>,
}

struct Latest {
    value: Arc<dyn Any + Send + Sync>,
    stamp: Stamp,
}

#[derive(Default)]
struct DepthPull {
    subscribers: usize,
    watch: Option<u64>,
}

impl BaseState {
    fn record<M: Uplink>(&self, rover_id: u64, msg: M, stamp: Stamp) {
        lock(&self.latest).insert((rover_id, M::TOPIC), Latest { value: Arc::new(msg), stamp });
        let mut seen = lock(&self.seen);
        match seen.get(&rover_id) {
            Some(old) if old.age() <= stamp.age() => {}
            _ => {
                seen.insert(rover_id, stamp);
            }
        }
    }

    fn pull_depth(self: &Arc<Self>, core: &Core) {
        let mut pull = lock(&self.depth);
        pull.subscribers += 1;
        if pull.watch.is_none() {
            pull.watch = Some(watch::<Depth>(core, self));
        }
    }

    fn release_depth(&self, core: &Core) {
        let watch = {
            let mut pull = lock(&self.depth);
            pull.subscribers -= 1;
            if pull.subscribers > 0 {
                return;
            }
            pull.watch.take()
        };
        if let Some(id) = watch {
            core.remove_sub(id);
        }
        lock(&self.latest).retain(|(_, topic), _| *topic != Topic::Depth);
    }
}

fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

fn parse<M: Uplink>(topics: &Topics, sample: &Received<'_>) -> Option<(u64, M)> {
    match topics.parse(sample.key) {
        Some((id, topic)) if topic == M::TOPIC => Some((id, M::decode(sample.payload).ok()?)),
        _ => None,
    }
}

/// The one recording subscription per topic: counts every message, and
/// keeps the latest for `latest`/`rovers`.
fn watch<M: Uplink>(core: &Core, state: &Arc<BaseState>) -> u64 {
    let topics = core.topics.clone();
    let counters = core.counters.clone();
    let state = state.clone();
    let callback: RawCallback = Arc::new(move |sample: Received<'_>| {
        let rover_id = match topics.parse(sample.key) {
            Some((id, topic)) if topic == M::TOPIC => id,
            _ => return counters.bad_payload(M::TOPIC),
        };
        if let Some(msg) = counters.decode::<M>(sample.payload) {
            let stamp = Stamp { received: Instant::now(), age_on_arrival: sample.age };
            state.record(rover_id, msg, stamp);
        }
    });
    core.add_sub(core.topics.any_rover(M::TOPIC), callback)
}
