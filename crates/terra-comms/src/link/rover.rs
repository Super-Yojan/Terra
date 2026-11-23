//! Rover side. Never blocks the control tick: `publish` only fills a
//! one-message slot per topic (newer replaces older) and a background thread
//! sends it while a router is connected. Incoming commands are kept in
//! memory for the tick to read.
use std::collections::hash_map::Entry;
use std::collections::{HashMap, VecDeque};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex, MutexGuard};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use zenoh::Wait;
use zenoh::pubsub::Publisher;
use zenoh::qos::{CongestionControl, Priority};

use super::{Core, Counters, Handle, Received, Stats, age_attachment};
use crate::COMMAND_LEASE;
use crate::codec::{Downlink, Uplink};
use crate::messages::{
    Estop, GateCommand, RemoteCmd, Task, TaskCommand, Teleop, WebrtcAnswer, WebrtcIceDashboard,
    WebrtcRequest,
};
use crate::topics::Topic;

/// Task commands kept between ticks; the oldest is dropped beyond this.
const MAX_QUEUED_COMMANDS: usize = 16;
/// Incremental `ARMeshAnchor` chunks queued for sending; oldest dropped if full.
const MAX_QUEUED_MESH_CHUNKS: usize = 64;
/// WebRTC FPV signaling messages queued inbound or outbound; oldest dropped if full.
const MAX_QUEUED_FPV: usize = 64;
/// How often the sender thread wakes to re-send and check the connection.
const SENDER_TICK: Duration = Duration::from_millis(100);

pub struct RoverLink {
    handle: Handle,
    rover_id: u64,
    inbox: Arc<Inbox>,
    outbox: Arc<Outbox>,
    sender: Option<JoinHandle<()>>,
}

impl RoverLink {
    pub(crate) fn start(handle: Handle, rover_id: u64) -> Self {
        let core = &handle.0;
        let inbox = Arc::new(Inbox {
            counters: core.counters.clone(),
            task: Mutex::default(),
            commands: Mutex::default(),
            teleop: Mutex::default(),
            remote: Mutex::default(),
            estop: AtomicBool::new(false),
            gate: Mutex::default(),
            fpv: Mutex::default(),
        });
        // Newest wins: a task replaced before the tick took it is counted as
        // dropped (the base sees it never became current in `TaskStatus`).
        listen::<Task>(core, rover_id, &inbox, |inbox, task| {
            if lock(&inbox.task).replace(task).is_some() {
                inbox.counters.dropped(Topic::Task);
            }
        });
        listen::<TaskCommand>(core, rover_id, &inbox, |inbox, command| {
            let mut queue = lock(&inbox.commands);
            if queue.len() == MAX_QUEUED_COMMANDS {
                queue.pop_front();
                inbox.counters.dropped(Topic::TaskCommand);
            }
            queue.push_back(command);
        });
        listen::<Teleop>(core, rover_id, &inbox, |inbox, teleop| {
            *lock(&inbox.teleop) = Some((teleop, Instant::now()));
        });
        listen::<RemoteCmd>(core, rover_id, &inbox, |inbox, RemoteCmd(teleop)| {
            *lock(&inbox.remote) = Some((teleop, Instant::now()));
        });
        // Latched: engaged sets it, only an explicit `engaged: false` clears it.
        listen::<Estop>(core, rover_id, &inbox, |inbox, estop| {
            inbox.estop.store(estop.engaged, Ordering::Release);
        });
        // Dump gate command: newest wins.
        listen::<GateCommand>(core, rover_id, &inbox, |inbox, cmd| {
            *lock(&inbox.gate) = Some(cmd);
        });
        // WebRTC FPV signaling: separate FIFO queue so SDP/ICE never competes with control.
        listen::<WebrtcRequest>(core, rover_id, &inbox, |inbox, WebrtcRequest(payload)| {
            push_fpv(inbox, Topic::WebrtcRequest, payload);
        });
        listen::<WebrtcAnswer>(core, rover_id, &inbox, |inbox, WebrtcAnswer(payload)| {
            push_fpv(inbox, Topic::WebrtcAnswer, payload);
        });
        listen::<WebrtcIceDashboard>(core, rover_id, &inbox, |inbox, WebrtcIceDashboard(payload)| {
            push_fpv(inbox, Topic::WebrtcIceDashboard, payload);
        });

        let outbox = Arc::new(Outbox::default());
        let sender = {
            let (core, outbox) = (core.clone(), outbox.clone());
            thread::Builder::new()
                .name("terra-comms-send".into())
                .spawn(move || send_loop(&core, &outbox, rover_id))
                .expect("spawn terra-comms sender thread")
        };
        Self { handle, rover_id, inbox, outbox, sender: Some(sender) }
    }

    pub fn rover_id(&self) -> u64 {
        self.rover_id
    }

    /// Queue for sending; returns at once. Newer replaces older per topic,
    /// and the newest waits while no router is connected.
    ///
    /// `Capabilities`, `TaskStatus` and `Status` are also re-sent on their
    /// own (see [`Topic::repeat_period`]) and right after reconnecting.
    /// Every send carries the message's age (time since this call), so the
    /// base never mistakes a repeat for a sign that the tick is alive.
    ///
    /// Publish `Pose` every tick, tracked or not: the base's `rovers()`
    /// relies on it.
    pub fn publish<M: Uplink>(&self, msg: &M) {
        let payload = Outgoing { payload: msg.encode(), published: Instant::now() };
        let topic = M::TOPIC;
        let mut out = self.outbox.lock();
        if topic.repeat_period().is_some() {
            out.retained[topic.index()] = Some(Retained { msg: payload.clone(), last_sent: None });
        }
        if topic == Topic::Mesh {
            if out.mesh_queue.len() == MAX_QUEUED_MESH_CHUNKS {
                out.mesh_queue.pop_front();
                self.handle.0.counters.dropped(topic);
            }
            out.mesh_queue.push_back(payload);
        } else if topic.is_webrtc() {
            if out.fpv_queue.len() == MAX_QUEUED_FPV {
                out.fpv_queue.pop_front();
                self.handle.0.counters.dropped(topic);
            }
            out.fpv_queue.push_back((topic, payload));
        } else if out.pending[topic.index()].replace(payload).is_some() {
            self.handle.0.counters.dropped(topic);
        }
        drop(out);
        self.outbox.wake.notify_one();
    }

    /// The newest task that hasn't been handed out yet. Each returned once.
    pub fn take_task(&self) -> Option<Task> {
        lock(&self.inbox.task).take()
    }

    /// The oldest task command that hasn't been handed out yet, in arrival
    /// order. Each returned once.
    pub fn take_task_command(&self) -> Option<TaskCommand> {
        lock(&self.inbox.commands).pop_front()
    }

    /// The latest teleop twist; `None` after `COMMAND_LEASE` of silence.
    pub fn teleop(&self) -> Option<Teleop> {
        fresh(&self.inbox.teleop)
    }

    /// The latest `remote/cmd` twist; `None` after `COMMAND_LEASE` of silence.
    pub fn remote_cmd(&self) -> Option<Teleop> {
        fresh(&self.inbox.remote)
    }

    /// Remote e-stop, latched: true from an engaged `Estop` until a base
    /// sends `Estop { engaged: false }`. Starts false. A base re-sends an
    /// engaged e-stop every `ESTOP_REPEAT`, so a restarted rover picks it up
    /// within about a second of connecting (a restored task comes back as
    /// `NeedsAttention(RoverRestarted)`, so nothing drives in that gap), and
    /// with several bases, any one holding it engaged wins.
    pub fn estop(&self) -> bool {
        self.inbox.estop.load(Ordering::Acquire)
    }

    /// The newest dump-gate command received since the last call, if any.
    pub fn take_gate(&self) -> Option<GateCommand> {
        lock(&self.inbox.gate).take()
    }

    /// Drain all queued inbound WebRTC FPV signaling messages in arrival order.
    pub fn take_fpv(&self) -> Vec<(Topic, String)> {
        lock(&self.inbox.fpv).drain(..).collect()
    }

    /// Whether a router is connected. For a status light, not for control.
    pub fn connected(&self) -> bool {
        self.handle.0.connected()
    }

    pub fn stats(&self) -> Stats {
        self.handle.0.counters.snapshot()
    }
}

impl Drop for RoverLink {
    fn drop(&mut self) {
        self.outbox.lock().closed = true;
        self.outbox.wake.notify_all();
        if let Some(sender) = self.sender.take() {
            let _ = sender.join();
        }
        // `handle` then closes the session.
    }
}

fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

fn fresh(slot: &Mutex<Option<(Teleop, Instant)>>) -> Option<Teleop> {
    lock(slot).filter(|(_, at)| at.elapsed() < COMMAND_LEASE).map(|(t, _)| t)
}

struct Inbox {
    counters: Arc<Counters>,
    task: Mutex<Option<Task>>,
    commands: Mutex<VecDeque<TaskCommand>>,
    teleop: Mutex<Option<(Teleop, Instant)>>,
    remote: Mutex<Option<(Teleop, Instant)>>,
    estop: AtomicBool,
    gate: Mutex<Option<GateCommand>>,
    fpv: Mutex<VecDeque<(Topic, String)>>,
}

fn push_fpv(inbox: &Inbox, topic: Topic, payload: String) {
    let mut queue = lock(&inbox.fpv);
    if queue.len() == MAX_QUEUED_FPV {
        queue.pop_front();
        inbox.counters.dropped(topic);
    }
    queue.push_back((topic, payload));
}

/// Subscribe to one base → rover topic for this rover.
fn listen<M: Downlink>(core: &Core, rover_id: u64, inbox: &Arc<Inbox>, on: fn(&Inbox, M)) {
    let counters = core.counters.clone();
    let inbox = inbox.clone();
    core.add_sub(
        core.topics.key(rover_id, M::TOPIC),
        Arc::new(move |sample: Received<'_>| {
            if let Some(msg) = counters.decode::<M>(sample.payload) {
                on(&inbox, msg);
            }
        }),
    );
}

#[derive(Default)]
struct Outbox {
    state: Mutex<OutState>,
    wake: Condvar,
}

#[derive(Clone)]
struct Outgoing {
    payload: Vec<u8>,
    /// When the tick published it.
    published: Instant,
}

#[derive(Default)]
struct OutState {
    /// Waiting to go out, newest only.
    pending: [Option<Outgoing>; Topic::COUNT],
    /// Incremental mesh chunks (FIFO so batched anchor updates don't clobber each other).
    mesh_queue: VecDeque<Outgoing>,
    /// Outbound WebRTC FPV signaling messages (FIFO so trickle ICE candidates never clobber).
    fpv_queue: VecDeque<(Topic, Outgoing)>,
    /// Last message of each repeating topic.
    retained: [Option<Retained>; Topic::COUNT],
    closed: bool,
}

struct Retained {
    msg: Outgoing,
    last_sent: Option<Instant>,
}

impl Outbox {
    fn lock(&self) -> MutexGuard<'_, OutState> {
        lock(&self.state)
    }
}

impl OutState {
    fn has_pending(&self) -> bool {
        self.pending.iter().any(Option::is_some)
            || !self.mesh_queue.is_empty()
            || !self.fpv_queue.is_empty()
    }

    /// Everything to send now: pending messages, plus repeats that are due
    /// (all of them when `resend_all`, after a reconnect).
    fn take_due(&mut self, now: Instant, resend_all: bool) -> Vec<(Topic, Outgoing)> {
        let mut due = Vec::new();
        while let Some(item) = self.fpv_queue.pop_front() {
            due.push(item);
        }
        for topic in Topic::ALL {
            if topic == Topic::Mesh {
                while let Some(msg) = self.mesh_queue.pop_front() {
                    due.push((Topic::Mesh, msg));
                }
                continue;
            }
            let i = topic.index();
            let retained = &mut self.retained[i];
            if let Some(msg) = self.pending[i].take() {
                if let Some(r) = retained {
                    r.last_sent = Some(now);
                }
                due.push((topic, msg));
            } else if let (Some(period), Some(r)) = (topic.repeat_period(), retained)
                && (resend_all || r.last_sent.is_none_or(|t| now - t >= period))
            {
                r.last_sent = Some(now);
                due.push((topic, r.msg.clone()));
            }
        }
        due
    }
}

/// Body of the sender thread. Returns when the `RoverLink` is dropped.
fn send_loop(core: &Core, outbox: &Outbox, rover_id: u64) {
    let mut publishers: HashMap<Topic, Publisher<'static>> = HashMap::new();
    let mut was_connected = false;
    loop {
        // Nothing goes out without a router: pending messages wait (newest
        // only), and everything retained is re-sent on (re)connect.
        let session = core.session().filter(|_| core.connected());
        let reconnected = session.is_some() && !was_connected;
        was_connected = session.is_some();
        let now = Instant::now();

        let due = {
            let mut out = outbox.lock();
            if out.closed {
                return;
            }
            match &session {
                Some(_) => out.take_due(now, reconnected),
                None => Vec::new(),
            }
        };

        if let Some(session) = &session {
            for (topic, msg) in due {
                let publisher = match publishers.entry(topic) {
                    Entry::Occupied(e) => e.into_mut(),
                    Entry::Vacant(e) => {
                        let (congestion, priority) = match topic {
                            Topic::Depth => (CongestionControl::Drop, Priority::DataLow),
                            Topic::WorldMap | Topic::Mesh => {
                                (CongestionControl::Block, Priority::DataLow)
                            }
                            t if t.is_webrtc() => (CongestionControl::Block, Priority::DataLow),
                            _ => (CongestionControl::Drop, Priority::Data),
                        };
                        match session
                            .declare_publisher(core.topics.key(rover_id, topic))
                            .congestion_control(congestion)
                            .priority(priority)
                            .wait()
                        {
                            Ok(p) => e.insert(p),
                            Err(err) => {
                                eprintln!("terra-comms: cannot declare publisher for {topic}: {err}");
                                core.counters.dropped(topic);
                                continue;
                            }
                        }
                    }
                };
                let age = age_attachment(now.saturating_duration_since(msg.published));
                match publisher.put(msg.payload).attachment(age).wait() {
                    Ok(()) => core.counters.sent(topic),
                    Err(_) => core.counters.dropped(topic),
                }
            }
        }

        let out = outbox.lock();
        if !out.closed && (session.is_none() || !out.has_pending()) {
            let _ = outbox.wake.wait_timeout(out, SENDER_TICK);
        }
    }
}
