# terra-comms: API design

The one crate that both the rover (`terra-av`) and the base station (the new
ARGOS) use to talk over Zenoh. It owns the wire contract, so the two sides
can't drift apart. Implemented in `src/`; see "Implementation notes" at the
end for where the code refines this design.

Diagram: `docs/diagrams/pipeline.png`.

Runnable example, rover and base station in one process with a local
router: `cargo run -p terra-comms --example demo` (`examples/demo.rs`).

## Goals

- **Research friendly.** Someone can add a controller to the rover, and an
  operator can then pick it, set its parameters and choose where to go,
  all from the base station, with no UI or comms changes.
- **One definition** of the config, key names and message encodings, used by
  both sides.
- **Never block the rover's control tick.** Publishing only queues. If the
  network is slow or down, the newest message replaces the older one.
- **Safety stays outside the controllers.** Nothing sent over the link can
  bypass the rover's clamp, e-stop or timeouts.
- **Small.** Anything that isn't shared between rover and base station stays
  out (planning, UI, logging, fleet bookkeeping).

## 1. Config

Loaded from the `[comms]` table in TOML: `terra-av.toml` on the rover, the
base station's own file on the other side.

```toml
[comms]
endpoint = "tls/argos.example.com:7448"   # tcp/host:port or tls/host:port
prefix   = "terra"                        # first part of every key
rover_id = 0                              # rover only; the base station leaves it out
user     = "terra"                        # optional router login
# password: never in the file. Pass it in from the Keychain / env.
```

```rust
pub struct CommsConfig {
    pub endpoint: String,
    pub prefix: String,
    pub rover_id: Option<u64>,
    pub user: Option<String>,
    pub password: Option<String>,
}

impl CommsConfig {
    /// Parse the `[comms]` table. Rejects unknown keys, a bad endpoint,
    /// and a prefix that isn't a plain key (`*`, `?`, `#`, `$`, empty parts).
    pub fn from_toml(table: &str) -> Result<Self, ConfigError>;
    /// Use this password instead of whatever the file had.
    pub fn with_password(self, password: Option<String>) -> Self;
}
```

`terra-av`'s `config.rs` embeds `CommsConfig` as `Option<CommsConfig>`. If
`[comms]` is missing, there's no link.

## 2. Connecting

```rust
/// Open a Zenoh client session in the background and return at once.
/// Retries forever if the router is down. Never fails on network errors,
/// only on a bad config.
pub fn connect(config: &CommsConfig) -> Result<Link, ConfigError>;

impl Link {
    pub fn rover(self) -> Result<RoverLink, ConfigError>;  // needs rover_id
    pub fn base(self) -> BaseLink;
    pub fn connected(&self) -> bool;   // for a status light, not for control
}
```

Fixed Zenoh settings: client mode, multicast scouting off, user/password
auth when `user` is set. Congestion control is `Drop` for streams
(rover → base topics, `teleop`, `remote/cmd`) and `Block` for `task`,
`task/cmd` and `estop` (`estop` at real-time priority), so congestion
doesn't drop those. A `Block` put can still fail (link closed, no router);
the failure is counted in `stats()`, and the base sends from a background
thread so the caller never waits.

## 3. Keys

Every key is `{prefix}/{rover_id}/{topic}`. The crate builds and parses
them; callers never format keys by hand.

| Topic          | Direction     | Message        | Rate / lifetime                       |
|----------------|---------------|----------------|---------------------------------------|
| `capabilities` | rover → base  | `Capabilities` | on connect and every 5 s              |
| `task/status`  | rover → base  | `TaskStatus`   | on change and 1 Hz                    |
| `pose`         | rover → base  | `Pose`         | 20 Hz                                 |
| `status`       | rover → base  | `Status`       | 1 Hz and on change                    |
| `depth`        | rover → base  | `Depth`        | up to 20 Hz, dropped when busy        |
| `occupancy`    | rover → base  | `Occupancy`    | on change; repeated every 2 s         |
| `remote/state` | rover → base  | `RemoteState`  | 20 Hz, only while `remote` is running |
| `task`         | base → rover  | `Task`         | on change; persists until replaced    |
| `task/cmd`     | base → rover  | `TaskCommand`  | on demand (resume, pause, clear)      |
| `teleop`       | base → rover  | `Teleop`       | 20 Hz; expires after `COMMAND_LEASE`  |
| `remote/cmd`   | base → rover  | `Teleop`       | 20 Hz; expires after `COMMAND_LEASE`  |
| `estop`        | base → rover  | `Estop`        | on change; engaged repeated at 1 Hz by the link; latched |

`COMMAND_LEASE = 1 s`: one constant in this crate, used by both sides. It
has to cover the trip through the cloud-hosted router (several hops), so
250 ms was too short. It only governs commands over the link (`teleop`,
`remote/cmd`). The phone → rover-pi hops keep their own short timeouts
(0.3 s velocity loop, 0.25 s `DriveRelay`).

```rust
pub struct Topics { prefix: String }
impl Topics {
    pub fn key(&self, rover_id: u64, topic: Topic) -> String;
    pub fn parse(&self, key: &str) -> Option<(u64, Topic)>;
}
```

## 4. Controllers, parameters and tasks

A **controller** decides the body twist `(v, w)` each tick. In `terra-av` it
is a `LocalPlanner` (`Teleop`, `DrivePose`, `Dwa`, …). Global planners
(`StraightLine`, `HybridAStar`) are chosen the same way but are optional.
The estimator, mapper, velocity loop and safety chain are **not**
selectable at run time.

### 4.1 Parameters (shared types)

The rover describes each parameter so the UI can draw a form without
knowing anything about the controller.

```rust
pub struct ParamSpec {
    pub name: String,              // "horizon"
    pub kind: ParamKind,           // Float | Int | Bool | Choice(Vec<String>)
    pub default: ParamValue,
    pub min: Option<f64>,
    pub max: Option<f64>,
    pub unit: Option<String>,      // "s", "m/s"
    pub help: String,
}

pub enum ParamValue { Float(f64), Int(i64), Bool(bool), Choice(String) }
pub type Params = BTreeMap<String, ParamValue>;   // missing → default

/// Check `params` against `specs`: unknown name, wrong kind, out of range.
pub fn validate(specs: &[ParamSpec], params: &Params) -> Result<Params, ParamError>;
```

`validate` lives here so the UI and the rover reject exactly the same
inputs. The rover always re-checks; the UI checks only to give early feedback.

### 4.2 Capabilities (rover → base)

```rust
pub struct Capabilities {
    pub t: f64,
    pub controllers: Vec<StageOption>,
    pub planners: Vec<StageOption>,
    pub default_controller: String,
    pub software: String,          // git describe of the rover build
}

pub struct StageOption {
    pub id: String,                // "dwa"
    pub version: String,           // bumped when behaviour changes
    pub description: String,
    pub params: Vec<ParamSpec>,
    pub goals: Vec<GoalKind>,      // goal kinds it accepts (see §4.4)
}

pub enum GoalKind { Local, Gps }
```

### 4.3 Task (base → rover)

A task says **where** to go and **how**. It is persistent (§4.5).

```rust
pub struct Task {
    pub id: u64,                   // chosen by the base, unique per rover;
                                   // same id = update of the current task (§4.7)
    pub goal: Goal,
    pub controller: String,        // a Capabilities.controllers id
    pub params: Params,
    pub planner: Option<String>,   // None → the rover's configured planner
    pub planner_params: Params,
}

/// Acts on the current task. Ignored (and rejected) if `task_id` isn't it.
pub struct TaskCommand { pub task_id: u64, pub action: TaskAction }
pub enum TaskAction { Resume, Pause, Clear }

pub enum Goal {
    Stop,                                          // hold still (still a task)
    Pose {                                         // GoalKind::Local
        x: f64, y: f64,                            // world frame, m
        yaw: Option<f64>,                          // rad, CCW from +X
        tolerance: f64,                            // m; Reached inside this
    },
    Gps {                                          // GoalKind::Gps
        latitude: f64, longitude: f64,             // WGS84, degrees
        heading: Option<f64>,                      // rad, clockwise from true north
        tolerance: f64,                            // m; at least the fix accuracy
    },
}
```

The world frame is the rover's own (Z up, X forward at start, yaw CCW; see
`docs/diagrams/coordinate-frames.png`). GPS goals use the compass
convention (`heading`, clockwise from north) on purpose, and a different
name, so the two are never mixed up.

### 4.4 GPS goals

Whether a GPS goal can be driven depends on two things: the **estimator**
(does it know where the world frame sits on Earth?) and the **controller**
(does it want local or raw GPS goals?).

**GPS anchor (estimator).** Converting latitude/longitude to world `x, y`
needs the GPS position of the world origin and the heading of world +X.
That is the estimator's job: `Localization` (VIO + GPS fusion) provides it,
plain `Vio` does not. The rover reports it in `Status.gps_anchor` (§5); it
is `None` until there is one.

**Goal kinds (controller).** Each controller lists the goal kinds it
accepts in `StageOption.goals`:

| Controller lists | A `Pose` goal              | A `Gps` goal                                   |
|------------------|----------------------------|------------------------------------------------|
| `[Local]`        | passed through             | converted to `Pose` with the anchor, then passed |
| `[Gps]`          | rejected                   | passed through untouched (native GPS follower) |
| `[Local, Gps]`   | passed through             | passed through untouched                       |

Most controllers (`drive_pose`, `dwa`, `remote`) list only `[Local]` and
never see GPS: the conversion happens once, in `set_task`, before the
planner runs. A controller that steers by compass bearing to a coordinate
lists `[Gps]`.

Conversion rules:
- No anchor when the task arrives → rejected (`"no GPS anchor"`). Anchor
  lost later → `NeedsAttention(NoGpsAnchor)`; the goal stays.
- `tolerance` smaller than the anchor's accuracy → raised to the accuracy,
  and the final value is reported in `TaskStatus`.
- When the anchor is refined mid-task, the goal is converted again and the
  path re-planned. The task is not restarted.

### 4.5 Tasks are persistent

A task, and its goal, stays until a human replaces or clears it. Nothing on
the rover clears it by itself: not a timeout, a stuck rover, an e-stop, a
lost pose, a lost link or a rover restart. When something goes wrong, the
rover **holds still and asks for attention**; the goal is still there.

| What happens                     | Who can do it | Effect on the task                                  |
|----------------------------------|---------------|-----------------------------------------------------|
| New `Task` (new `id`)            | human         | replaces the old task; the old one ends             |
| Updated `Task` (same `id`)       | human         | swaps controller / params / planner; goal kept      |
| `TaskCommand::Resume`            | human         | `NeedsAttention` or `Paused` → `Running`            |
| `TaskCommand::Pause`             | human         | → `Paused`, hold still                              |
| `TaskCommand::Clear`             | human         | task removed → `Idle`                               |
| timeout, stuck, error, e-stop, … | rover         | → `NeedsAttention(reason)`, hold still, goal kept   |
| goal reached                     | rover         | → `Reached`, hold still, goal kept                  |
| pushed out of tolerance          | rover         | `Reached` → `Running` (drives back), if the controller allows (§4.8) |

`Goal::Stop` in a new task also means "hold still", but it's a task like any
other: it stays until replaced.

### 4.6 Task status (rover → base)

```rust
pub struct TaskStatus {
    pub t: f64,
    pub task_id: Option<u64>,      // None when Idle
    pub state: TaskState,
    pub controller: String,        // what is actually driving
    pub controller_version: String,
    pub params: Params,            // after defaults were filled in
    pub goal: Goal,                // as the controller received it
                                   // (after GPS conversion and tolerance)
    pub timeout: Option<f64>,      // s, set by the controller (§4.8); None = no limit
    pub last_rejected: Option<Rejection>,  // latest refused request, if any
}

pub enum TaskState {
    Idle,                          // no task
    Running { distance_to_goal: f64, elapsed: f64 },
    Reached,                       // within tolerance; holding still, watching for drift
    Paused,                        // by the operator; holding still
    NeedsAttention { reason: Attention, since: f64 },  // holding still
}

pub enum Attention {
    Timeout,                       // the controller's timeout passed (§4.8)
    Stuck,                         // no progress, by the controller's rule (§4.8)
    ControllerError(String),
    PoseLost,                      // estimator untracked for too long
    NoGpsAnchor,                   // a Gps goal lost its anchor
    EStop,                         // e-stop engaged (phone or remote)
    RemoteSilent,                  // `remote` controller: no remote/cmd for COMMAND_LEASE
    Displaced,                     // pushed out of tolerance, controller won't return (§4.8)
    RoverRestarted,                // task restored from disk at startup
}

/// A request the rover refused. The current task is untouched.
pub struct Rejection { pub task_id: u64, pub reason: String, pub t: f64 }
```

Status carries the exact controller, version and parameters, so every run
can be reproduced from the log. `NeedsAttention` is what the base station
turns into an alert for a human.

### 4.7 Rover rules

**A new task (new `id`):**
1. Check that the controller and planner ids exist and `validate` the params.
2. Check the goal kind against the controller's `goals`, and convert a GPS
   goal if needed (§4.4).
3. If either fails, record a `Rejection`. The current task keeps going.
4. Otherwise build the controller, ask it for its timeout, and swap it in
   **between ticks**. The old task ends; the new one is `Running`.

**An update (same `id`)**, sent to change how the rover drives toward the
same goal, including while it's moving:
1. The goal must be unchanged; a different goal needs a new task id.
2. Checked like a new task. If it fails, it's rejected and the current
   controller keeps driving.
3. The new controller is swapped in between ticks, with no stop. The
   acceleration limits in the fixed chain smooth the change.
4. The new controller computes a fresh timeout from the current state.
5. The state doesn't change: a `Running` task keeps running, a
   `NeedsAttention` or `Paused` task stays put until `Resume`.

**`TaskCommand { task_id, action }`** must name the current task, or it's
rejected, so a stale click can't act on a newer task. `Resume` recomputes
the timeout from the current state. Nothing resumes by itself, including
after an e-stop is released.

**Resume needs every e-stop released.** The rover refuses `Resume` (a
`Rejection` with reason `"e-stop engaged"`) while any e-stop is engaged:
the phone's own, or a remote one from any base station. Releasing doesn't
resume; a human sends `Resume` after the release.

From one base station, `send(Estop { engaged: false })` followed straight
away by `send(TaskCommand { action: Resume })` is safe. The base sends its
commands in order from one thread, and the e-stop travels at a higher
priority, so a later `Resume` can't overtake the release. (The other way
around, an e-stop sent after a command may overtake it, which is the safe
direction.) What one base can't see is an e-stop it didn't send, so the UI
should check `Status.estopped` before offering Resume (§7).

**Persistence.** The rover writes the current task to disk whenever it
changes. At startup it restores it as `NeedsAttention(RoverRestarted)`, so
the goal survives a crash or reboot, but the rover never starts driving on
its own.

**Link loss** doesn't affect an autonomous task. The rover keeps driving
until the task is `Reached` (or needs attention for its own reasons, such as
a timeout), with its own fail-safes still in force. Status queues up and is
seen again when the link returns. Only link-driven control stops: teleop's
twist goes to zero after `COMMAND_LEASE`, and the `remote` controller goes to
`NeedsAttention(RemoteSilent)`, because it depends on the base by design.

### 4.8 Timeout, stuck and holding the goal

These rules belong to the controller, because only its design knows how long
a goal should take, what progress looks like and how to hold a goal. None of
them ever clears the task.

- **Timeout.** Computed when the task starts, on an update and on `Resume`,
  from the goal, the rover's state and the path. A typical rule is
  `distance / expected_speed × margin + slack`. `None` means no limit (e.g.
  `teleop`, `remote`). A refined GPS anchor (§4.4) doesn't change it.
- **Stuck.** The controller says how long it may go without progress
  toward the goal (`stuck_after`, seconds), and what counts as progress.
  The default is distance to goal shrinking by at least 10 cm. `None`
  turns it off.
- **Holding the goal.** While `Reached`, the rover watches its distance to
  the goal. If it's pushed out, the controller's `returns_to_goal` decides:
  - `true` (the default): back to `Running`, it drives back, then `Reached`
    again. A fresh timeout is computed. To avoid hunting at the edge, it
    re-drives only past `tolerance × 1.5` (the controller may change the
    factor).
  - `false`: `NeedsAttention(Displaced)`, holding still, for a controller
    that shouldn't move without a human (e.g. a one-shot experiment).
- A controller that wants the operator to tune any of these exposes it as a
  parameter (`timeout`, `margin`, `stuck_after`, `returns_to_goal`). The rule
  stays the controller's.
- The rover enforces both, not the base, so a base that loses the link
  can't leave a task running forever.

### 4.9 Remote controller

`remote` is a built-in controller id. It lets someone write a controller in
any language on a laptop, without rebuilding the rover.

- While it runs, the rover publishes `RemoteState` every tick:
  `{ t, pose, twist, path, obstacles }` in the world frame.
- The base answers on `remote/cmd` with a `Teleop` twist.
- With no `remote/cmd` for `COMMAND_LEASE`, the twist is zero and the task
  goes to `NeedsAttention(RemoteSilent)`.

Network latency makes this a prototyping tool. Final results should come
from a compiled-in controller.

## 5. Other messages

Plain structs. Every message carries `t`, the sender's clock in seconds.

```rust
pub struct Pose   { pub t: f64, pub x: f64, pub y: f64, pub yaw: f64, pub tracked: bool }
pub struct Status {
    pub t: f64, pub estopped: bool, pub link_ok: bool, pub detail: String,
    pub gps_anchor: Option<GpsAnchor>,   // None until the estimator has one
}
/// Where the world frame sits on Earth (§4.4). The base can also use it to
/// draw `Pose` on a real map; the rover still does the goal conversion.
pub struct GpsAnchor {
    pub latitude: f64, pub longitude: f64,   // world origin, WGS84 degrees
    pub heading: f64,                        // world +X, rad clockwise from north
    pub accuracy: f64,                       // m, 1σ
}
pub struct Depth  { pub t: f64, pub sequence: u64, pub points: Vec<[f32; 3]> } // camera frame, m
/// The rover's obstacle map: every occupied cell of its occupancy grid,
/// world frame. The base draws it; the planners on the rover use the same cells.
pub struct Occupancy { pub t: f64, pub resolution: f64, pub cells: Vec<[f32; 2]> } // cell centres, m
pub struct Teleop { pub t: f64, pub v: f64, pub w: f64 }                     // body twist
pub struct Estop  { pub t: f64, pub engaged: bool }
```

Encoding is the crate's job, behind one trait:

```rust
pub trait Message: Sized {
    const TOPIC: Topic;
    fn encode(&self) -> Vec<u8>;
    fn decode(bytes: &[u8]) -> Result<Self, DecodeError>;
}
```

- Small messages are JSON, so you can read them with `z_sub`.
- `Depth` is a one-line JSON header (`t`, `sequence`, `count`) followed by
  `count × 3` little-endian `f32`s. That's roughly 4× smaller than JSON points.
- `Occupancy` is encoded the same way: a header (`t`, `resolution`,
  `count`) followed by `count × 2` little-endian `f32`s.
- Each payload starts with a `version` field. A decoder rejects a version it
  doesn't know instead of guessing.
- Rover → base samples carry a Zenoh attachment: the message's age when it
  was sent (time since `publish`), as a little-endian `u64` of
  microseconds. The base uses it to tell how old a message (or a repeat) is
  without comparing clocks.
- Non-finite numbers (NaN, ±inf) encode as JSON `null`. The receiver
  rejects such a message as a bad payload, so don't publish them.

## 6. Rover side

```rust
impl RoverLink {
    /// Queue for sending; returns at once. Newer replaces older per topic.
    pub fn publish<M: Message>(&self, msg: &M);
    /// A task or task command that hasn't been handed out yet. Each returned once.
    pub fn take_task(&self) -> Option<Task>;
    pub fn take_task_command(&self) -> Option<TaskCommand>;
    pub fn teleop(&self) -> Option<Teleop>;      // None after COMMAND_LEASE of silence
    pub fn remote_cmd(&self) -> Option<Teleop>;  // None after COMMAND_LEASE of silence
    pub fn estop(&self) -> bool;                 // true until the base clears it
}
```

In `terra-av`, each tick:

1. `take_task()` and `take_task_command()`. Apply them with the rules in §4.7
   (`Pipeline::set_task`, `Pipeline::task_command`).
2. Read `estop()` into `SensorFrame.estop`, and `teleop()` when the active
   controller is `teleop`.
3. Run the pipeline.
4. Check the controller's timeout and stuck rules (§4.8).
5. `publish` `Pose` (every tick, tracked or not: the base's `rovers()`
   relies on it), `Depth`, and `TaskStatus` when it changed.

`Capabilities` is built once at startup from the controller registry
(`terra-av`'s `controllers()`; see the pipeline diagram) and published
on connect and every 5 s.

**Safety rule:** losing the link must never make the rover move.
- A stale `teleop` or `remote/cmd` reads as `None`, so the twist is zero.
- A remote e-stop is latched. Only an explicit `Estop { engaged: false }`
  clears it. The latch starts unset at startup. A base re-sends an
  *engaged* e-stop every second, so a restarted rover picks it up within
  about a second of connecting. Nothing drives in that gap, because a
  restored task comes back as `NeedsAttention(RoverRestarted)`.
- A release is sent once and never repeated. With several bases, any base
  holding e-stop engaged re-latches the rover within a second, so a
  disagreement always ends with the rover stopped. Only the base that
  engaged it can release it for good.
- Whatever the controller outputs still goes through
  `clamp → acceleration limits → SafetyGate → velocity-loop timeout → rover-pi lease`.

## 7. Base-station side

```rust
impl BaseLink {
    /// Rovers seen in the last few seconds, from any rover → base topic.
    pub fn rovers(&self) -> Vec<u64>;
    /// Latest message of type M from this rover, with its age in seconds.
    pub fn latest<M: Message>(&self, rover_id: u64) -> Option<(M, f64)>;
    /// Call `f` for every message of type M from any rover. Drop the
    /// returned handle to unsubscribe (e.g. stop pulling `Depth`).
    pub fn subscribe<M: Message>(&self, f: impl Fn(u64, M) + Send + 'static) -> Subscription;
    /// Send a command (Task, TaskCommand, Teleop, Estop, remote Teleop) to one rover.
    pub fn send<M: Message>(&self, rover_id: u64, msg: &M);
}
```

A typical UI flow:

1. `latest::<Capabilities>(id)` → controller dropdown and parameter form.
2. The operator picks a controller. Goal types it can't take are greyed out
   (`StageOption.goals`). A `Gps` goal is also greyed out for a `[Local]`
   controller while `latest::<Status>(id)` has no `gps_anchor`.
3. The operator clicks a goal on the map. The tolerance defaults to the GPS
   accuracy for `Gps` goals.
4. `validate(...)` the form, then `send(id, &Task { … })`.
5. Show `latest::<TaskStatus>(id)`. Alert a human on `NeedsAttention`, and
   offer: Resume, change controller (an update, same id), new goal, or Clear.
6. Grey out Resume while `latest::<Status>(id)` shows `estopped: true`.
   That covers the phone's e-stop and other base stations' e-stops, which
   this base didn't send. If the operator releases this base's e-stop and
   resumes, both can be sent back to back (§4.7); no need to wait.
7. Show `last_rejected` next to the form if a request was refused (e.g. a
   Resume that met an e-stop engaged elsewhere).

`send` never blocks.
- `Teleop` and `RemoteCmd` go out at once. Holding a stick means calling
  `send(Teleop)` at 20 Hz. The crate doesn't resend it for you, so a
  frozen UI stops the rover.
- `Task`, `TaskCommand` and `Estop` are queued for one background thread
  that sends them in order with `Block`. Without a router they are
  dropped and counted, so watch `TaskStatus` to confirm a task arrived.
- An engaged `Estop` is re-sent to that rover every `ESTOP_REPEAT = 1 s`
  until this base releases it or is dropped (§6).

## 8. Errors

- `ConfigError`: from `from_toml`, `connect` and `rover()`. Shown to the
  user, never retried.
- `ParamError`: from `validate`. Shown next to the form field.
- `DecodeError`: a bad payload is dropped and counted, and never reaches the
  caller.
- Network errors are never returned. `Link::connected()` and `stats()` (sent,
  dropped and bad-payload counts per topic) are the only view into them.

## 9. Features

- `zenoh` (default): everything above.
- No features: only `CommsConfig`, `Topics`, the messages, `ParamSpec` and
  `validate`. This is useful for tests, replay tools and a smaller iOS build
  with no link.

## Adding a controller (terra-av side, for reference)

Not part of this crate, but it's what the design is for:

1. Write `planning/local/my_controller.rs` implementing `LocalPlanner`,
   including its timeout rule:
   `fn timeout(&self, goal: &Goal, state: &State, path: &Path) -> Option<f64>`
   `fn stuck_after(&self) -> Option<f64>` and
   `fn returns_to_goal(&self) -> bool` (§4.8; defaults exist for all three).
2. Add one `ControllerSpec { id, version, description, params, build }` to
   `controllers()`.
3. Rebuild the rover. It appears in `Capabilities`, so in the UI dropdown,
   with its parameter form.

Controllers are compiled in, not loaded as plugins: iOS forbids loading code
at run time. `remote` covers the "no rebuild" case.

## Not in this crate

Fleet authority beyond leases, missions with several goals, maps and
occupancy grids, point-capture sessions, video, logging/observability, and
running a router. Add one here only once both sides need it.

## Decisions

- `COMMAND_LEASE` is 1 s (§3).
- Link loss never interrupts an autonomous task; it runs to `Reached` (§4.7).
- Controllers can be swapped while moving; the goal is kept (§4.7).
- Tasks and goals are persistent; problems mean `NeedsAttention`, not
  clearing (§4.5).
- Timeout, stuck and drive-back-when-pushed are each the controller's
  rule (§4.8).

## Open questions

None right now.

## Implementation notes

- Direction is checked at compile time: messages implement `Uplink`
  (rover → base) or `Downlink` (base → rover). `RoverLink::publish` and
  `BaseLink::latest`/`subscribe` take `M: Uplink`. `BaseLink::send` takes
  `M: Downlink`.
- The `remote/cmd` twist is sent as `RemoteCmd(Teleop)`, so each type maps
  to one topic. `RoverLink::remote_cmd()` still returns `Option<Teleop>`.
- `RemoteState` is `{ t, pose: Pose2, twist: Twist, path: Vec<Pose2>,
  obstacles: Vec<[f64; 2]> }`.
- `prefix` defaults to `"terra"`. A `password` key in the file is an error.
- The rover link sends nothing while no router is connected. Each topic
  keeps only its newest message, which goes out on connect.
- The rover link re-sends the last `Capabilities` (every 5 s), `TaskStatus`
  and `Status` (every 1 s) by itself, and sends all of them again on
  reconnect. The rover only needs to publish them when they change.
  Every send carries its age (§5), so for `latest()` the age means "time
  since the rover's tick published it", and `rovers()` only counts
  messages the tick produced in the last `ROVER_SEEN_WINDOW`. This holds
  even for a base that has just started. If the tick hangs, the rover drops
  out because no new `Pose` arrives. A large `TaskStatus`/`Status` age is
  normal (they are sent on change). Use `latest::<Pose>` for liveness.
- A `Task` replaced by a newer one before the tick took it is counted as
  `dropped`; newest wins. Task commands queue (16 max, in order).
- `BaseLink` always watches every rover → base topic except `Depth`. One
  shared `Depth` subscription runs while any `subscribe::<Depth>` is alive.
  Each frame is counted once, and `latest::<Depth>` is cleared when the
  last subscription is dropped.
- `validate` also checks the specs (duplicate names, defaults that don't
  fit, `min > max`; `ParamError::BadSpec`, also available as
  `check_specs`). It accepts an `Int` for a `Float` parameter and returns
  it as a `Float`.
- The endpoint is also checked by Zenoh's own `EndPoint` parser (with the
  `zenoh` feature).
- `TaskState` is tagged `kind` (`"state":{"kind":"running",…}`).
- `rovers()` uses `ROVER_SEEN_WINDOW = 3 s`. `stats()` (on `Link`,
  `RoverLink` and `BaseLink`) also counts `received`.
- Wire `version` is `WIRE_VERSION = 1` for every message.
