//! A rover and a base station talking through terra-comms, in one process.
//!
//!     cargo run -p terra-comms --example demo
//!
//! It starts a local Zenoh router, then:
//! - `run_rover`: a pretend rover (a 20 Hz loop in a thread). It advertises
//!   one controller, checks incoming tasks, drives toward the goal, and holds
//!   still on e-stop.
//! - `run_base`: a base station. It reads the rover's capabilities, validates
//!   a parameter form, sends a task, sees a bad task rejected, e-stops and
//!   resumes, then waits until the goal is reached.
//!
//! For a real router, drop `local_router` and put its endpoint in `[comms]`.
use std::net::TcpListener;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use terra_comms::*;
use zenoh::Wait;

const ROVER_ID: u64 = 7;
const TICK: Duration = Duration::from_millis(50); // 20 Hz

fn main() {
    let port = free_port();
    let _router = local_router(port);

    let stop = Arc::new(AtomicBool::new(false));
    let rover = {
        let stop = stop.clone();
        thread::spawn(move || run_rover(port, &stop))
    };
    run_base(port);
    stop.store(true, Ordering::Relaxed);
    rover.join().unwrap();
}

/// The `[comms]` table. In terra-av it is part of `terra-av.toml`; the
/// password, if any, comes from the Keychain via `with_password`.
fn config(port: u16, rover_id: Option<u64>) -> CommsConfig {
    let id = rover_id.map(|id| format!("rover_id = {id}\n")).unwrap_or_default();
    let toml = format!("[comms]\nendpoint = \"tcp/127.0.0.1:{port}\"\nprefix = \"demo\"\n{id}");
    CommsConfig::from_toml(&toml).expect("valid [comms]")
}

// ------------------------------------------------------------------ rover

/// The one controller this rover offers. In terra-av this comes from the
/// controller registry.
fn drive_pose() -> StageOption {
    StageOption {
        id: "drive_pose".into(),
        version: "1".into(),
        description: "Steer straight onto the goal".into(),
        params: vec![ParamSpec {
            name: "speed".into(),
            kind: ParamKind::Float,
            default: ParamValue::Float(0.5),
            min: Some(0.05),
            max: Some(1.0),
            unit: Some("m/s".into()),
            help: "Cruise speed".into(),
        }],
        goals: vec![GoalKind::Local],
    }
}

/// The rover's own check of a task (it never trusts the UI's check).
fn check(controllers: &[StageOption], task: &Task) -> Result<Params, String> {
    let c = controllers
        .iter()
        .find(|c| c.id == task.controller)
        .ok_or_else(|| format!("unknown controller {:?}", task.controller))?;
    if let Some(kind) = task.goal.kind()
        && !c.accepts(kind)
    {
        return Err(format!("{} doesn't take {kind:?} goals", c.id));
    }
    validate(&c.params, &task.params).map_err(|e| e.to_string())
}

fn run_rover(port: u16, stop: &AtomicBool) {
    let link = connect(&config(port, Some(ROVER_ID))).expect("valid config").rover().expect("rover_id is set");
    let clock = Instant::now();
    let now = || clock.elapsed().as_secs_f64();

    // Published once; the link re-sends it every 5 s and after a reconnect.
    let controllers = vec![drive_pose()];
    link.publish(&Capabilities {
        t: now(),
        controllers: controllers.clone(),
        planners: vec![],
        default_controller: "drive_pose".into(),
        software: env!("CARGO_PKG_VERSION").into(),
    });

    let (mut x, mut y) = (0.0_f64, 0.0_f64);
    let mut task: Option<(Task, Params)> = None; // accepted task, params with defaults
    let mut state = TaskState::Idle;
    let mut started = 0.0;
    let mut last_rejected: Option<Rejection> = None;
    let mut last_published: Option<(TaskStatus, bool)> = None;

    while !stop.load(Ordering::Relaxed) {
        let estop = link.estop();

        // 1. A new task. A bad one is rejected; the current task carries on.
        if let Some(new) = link.take_task() {
            match check(&controllers, &new) {
                Ok(params) => {
                    println!("rover: accepted task {} ({})", new.id, new.controller);
                    task = Some((new, params));
                    state = TaskState::Running { distance_to_goal: 0.0, elapsed: 0.0 };
                    started = now();
                }
                Err(reason) => {
                    println!("rover: rejected task {}: {reason}", new.id);
                    last_rejected = Some(Rejection { task_id: new.id, reason, t: now() });
                }
            }
        }

        // 2. Commands, only for the current task. Nothing resumes by itself.
        while let Some(cmd) = link.take_task_command() {
            let refuse = match &task {
                Some((t, _)) if t.id != cmd.task_id => Some("not the current task"),
                None => Some("no task"),
                Some(_) if estop && cmd.action == TaskAction::Resume => Some("e-stop engaged"),
                Some(_) => None,
            };
            if let Some(reason) = refuse {
                last_rejected = Some(Rejection { task_id: cmd.task_id, reason: reason.into(), t: now() });
                continue;
            }
            println!("rover: {:?} task {}", cmd.action, cmd.task_id);
            state = match cmd.action {
                TaskAction::Resume => TaskState::Running { distance_to_goal: 0.0, elapsed: 0.0 },
                TaskAction::Pause => TaskState::Paused,
                TaskAction::Clear => {
                    task = None;
                    TaskState::Idle
                }
            };
        }

        // 3. E-stop: hold still and ask for a human. The goal stays.
        if estop && task.is_some() && !matches!(state, TaskState::NeedsAttention { .. }) {
            state = TaskState::NeedsAttention { reason: Attention::EStop, since: now() };
        }

        // 4. Drive (stand-in for the pipeline): straight at the goal.
        if let (Some((t, params)), TaskState::Running { .. }) = (&task, &state)
            && let Goal::Pose { x: gx, y: gy, tolerance, .. } = t.goal
        {
            let speed = match params["speed"] {
                ParamValue::Float(s) => s,
                _ => unreachable!("validated"),
            };
            let (dx, dy) = (gx - x, gy - y);
            let d = dx.hypot(dy);
            if d <= tolerance {
                println!("rover: reached ({x:.2}, {y:.2})");
                state = TaskState::Reached;
            } else {
                let step = (speed * TICK.as_secs_f64()).min(d);
                (x, y) = (x + dx / d * step, y + dy / d * step);
                state = TaskState::Running { distance_to_goal: d, elapsed: now() - started };
            }
        }

        // 5. Publish. Pose every tick (it's how the base knows we're alive);
        //    status only when it changes (the link repeats it).
        link.publish(&Pose { t: now(), x, y, yaw: 0.0, tracked: true });
        let status = TaskStatus {
            t: 0.0, // set below, so the change check ignores it
            task_id: task.as_ref().map(|(t, _)| t.id),
            state: state.clone(),
            controller: task.as_ref().map(|(t, _)| t.controller.clone()).unwrap_or_default(),
            controller_version: "1".into(),
            params: task.as_ref().map(|(_, p)| p.clone()).unwrap_or_default(),
            goal: task.as_ref().map(|(t, _)| t.goal.clone()).unwrap_or(Goal::Stop),
            timeout: None,
            last_rejected: last_rejected.clone(),
        };
        if last_published.as_ref() != Some(&(status.clone(), estop)) {
            link.publish(&TaskStatus { t: now(), ..status.clone() });
            link.publish(&Status { t: now(), estopped: estop, link_ok: true, detail: String::new(), gps_anchor: None });
            last_published = Some((status, estop));
        }
        thread::sleep(TICK);
    }
}

// ------------------------------------------------------------------ base station

fn run_base(port: u16) {
    let base = connect(&config(port, None)).expect("valid config").base();
    let status = || base.latest::<TaskStatus>(ROVER_ID).map(|(s, _)| s);

    // 1. Find the rover and read what it can run (drives the UI dropdown).
    let (caps, _) = wait("capabilities", || base.latest::<Capabilities>(ROVER_ID));
    let ids: Vec<_> = caps.controllers.iter().map(|c| c.id.as_str()).collect();
    println!("base:  rovers online {:?}, rover {ROVER_ID} offers {ids:?}", base.rovers());
    let controller = caps.controller(&caps.default_controller).expect("default is listed");

    // 2. Validate the parameter form before sending. Same check as the rover.
    let mut params = Params::new();
    params.insert("speed".into(), ParamValue::Float(5.0));
    if let Err(e) = validate(&controller.params, &params) {
        println!("base:  form error, shown next to the field: {e}");
    }
    params.insert("speed".into(), ParamValue::Float(0.8));
    validate(&controller.params, &params).expect("valid form");

    // 3. Send a task: where (goal) and how (controller + params).
    let task = Task {
        id: 1,
        goal: Goal::Pose { x: 3.0, y: 1.0, yaw: None, tolerance: 0.1 },
        controller: controller.id.clone(),
        params,
        planner: None,
        planner_params: Params::new(),
    };
    base.send(ROVER_ID, &task);
    wait("running", || status().filter(|s| matches!(s.state, TaskState::Running { .. })));
    println!("base:  task 1 is running");

    // 4. A task the rover can't run is rejected; task 1 keeps going.
    base.send(ROVER_ID, &Task { id: 2, controller: "warp_drive".into(), ..task.clone() });
    let s = wait("rejection", || status().filter(|s| s.last_rejected.is_some()));
    println!("base:  rover refused task 2 ({}); still on task {:?}", s.last_rejected.unwrap().reason, s.task_id);

    // 5. E-stop: the rover holds still and asks for attention. The goal stays.
    base.send(ROVER_ID, &Estop { t: 0.0, engaged: true });
    let s = wait("attention", || status().filter(|s| matches!(s.state, TaskState::NeedsAttention { .. })));
    println!("base:  ALERT rover {ROVER_ID}: {:?}, goal still {:?}", s.state, s.goal);

    // 6. A human releases the e-stop, then resumes; releasing alone doesn't
    //    resume. Back to back is safe: this base sends commands in order and
    //    the e-stop has the higher priority, so Resume can't overtake the
    //    release. A real UI also greys out Resume while `Status.estopped`
    //    is true, for e-stops this base didn't send (the phone's, another
    //    base's); the rover would refuse it.
    base.send(ROVER_ID, &Estop { t: 0.0, engaged: false });
    base.send(ROVER_ID, &TaskCommand { task_id: 1, action: TaskAction::Resume });

    // 7. Wait for the goal. Pose arrives at 20 Hz, so its age shows liveness.
    wait("reached", || status().filter(|s| s.state == TaskState::Reached));
    let (pose, age) = base.latest::<Pose>(ROVER_ID).expect("pose");
    println!("base:  goal reached; rover at ({:.2}, {:.2}), pose {:.0} ms old", pose.x, pose.y, age * 1000.0);

    let stats = base.stats();
    for topic in [Topic::Task, Topic::Estop, Topic::Pose, Topic::TaskStatus] {
        let s = stats[&topic];
        println!("base:  {:<12} sent {:>3}  received {:>3}  dropped {}", topic.as_str(), s.sent, s.received, s.dropped);
    }
}

// ------------------------------------------------------------------ helpers

fn wait<T>(what: &str, mut f: impl FnMut() -> Option<T>) -> T {
    let deadline = Instant::now() + Duration::from_secs(15);
    loop {
        if let Some(v) = f() {
            return v;
        }
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
        thread::sleep(Duration::from_millis(20));
    }
}

fn free_port() -> u16 {
    TcpListener::bind("127.0.0.1:0").unwrap().local_addr().unwrap().port()
}

/// A Zenoh router on localhost, standing in for the cloud-hosted one.
fn local_router(port: u16) -> zenoh::Session {
    let mut c = zenoh::Config::default();
    c.insert_json5("mode", r#""router""#).unwrap();
    c.insert_json5("listen/endpoints", &format!(r#"["tcp/127.0.0.1:{port}"]"#)).unwrap();
    c.insert_json5("scouting/multicast/enabled", "false").unwrap();
    zenoh::open(c).wait().expect("start the local router")
}
