//! Tasks from the base station: where to go and how (`terra_comms::Task`).
//! The rules are in `crates/terra-comms/API.md` §4.5–4.9; in short:
//!
//! - A task is **persistent**. Nothing on the rover clears it: only a human
//!   replaces it (a `Task` with a new id) or clears it (`TaskCommand`).
//! - When something goes wrong the rover **holds still and asks for
//!   attention**; the goal stays. Nothing resumes by itself.
//! - The same id again is an **update**: swap the controller, its
//!   parameters or the planner while keeping the goal, even while moving.
//!
//! ```text
//!            new task                      goal reached
//!   Idle ──────────────► Running ◄────────────────────► Reached
//!     ▲                   │  ▲      pushed away again
//!     │ Clear             │  │ Resume (refused while e-stopped)
//!     │        Pause, or  ▼  │
//!     └────── a problem: Paused / NeedsAttention(reason)
//! ```
//!
//! Each tick the pipeline calls [`Tasks::set_task`] / [`Tasks::command`]
//! for what arrived, then [`Tasks::step`] once, which returns the twist the
//! task wants (zero unless it is `Running`).
use std::path::PathBuf;

use terra_comms::{
    Attention, COMMAND_LEASE, Goal as TaskGoal, GoalKind, Params, Rejection, RemoteState, Task,
    TaskAction, TaskCommand, TaskState, TaskStatus,
};

use crate::estimation::{GpsAnchor, State};
use crate::mapping::Map;
use crate::planning::registry::{self, ControllerSpec};
use crate::planning::{ExploreStep, Explorer, Goal, Navigator, Path, PoseGoal};
use crate::vehicle::{Twist, VehicleModel};

/// A `Running` task without a tracked pose for this long (s) asks for
/// attention. Shorter gaps (ARKit relocalising) only pause the driving.
const POSE_LOST_AFTER_S: f64 = 2.0;
/// Progress, for the stuck rule: this much closer to the goal (m)…
const PROGRESS_M: f64 = 0.1;
/// …or, when turning to the goal heading, this much closer to it (rad).
const PROGRESS_RAD: f64 = 0.1;
/// A `Reached` rover drives back only once it is pushed further than
/// `tolerance ×` this from the goal, so it does not hunt at the edge.
const REDRIVE_FACTOR: f64 = 1.5;
/// A `remote` task waits this long (s) for the first `remote/cmd`: the
/// remote program can only answer after it has seen a `RemoteState`.
const REMOTE_START_GRACE_S: f64 = 3.0;

/// A planner or controller picked by id, with its parameter values.
#[derive(Clone, Debug, PartialEq)]
pub struct Choice {
    pub id: String,
    pub params: Params,
}

pub struct Tasks {
    /// The path planner for a task that names none: the one from `terra-av.toml`.
    default_planner: Choice,
    current: Option<ActiveTask>,
    /// The newest refused request, reported in `TaskStatus`.
    last_rejected: Option<Rejection>,
    /// Where the current task is kept across restarts. `None`: not kept.
    file: Option<PathBuf>,
}

/// The accepted task and everything built for it.
struct ActiveTask {
    /// As accepted; `params` are the checked values with defaults filled in.
    task: Task,
    controller_version: &'static str,
    /// The controller takes no goal: it is driven by a stick (`teleop`).
    stick_driven: bool,
    /// The task's planner and controller.
    navigator: Navigator,
    /// Present only for `TaskGoal::Explore`: picks frontiers and scans in place.
    explorer: Option<Explorer>,
    phase: Phase,
    /// The goal in the world frame, as last given to the controller.
    /// `None` for `Goal::Stop`, and for a GPS goal not yet converted.
    goal: Option<PoseGoal>,
    /// Distance from the rover to `goal` at the last tick with a pose (m).
    distance_to_goal: f64,
    /// The current stretch of `Running`; `None` until its first tick.
    run: Option<Run>,
    /// When the pose was lost (frame time, s); `None` while tracked.
    untracked_since: Option<f64>,
}

#[derive(Clone, Debug, PartialEq)]
enum Phase {
    Running,
    Reached,
    Paused,
    NeedsAttention { reason: Attention, since: f64 },
}

/// One stretch of `Running`: from a new task, an update, a `Resume`, or a
/// drive back to the goal, until the task leaves `Running`.
struct Run {
    /// Frame time of its first tick (s).
    started: f64,
    /// The controller's time limit for this stretch (s). `None`: no limit.
    timeout: Option<f64>,
    // The stuck rule: what was left to do when progress was last made.
    distance_left: f64,
    turn_left: f64,
    progress_at: f64,
}

/// What [`Tasks::step`] needs to know about this tick.
pub struct Tick<'a> {
    /// Frame time (s).
    pub t: f64,
    /// The estimated state and the map; `None` while the pose is not tracked.
    pub tracked: Option<(&'a State, &'a Map)>,
    pub vehicle: &'a dyn VehicleModel,
    /// Any e-stop engaged (the phone's, or a base station's).
    pub estop: bool,
    pub gps_anchor: Option<GpsAnchor>,
    /// The base station's newest `remote/cmd` twist; `None` when silent.
    pub remote_command: Option<Twist>,
}

/// What the task wants this tick.
pub struct Step {
    pub twist: Twist,
    /// For the `remote` controller: what to publish as `remote/state`.
    pub remote_state: Option<RemoteState>,
}

impl Step {
    const HOLD: Step = Step { twist: Twist::STOP, remote_state: None };
}

impl Tasks {
    /// `file` is where the current task is kept across restarts. If it
    /// holds a task, that task comes back as `NeedsAttention(RoverRestarted)`:
    /// the goal survives a crash or reboot, but the rover never starts
    /// driving on its own.
    pub fn new(default_planner: Choice, file: Option<PathBuf>) -> Self {
        let mut tasks = Self { default_planner, current: None, last_rejected: None, file };
        if let Some(task) = tasks.load() {
            match tasks.accept(task) {
                Ok(mut restored) => {
                    // `since` is in frame time, which is not known here (and
                    // restarts with the phone): 0 means "since startup".
                    restored.phase = Phase::NeedsAttention { reason: Attention::RoverRestarted, since: 0.0 };
                    tasks.current = Some(restored);
                }
                // E.g. saved by a build that had a controller this one lacks.
                Err(reason) => eprintln!("terra-av: saved task dropped: {reason}"),
            }
        }
        tasks
    }

    // ------------------------------------------------------- requests

    /// A `Task` arrived. A new id replaces the current task; the current id
    /// updates it. A task that fails a check is refused with a reason and
    /// the current task carries on untouched.
    pub fn set_task(&mut self, task: Task, gps_anchor: Option<&GpsAnchor>, t: f64) {
        let id = task.id;
        let is_gps_goal = matches!(task.goal, TaskGoal::Gps { .. });
        let accepted = if is_gps_goal && gps_anchor.is_none() {
            Err("no GPS anchor".to_string())
        } else {
            self.accept(task)
        };
        match accepted {
            Ok(active) => {
                self.current = Some(active);
                self.save();
            }
            Err(reason) => self.reject(id, reason, t),
        }
    }

    /// A `TaskCommand` arrived. It must name the current task, so a stale
    /// click cannot act on a newer one. `estop`: any e-stop is engaged.
    pub fn command(&mut self, command: TaskCommand, estop: bool, t: f64) {
        let Some(active) = self.current.as_mut().filter(|a| a.task.id == command.task_id) else {
            return self.reject(command.task_id, "not the current task".into(), t);
        };
        match command.action {
            TaskAction::Clear => self.current = None,
            TaskAction::Pause => active.phase = Phase::Paused,
            TaskAction::Resume => {
                // Releasing an e-stop never resumes; and a resume never
                // overrides an e-stop.
                if estop {
                    return self.reject(command.task_id, "e-stop engaged".into(), t);
                }
                active.start_running();
            }
        }
        self.save();
    }

    /// Check `task` and build its controller and planner (API.md §4.7).
    /// On `Err` nothing has changed.
    fn accept(&self, task: Task) -> Result<ActiveTask, String> {
        // 1. The controller and planner exist and their parameters are valid.
        let controller_spec = registry::controller(&task.controller)?;
        let (params, controller) = controller_spec.build(&task.params)?;
        let planner_choice = match &task.planner {
            Some(id) => Choice { id: id.clone(), params: task.planner_params.clone() },
            None => self.default_planner.clone(),
        };
        let (_, path_planner) = registry::planner(&planner_choice.id)?.build(&planner_choice.params)?;

        // 2. The controller can drive to this kind of goal.
        check_goal(&task.goal, &controller_spec)?;

        // 3. An update keeps the goal, and the phase: a running task keeps
        //    running, a paused one stays paused until `Resume`.
        let phase = match &self.current {
            Some(current) if current.task.id == task.id => {
                if current.task.goal != task.goal {
                    return Err("an update must keep the goal; send a new task id for a new goal".into());
                }
                current.phase.clone()
            }
            _ => Phase::Running,
        };

        let explorer = match task.goal {
            TaskGoal::Explore { budget_s } => Some(Explorer::new(budget_s)),
            _ => None,
        };
        Ok(ActiveTask {
            task: Task { params, ..task },
            controller_version: controller_spec.version,
            stick_driven: controller_spec.goals.is_empty(),
            navigator: Navigator::new(path_planner, controller),
            explorer,
            phase,
            goal: None,
            distance_to_goal: 0.0,
            // A fresh stretch: the new controller sets a fresh time limit.
            run: None,
            untracked_since: None,
        })
    }

    fn reject(&mut self, task_id: u64, reason: String, t: f64) {
        self.last_rejected = Some(Rejection { task_id, reason, t });
    }

    // ----------------------------------------------------------- tick

    /// An autonomous controller is driving right now. While it is, the
    /// base station's teleop stick is ignored.
    pub fn is_driving(&self) -> bool {
        self.current
            .as_ref()
            .is_some_and(|active| active.phase == Phase::Running && !active.stick_driven)
    }

    /// One tick of the current task: update its phase, and return the
    /// twist it wants. Anything but `Running` holds still.
    pub fn step(&mut self, tick: &Tick) -> Step {
        let t = tick.t;
        let Some(active) = &mut self.current else {
            return Step::HOLD;
        };

        // An e-stop stops a running task for good: after the release a
        // human must send `Resume`.
        if active.phase == Phase::Running && tick.estop {
            active.needs_attention(Attention::EStop, t);
        }

        // No pose: hold still, and after a while say so.
        let Some((state, map)) = tick.tracked else {
            let since = *active.untracked_since.get_or_insert(t);
            if active.phase == Phase::Running && t - since > POSE_LOST_AFTER_S {
                active.needs_attention(Attention::PoseLost, t);
            }
            return Step::HOLD;
        };
        active.untracked_since = None;

        // Frontier exploration manages its own sequence of frontier poses.
        if let Some(explorer) = &mut active.explorer {
            if active.phase != Phase::Running {
                return Step::HOLD;
            }
            active.run.get_or_insert(Run::without_limits(t));
            match explorer.step(t, state, map, &mut active.navigator, tick.vehicle) {
                ExploreStep::Drive { twist, goal } => {
                    active.goal = goal;
                    active.distance_to_goal = goal.map_or(0.0, |g| g.distance(&state.pose));
                    return Step { twist, remote_state: None };
                }
                ExploreStep::Done => {
                    active.phase = Phase::Reached;
                    active.run = None;
                    active.goal = None;
                    active.distance_to_goal = 0.0;
                    return Step::HOLD;
                }
            }
        }

        // The goal in the world frame. A GPS goal is converted with the
        // newest anchor on every tick, so a refined anchor moves the goal
        // (and the navigator re-plans, because the goal changed).
        active.goal = match &active.task.goal {
            TaskGoal::Stop | TaskGoal::Explore { .. } => None,
            &TaskGoal::Pose { x, y, yaw, tolerance } => Some(PoseGoal { x, y, yaw, tolerance }),
            &TaskGoal::Gps { latitude, longitude, heading, tolerance } => {
                let Some(anchor) = tick.gps_anchor else {
                    if active.phase == Phase::Running {
                        active.needs_attention(Attention::NoGpsAnchor, t);
                    }
                    return Step::HOLD;
                };
                let position = anchor.to_world(latitude, longitude);
                Some(PoseGoal {
                    x: position.x,
                    y: position.y,
                    yaw: heading.map(|heading| anchor.to_world_yaw(heading)),
                    // The goal is only known as well as the anchor is.
                    tolerance: tolerance.max(anchor.accuracy),
                })
            }
        };
        active.distance_to_goal = active.goal.map_or(0.0, |goal| goal.distance(&state.pose));

        // Reached, then pushed away from the goal: drive back, or ask.
        if let (Phase::Reached, Some(goal)) = (&active.phase, &active.goal)
            && active.distance_to_goal > goal.tolerance * REDRIVE_FACTOR
        {
            if active.navigator.controller.returns_to_goal() {
                active.start_running();
            } else {
                active.needs_attention(Attention::Displaced, t);
            }
        }

        if active.phase != Phase::Running {
            return Step::HOLD;
        }

        // ---- Running ----

        // The `remote` controller gets the base station's newest twist.
        let is_remote = match active.navigator.controller.as_remote() {
            Some(remote) => {
                remote.command = tick.remote_command;
                true
            }
            None => false,
        };

        let twist = match active.goal {
            // `Goal::Stop`: no goal to reach, so no route, no time limit and
            // no stuck rule. The controller is still asked: `remote` answers
            // with the base station's twist, the others hold still.
            None => {
                active.run.get_or_insert(Run::without_limits(t));
                let no_path = Path::default();
                active.navigator.controller.follow(state, map, &Goal::None, &no_path, tick.vehicle)
            }
            Some(goal) => {
                if goal.reached(&state.pose) {
                    active.phase = Phase::Reached;
                    active.run = None;
                    return Step::HOLD;
                }
                let Ok(twist) = active.navigator.drive(state, map, &goal, tick.vehicle) else {
                    active.needs_attention(Attention::ControllerError("no route to the goal".into()), t);
                    return Step::HOLD;
                };

                // The first tick of this stretch: now that there is a
                // route, ask the controller for its time limit.
                let distance_left = active.distance_to_goal;
                let turn_left = goal.yaw_error(&state.pose).abs();
                let timeout = active.navigator.timeout(state, &goal, tick.vehicle);
                let run = active.run.get_or_insert(Run {
                    started: t,
                    timeout,
                    distance_left,
                    turn_left,
                    progress_at: t,
                });

                // Timeout: this stretch has taken longer than the limit.
                if run.timeout.is_some_and(|limit| t - run.started > limit) {
                    active.needs_attention(Attention::Timeout, t);
                    return Step::HOLD;
                }
                // Stuck: no progress for longer than the controller allows.
                let progressed = distance_left < run.distance_left - PROGRESS_M
                    || turn_left < run.turn_left - PROGRESS_RAD;
                if progressed {
                    run.distance_left = distance_left;
                    run.turn_left = turn_left;
                    run.progress_at = t;
                } else if let Some(limit) = active.navigator.controller.stuck_after()
                    && t - run.progress_at > limit
                {
                    active.needs_attention(Attention::Stuck, t);
                    return Step::HOLD;
                }
                twist
            }
        };

        if !is_remote {
            return Step { twist, remote_state: None };
        }

        // `remote`: stop and ask if the base station has gone silent (the
        // link reports no command after `COMMAND_LEASE` without one).
        let started = active.run.as_ref().map_or(t, |run| run.started);
        if tick.remote_command.is_none() && t - started > REMOTE_START_GRACE_S.max(COMMAND_LEASE.as_secs_f64()) {
            active.needs_attention(Attention::RemoteSilent, t);
            return Step::HOLD;
        }
        // Tell the remote program what the rover sees.
        let wire_pose = |pose: &crate::geometry::Pose2| terra_comms::Pose2 { x: pose.x, y: pose.y, yaw: pose.yaw };
        let remote_state = RemoteState {
            t,
            pose: wire_pose(&state.pose),
            twist: terra_comms::Twist { v: state.twist.v, w: state.twist.w },
            path: active.navigator.path().iter().map(wire_pose).collect(),
            obstacles: map.obstacles.iter().map(|o| [o.x, o.y]).collect(),
        };
        Step { twist, remote_state: Some(remote_state) }
    }

    // --------------------------------------------------------- status

    /// The task's state for the base station. Carries the exact controller,
    /// version and parameters, so a run can be reproduced from the log.
    pub fn status(&self, t: f64) -> TaskStatus {
        let Some(active) = &self.current else {
            return TaskStatus {
                t,
                task_id: None,
                state: TaskState::Idle,
                controller: String::new(),
                controller_version: String::new(),
                params: Params::new(),
                goal: TaskGoal::Stop,
                timeout: None,
                last_rejected: self.last_rejected.clone(),
            };
        };
        let state = match &active.phase {
            Phase::Running => TaskState::Running {
                distance_to_goal: active.distance_to_goal,
                elapsed: active.run.as_ref().map_or(0.0, |run| t - run.started),
            },
            Phase::Reached => TaskState::Reached,
            Phase::Paused => TaskState::Paused,
            Phase::NeedsAttention { reason, since } => {
                TaskState::NeedsAttention { reason: reason.clone(), since: *since }
            }
        };
        // Keep `Goal::Explore` on `TaskStatus.goal` so the base station knows
        // the task is exploring (the active frontier route is on `PlannedPath`).
        let goal = match (&active.task.goal, active.goal) {
            (TaskGoal::Explore { .. }, _) => active.task.goal.clone(),
            (_, Some(PoseGoal { x, y, yaw, tolerance })) => TaskGoal::Pose { x, y, yaw, tolerance },
            (_, None) => active.task.goal.clone(),
        };
        TaskStatus {
            t,
            task_id: Some(active.task.id),
            state,
            controller: active.task.controller.clone(),
            controller_version: active.controller_version.into(),
            params: active.task.params.clone(),
            goal,
            timeout: active.run.as_ref().and_then(|run| run.timeout),
            last_rejected: self.last_rejected.clone(),
        }
    }

    /// The active task's current planned route in the world frame, or empty
    /// when there is no active task or no route.
    pub fn path(&self) -> &[crate::geometry::Pose2] {
        self.current
            .as_ref()
            .map_or(&[], |active| active.navigator.path())
    }

    /// One line for the phone's screen, e.g. `task 7 · dwa · running, 3.2 m to go`.
    pub fn summary(&self) -> String {
        let Some(active) = &self.current else {
            return "no task".into();
        };
        let phase = match &active.phase {
            Phase::Running if active.goal.is_some() => format!("running, {:.1} m to go", active.distance_to_goal),
            Phase::Running => "running".into(),
            Phase::Reached => "reached".into(),
            Phase::Paused => "paused".into(),
            Phase::NeedsAttention { reason, .. } => format!("needs attention: {reason:?}"),
        };
        format!("task {} · {} · {phase}", active.task.id, active.task.controller)
    }

    // ---------------------------------------------------- persistence

    /// Write the current task to `file` (or remove the file when there is
    /// none). Called whenever the task changes, which is rare, so the tick
    /// is not slowed. A failure is logged; the task still runs.
    fn save(&self) {
        let Some(file) = &self.file else { return };
        let result = match &self.current {
            Some(active) => serde_json::to_vec(&active.task)
                .map_err(std::io::Error::other)
                .and_then(|json| std::fs::write(file, json)),
            None => match std::fs::remove_file(file) {
                Err(e) if e.kind() != std::io::ErrorKind::NotFound => Err(e),
                _ => Ok(()),
            },
        };
        if let Err(e) = result {
            eprintln!("terra-av: could not save the task to {}: {e}", file.display());
        }
    }

    /// The task saved by an earlier run, if any.
    fn load(&self) -> Option<Task> {
        let json = std::fs::read(self.file.as_ref()?).ok()?;
        serde_json::from_slice(&json).ok()
    }
}

impl ActiveTask {
    /// Enter `Running` with a fresh stretch: the time limit and the stuck
    /// rule start again from the current state, and the route is re-planned.
    fn start_running(&mut self) {
        self.phase = Phase::Running;
        self.run = None;
        self.navigator.reset();
        if let Some(explorer) = &mut self.explorer {
            explorer.reset_for_resume();
        }
    }

    /// Stop and wait for a human. The goal is kept.
    fn needs_attention(&mut self, reason: Attention, t: f64) {
        self.phase = Phase::NeedsAttention { reason, since: t };
        self.run = None;
    }
}

impl Run {
    /// A stretch with no time limit and no stuck rule (`Goal::Stop`).
    fn without_limits(t: f64) -> Self {
        Self { started: t, timeout: None, distance_left: 0.0, turn_left: 0.0, progress_at: t }
    }
}

/// Can `controller` drive to `goal`?
///
/// Every controller in the registry that takes goals takes world-frame
/// ones (`GoalKind::Local`); a GPS goal is converted to the world frame for
/// it in [`Tasks::step`]. There is no controller that follows raw GPS, so
/// "lists `Local`" is the whole check.
fn check_goal(goal: &TaskGoal, controller: &ControllerSpec) -> Result<(), String> {
    let tolerance = match goal {
        TaskGoal::Stop => return Ok(()),
        TaskGoal::Explore { budget_s } => {
            if !controller.goals.contains(&GoalKind::Explore) {
                return Err(format!("controller {:?} does not support exploration", controller.id));
            }
            if let Some(b) = budget_s
                && (!b.is_finite() || *b <= 0.0)
            {
                return Err("the exploration budget must be positive".into());
            }
            return Ok(());
        }
        TaskGoal::Pose { tolerance, .. } | TaskGoal::Gps { tolerance, .. } => *tolerance,
    };
    if !controller.goals.contains(&GoalKind::Local) {
        return Err(format!("controller {:?} takes no goal; send Goal::Stop", controller.id));
    }
    if tolerance.is_nan() || tolerance <= 0.0 {
        return Err("the goal tolerance must be positive".into());
    }
    Ok(())
}
