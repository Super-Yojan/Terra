//! L5 through an MCP agent: tool calls get the same checks as a built-in model reply.
use terra_autonomy::*;
use terra_mapping::MapSnapshot;
use terra_navigation::{Pose, Twist};

/// 12 x 12 m at 0.1 m: free inside, unknown border, a wall at x 7.0..7.3 for y 3..9.
fn room() -> MapSnapshot {
    let mut map = MapSnapshot {
        width: 120,
        height: 120,
        resolution: 0.1,
        origin_x: 0.,
        origin_y: 0.,
        occupancy: vec![0; 14_400],
    };
    for r in 0..120 {
        for c in 0..120 {
            let x = c as f64 * 0.1;
            let y = r as f64 * 0.1;
            map.occupancy[r * 120 + c] = if (7.0..7.3).contains(&x) && (3.0..9.0).contains(&y) {
                100
            } else if x >= 10.5 || x < 0.5 || y < 0.5 || y >= 11.5 {
                -1
            } else {
                0
            };
        }
    }
    map
}

fn spec(agent: TaskAgent) -> TaskSpec {
    TaskSpec {
        task_id: "task-1".into(),
        instruction: "Drive to the wall.".into(),
        budget_seconds: Some(600.),
        max_steps: Some(10),
        agent: Some(agent),
    }
}

struct Sim {
    arbiter: AutonomyArbiter,
    map: MapSnapshot,
    pose: Pose,
    measured: Twist,
    now: f64,
}

impl Sim {
    fn new() -> Self {
        let mut arbiter = AutonomyArbiter::default();
        arbiter.set_task_agent(Some("qwen".into()));
        Self {
            arbiter,
            map: room(),
            pose: Pose {
                x: 3.,
                y: 6.,
                yaw: 0.,
            },
            measured: Twist::default(),
            now: 0.,
        }
    }
    fn tick(&mut self) -> ArbiterOutput {
        self.now += 0.1;
        let output = self.arbiter.step(ArbiterInput {
            now: self.now,
            pose: self.pose,
            pose_time: self.now,
            map: Some(&self.map),
            map_time: Some(self.now),
            map_revision: 1,
            measured: self.measured,
            healthy: true,
        });
        self.measured = output.twist;
        self.pose.yaw += self.measured.angular * 0.1;
        self.pose.x += self.measured.linear * self.pose.yaw.cos() * 0.1;
        self.pose.y += self.measured.linear * self.pose.yaw.sin() * 0.1;
        output
    }
    fn start(&mut self, agent: TaskAgent) {
        self.tick();
        self.arbiter
            .set_level_with_task(LevelRequest::new(Level::Task, "l5"), Some(spec(agent)));
        assert_eq!(self.arbiter.request_receipt(), Some(("l5", true)));
        self.tick();
    }
    fn view(&mut self) -> String {
        let map = self.map.clone();
        self.arbiter
            .agent_view(Some((&map, self.now)), self.pose, self.now)
            .unwrap()
    }
    fn go(&mut self, waypoints: &[RequestedWaypoint]) -> Result<AgentOutcome, String> {
        let map = self.map.clone();
        self.arbiter
            .agent_go(waypoints, None, Some((&map, self.now)), self.pose, self.now)
    }
    fn action(&mut self, action: TaskActionKind, token: &str) -> Result<(), &'static str> {
        self.arbiter.apply_task_action(
            TaskAction {
                task_id: "task-1".into(),
                action,
                text: None,
                token: token.into(),
                run_id: None,
            },
            self.now,
        )
    }
}

#[test]
fn an_mcp_task_needs_an_attached_agent_and_never_queries_the_builtin_model() {
    let mut arbiter = AutonomyArbiter::default();
    arbiter.set_task_model(Some("apple".into()));
    arbiter.set_level_with_task(
        LevelRequest::new(Level::Task, "l5"),
        Some(spec(TaskAgent::Mcp)),
    );
    assert_eq!(arbiter.request_receipt(), Some(("l5", false)));

    // An agent alone is enough to offer the task level.
    let mut sim = Sim::new();
    sim.start(TaskAgent::Mcp);
    for _ in 0..20 {
        let output = sim.tick();
        assert_eq!(output.twist, Twist::default());
        assert_eq!(output.task.as_ref().unwrap().reason, "awaiting_agent");
        assert!(sim.arbiter.take_task_query().is_none());
    }
    assert_eq!(sim.arbiter.agent_task_id(), Some("task-1"));
}

#[test]
fn go_to_is_checked_like_a_model_reply_then_driven_on_the_l3_route() {
    let mut sim = Sim::new();
    sim.start(TaskAgent::Mcp);
    let view = sim.view();
    assert!(view.contains("Drive to the wall."));
    assert!(view.contains("O1"), "{view}");
    assert!(!view.contains("Reply with one JSON object"));

    // Inside the wall and an unknown id are refused, and nothing moves.
    let refused = sim.go(&[RequestedWaypoint::Point(7.1, 6.)]).unwrap_err();
    assert!(
        refused.contains("not reachable seen free space"),
        "{refused}"
    );
    assert!(sim.go(&[RequestedWaypoint::Target("F99".into())]).is_err());
    assert_eq!(sim.tick().twist, Twist::default());

    let AgentOutcome::Following(plan) = sim.go(&[RequestedWaypoint::Target("O1".into())]).unwrap()
    else {
        panic!("expected a plan");
    };
    let (gx, gy) = (plan[0].x, plan[0].y);
    let mut moved = false;
    loop {
        let output = sim.tick();
        moved |= output.twist.linear > 0.;
        let task = output.task.unwrap();
        if task.phase == TaskPhase::Planning {
            assert!(
                task.history.iter().any(|h| h.text.contains("reached O1")),
                "{task:?}"
            );
            break;
        }
        assert!(sim.now < 120., "never arrived: {:?}", sim.pose);
    }
    assert!(moved);
    assert!((sim.pose.x - gx).hypot(sim.pose.y - gy) < 1.0);

    sim.arbiter
        .agent_finish(true, "beside the wall", sim.now)
        .unwrap();
    let output = sim.tick();
    assert_eq!(output.task.unwrap().phase, TaskPhase::Completed);
    assert!(sim.go(&[RequestedWaypoint::Target("F1".into())]).is_err());
}

#[test]
fn agent_writes_need_an_mcp_task_at_the_task_level_without_a_stop() {
    let mut sim = Sim::new();
    sim.arbiter.set_task_model(Some("apple".into()));
    sim.start(TaskAgent::Builtin);
    let err = sim
        .go(&[RequestedWaypoint::Target("O1".into())])
        .unwrap_err();
    assert!(err.contains("built-in model"), "{err}");

    let mut sim = Sim::new();
    sim.start(TaskAgent::Mcp);
    sim.arbiter.set_safety(
        SafetyRequest {
            action: SafetyAction::Stop,
            token: "stop-1".into(),
        },
        true,
    );
    let err = sim
        .go(&[RequestedWaypoint::Target("O1".into())])
        .unwrap_err();
    assert!(err.starts_with("not_in_control"), "{err}");
    assert_eq!(sim.tick().task.unwrap().reason, "operator_stop");
}

#[test]
fn raw_drive_needs_operator_approval_is_capped_and_ends_with_the_grant() {
    let mut sim = Sim::new();
    sim.start(TaskAgent::Mcp);
    let err = sim.arbiter.agent_drive(1.0, 0., 1., sim.now).unwrap_err();
    assert!(err.starts_with("drive_not_approved"), "{err}");
    assert_eq!(
        sim.action(TaskActionKind::ApproveDrive, "approve-0"),
        Err("no_drive_request")
    );

    sim.arbiter
        .agent_request_drive("nudge closer to the wall", 120., sim.now)
        .unwrap();
    let output = sim.tick();
    let task = output.task.unwrap();
    assert_eq!(task.reason, "awaiting_drive_approval");
    assert_eq!(task.drive_request.as_ref().unwrap().seconds, 60.);
    assert_eq!(output.twist, Twist::default());

    sim.action(TaskActionKind::ApproveDrive, "approve-1")
        .unwrap();
    // Asked for 2 m/s: capped to 0.5, then the planner decides what is safe.
    sim.arbiter.agent_drive(2.0, 0., 5., sim.now).unwrap();
    let output = sim.tick();
    assert_eq!(output.status.active_source, "task_agent_drive");
    assert!(output.twist.linear > 0. && output.twist.linear <= TASK_DRIVE_MAX_LINEAR + 1e-9);
    assert!(output.task.unwrap().drive_remaining_seconds.unwrap() > 59.);

    // Each command lasts at most 1 s.
    for _ in 0..12 {
        sim.tick();
    }
    assert_eq!(sim.tick().twist, Twist::default());

    // The operator can end the grant at any time.
    sim.arbiter.agent_drive(0.3, 0., 1., sim.now).unwrap();
    assert!(sim.tick().twist.linear > 0.);
    sim.action(TaskActionKind::DenyDrive, "deny-1").unwrap();
    assert_eq!(sim.tick().twist, Twist::default());
    assert!(sim.arbiter.agent_drive(0.3, 0., 1., sim.now).is_err());
}

#[test]
fn a_drive_grant_expires_on_the_task_clock() {
    let mut sim = Sim::new();
    sim.start(TaskAgent::Mcp);
    sim.arbiter
        .agent_request_drive("look around", 2., sim.now)
        .unwrap();
    sim.action(TaskActionKind::ApproveDrive, "approve-1")
        .unwrap();
    for _ in 0..25 {
        sim.arbiter.agent_drive(0., 0.5, 1., sim.now).ok();
        sim.tick();
    }
    let output = sim.tick();
    let task = output.task.unwrap();
    assert!(task.drive_remaining_seconds.is_none(), "{task:?}");
    assert!(task.history.iter().any(|h| h.text == "drive grant ended"));
    assert_eq!(output.twist, Twist::default());
}

#[test]
fn a_silent_agent_ends_the_task() {
    let mut sim = Sim::new();
    sim.start(TaskAgent::Mcp);
    while sim.now < TASK_QUERY_TIMEOUT_SECONDS + 1. {
        sim.tick();
    }
    let task = sim.tick().task.unwrap();
    assert_eq!(task.phase, TaskPhase::Failed);
    assert_eq!(task.reason, "agent_timeout");
}

#[test]
fn ask_operator_holds_until_the_reply() {
    let mut sim = Sim::new();
    sim.start(TaskAgent::Mcp);
    sim.arbiter.agent_ask("Which wall?").unwrap();
    let task = sim.tick().task.unwrap();
    assert_eq!(task.phase, TaskPhase::WaitingOperator);
    assert_eq!(task.question.as_deref(), Some("Which wall?"));
    assert!(sim.go(&[RequestedWaypoint::Target("O1".into())]).is_err());
    sim.arbiter
        .apply_task_action(
            TaskAction {
                task_id: "task-1".into(),
                action: TaskActionKind::Reply,
                text: Some("The one ahead.".into()),
                token: "reply-1".into(),
                run_id: None,
            },
            sim.now,
        )
        .unwrap();
    assert_eq!(sim.tick().task.unwrap().phase, TaskPhase::Planning);
    assert!(sim.go(&[RequestedWaypoint::Target("O1".into())]).is_ok());
}

#[test]
fn approved_agent_can_back_up_on_a_stale_map() {
    let mut sim = Sim::new();
    sim.start(TaskAgent::Mcp);
    sim.arbiter
        .agent_request_drive("back away from the wall", 30., sim.now)
        .unwrap();
    sim.tick();
    sim.action(TaskActionKind::ApproveDrive, "approve-r")
        .unwrap();
    let map_time = sim.now;
    let stale = |sim: &mut Sim| {
        sim.now += 5.;
        let map = sim.map.clone();
        sim.arbiter.step(ArbiterInput {
            now: sim.now,
            pose: sim.pose,
            pose_time: sim.now,
            map: Some(&map),
            map_time: Some(map_time),
            map_revision: 1,
            measured: Twist::default(),
            healthy: true,
        })
    };
    sim.arbiter.agent_drive(-0.3, 0., 1., sim.now + 5.).unwrap();
    let output = stale(&mut sim);
    assert_eq!(output.status.active_source, "task_agent_drive");
    assert!(output.twist.linear < 0., "{}", output.status.reason);

    // Forward on the same stale map still holds.
    sim.arbiter.agent_drive(0.3, 0., 1., sim.now + 5.).unwrap();
    assert_eq!(stale(&mut sim).status.reason, "map_stale");
}
