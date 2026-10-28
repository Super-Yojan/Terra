//! L5: the arbiter asks a model for waypoints and drives them on the L3 route.
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

fn spec(instruction: &str) -> TaskSpec {
    TaskSpec {
        task_id: "task-1".into(),
        instruction: instruction.into(),
        budget_seconds: Some(600.),
        max_steps: Some(10),
        agent: None,
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
        arbiter.set_task_model(Some("mock".into()));
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
    /// Tick until the arbiter asks the model, then return the query.
    fn next_query(&mut self, limit: usize) -> TaskQuery {
        for _ in 0..limit {
            let output = self.tick();
            if let Some(query) = self.arbiter.take_task_query() {
                return query;
            }
            assert!(
                !output.task.as_ref().unwrap().phase.terminal(),
                "task ended while waiting for a query: {:?}",
                output.task
            );
        }
        panic!("no query after {limit} ticks; pose {:?}", self.pose);
    }
    fn start(&mut self, instruction: &str) {
        self.tick();
        self.arbiter.set_level_with_task(
            LevelRequest::new(Level::Task, "l5"),
            Some(spec(instruction)),
        );
        assert_eq!(self.arbiter.request_receipt(), Some(("l5", true)));
    }
}

#[test]
fn task_level_needs_an_advertised_model() {
    let mut arbiter = AutonomyArbiter::default();
    arbiter.set_level(LevelRequest::new(Level::Task, "l5"));
    assert_eq!(arbiter.request_receipt(), Some(("l5", false)));
    arbiter.set_task_model(Some("apple-on-device".into()));
    arbiter.set_level(LevelRequest::new(Level::Task, "l5-again"));
    assert_eq!(arbiter.request_receipt(), Some(("l5-again", true)));
}

#[test]
fn inline_task_decodes_only_with_the_task_level() {
    let (r, task) = decode_level_with_task(
        br#"{"level":"task","token":"t","task":{"task_id":"a","instruction":"go to the wall"}}"#,
    )
    .unwrap();
    assert_eq!(r.level, Level::Task);
    assert_eq!(task.unwrap().instruction, "go to the wall");
    assert!(
        decode_level_with_task(
            br#"{"level":"waypoint","token":"t","task":{"task_id":"a","instruction":"x"}}"#
        )
        .is_none()
    );
    assert!(
        decode_level_with_task(br#"{"level":"task","token":"t"}"#)
            .unwrap()
            .1
            .is_none()
    );
    // The plain decoder keeps rejecting the extra field.
    assert!(
        decode_level(br#"{"level":"task","token":"t","task":{"task_id":"a","instruction":"x"}}"#)
            .is_none()
    );
}

#[test]
fn model_waypoint_drives_on_the_l3_route_then_the_task_completes() {
    let mut sim = Sim::new();
    sim.start("Drive to the wall in front of you and stop beside it.");
    let query = sim.next_query(5);
    assert!(query.prompt.contains("Drive to the wall"));
    assert!(query.prompt.contains("O1"), "{}", query.prompt);
    assert!(query.system.contains("never send speeds"));

    // While the model thinks, the rover holds.
    let held = sim.tick();
    assert_eq!(held.twist, Twist::default());
    assert_eq!(held.status.reason, "task_planning");
    assert_eq!(held.task.as_ref().unwrap().query_id, Some(query.query_id));

    assert!(
        sim.arbiter.submit_task_response(
            query.query_id,
            r#"```json
{"decision":"go","waypoints":[{"target":"O1"}],"rationale":"the wall is O1"}
```"#
                .into(),
            Some(2.5),
        )
    );
    assert!(
        !sim.arbiter
            .submit_task_response(query.query_id, "{}".into(), None)
    );
    let output = sim.tick();
    let task = output.task.clone().unwrap();
    assert_eq!(task.phase, TaskPhase::Navigating, "{task:?}");
    assert_eq!(task.plan[0].label, "O1");
    assert_eq!(task.last_latency_seconds, Some(2.5));
    let (gx, gy) = (task.plan[0].x, task.plan[0].y);
    assert!(
        gx < 7.0,
        "approach point is on the rover's side of the wall: {gx}"
    );
    assert!(
        (3.0..9.0).contains(&gy),
        "approach point faces the wall, not its end: {gy}"
    );

    // Driving uses the L3 A* route, not a straight line from the model.
    let mut moved = false;
    let mut saw_route = false;
    let query = loop {
        let output = sim.tick();
        moved |= output.twist.linear > 0.;
        saw_route |= !output.status.global_route.is_empty();
        if output.status.effective_level.is_some() {
            assert_eq!(output.status.active_source, "task");
        }
        if let Some(q) = sim.arbiter.take_task_query() {
            break q;
        }
        assert!(
            sim.now < 120.,
            "never arrived: {:?} {:?}",
            sim.pose,
            output.status.reason
        );
    };
    assert!(moved && saw_route);
    assert!(
        (sim.pose.x - gx).hypot(sim.pose.y - gy) < 1.0,
        "{:?} goal ({gx}, {gy})\n{}",
        sim.pose,
        query.prompt
    );
    assert!(query.prompt.contains("reached O1"), "{}", query.prompt);

    sim.arbiter.submit_task_response(
        query.query_id,
        r#"{"decision":"done","rationale":"beside the wall"}"#.into(),
        None,
    );
    let output = sim.tick();
    let task = output.task.unwrap();
    assert_eq!(task.phase, TaskPhase::Completed);
    assert_eq!(task.reason, "model_done");
    assert_eq!(output.twist, Twist::default());
    assert_eq!(output.status.reason, "task_completed");
}

#[test]
fn motor_commands_and_unreachable_points_are_refused_and_fed_back() {
    let mut sim = Sim::new();
    sim.start("Go somewhere.");
    let q = sim.next_query(5);
    sim.arbiter.submit_task_response(
        q.query_id,
        r#"{"decision":"go","waypoints":[{"x":5,"y":6}],"linear":1.0,"angular":0.2}"#.into(),
        None,
    );
    let q = sim.next_query(5);
    assert!(q.prompt.contains("motor_fields_forbidden"), "{}", q.prompt);
    assert_eq!(sim.pose.x, 3.);

    // Inside the wall: no waypoint is latched.
    sim.arbiter.submit_task_response(
        q.query_id,
        r#"{"decision":"go","waypoints":[{"x":7.1,"y":6.0}]}"#.into(),
        None,
    );
    let q = sim.next_query(5);
    assert!(
        q.prompt.contains("not reachable seen free space"),
        "{}",
        q.prompt
    );

    // A third bad reply in a row fails the task.
    sim.arbiter
        .fail_task_query(q.query_id, "model unavailable".into());
    let output = sim.tick();
    let task = output.task.unwrap();
    assert_eq!(task.phase, TaskPhase::Failed);
    assert_eq!(task.reason, "model_failures");
    assert_eq!(output.twist, Twist::default());
}

#[test]
fn ask_operator_waits_for_a_reply_that_reaches_the_next_prompt() {
    let mut sim = Sim::new();
    sim.start("Find the red bucket.");
    let q = sim.next_query(5);
    sim.arbiter.submit_task_response(
        q.query_id,
        r#"{"decision":"ask_operator","question":"Which side of the wall?"}"#.into(),
        None,
    );
    for _ in 0..20 {
        let output = sim.tick();
        let task = output.task.unwrap();
        assert_eq!(task.phase, TaskPhase::WaitingOperator);
        assert_eq!(task.question.as_deref(), Some("Which side of the wall?"));
        assert_eq!(output.twist, Twist::default());
        assert!(sim.arbiter.take_task_query().is_none());
    }
    let wrong = TaskAction {
        task_id: "other".into(),
        action: TaskActionKind::Reply,
        text: Some("left".into()),
        token: "reply-0".into(),
        run_id: None,
    };
    assert_eq!(
        sim.arbiter.apply_task_action(wrong, sim.now),
        Err("wrong_task")
    );
    sim.arbiter
        .apply_task_action(
            TaskAction {
                task_id: "task-1".into(),
                action: TaskActionKind::Reply,
                text: Some("The far side, behind the wall.".into()),
                token: "reply-1".into(),
                run_id: None,
            },
            sim.now,
        )
        .unwrap();
    let q = sim.next_query(5);
    assert!(
        q.prompt
            .contains("operator: The far side, behind the wall."),
        "{}",
        q.prompt
    );
}

#[test]
fn stop_and_level_change_cancel_the_task() {
    let mut sim = Sim::new();
    sim.start("Explore.");
    let q = sim.next_query(5);
    sim.arbiter.submit_task_response(
        q.query_id,
        r#"{"decision":"go","waypoints":[{"target":"F1"}]}"#.into(),
        None,
    );
    for _ in 0..10 {
        sim.tick();
    }
    sim.arbiter.set_safety(
        SafetyRequest {
            action: SafetyAction::Stop,
            token: "stop".into(),
        },
        true,
    );
    let output = sim.tick();
    assert_eq!(output.task.as_ref().unwrap().phase, TaskPhase::Cancelled);
    assert_eq!(output.task.unwrap().reason, "operator_stop");
    assert_eq!(output.twist, Twist::default());

    let mut sim = Sim::new();
    sim.start("Explore.");
    sim.next_query(5);
    sim.arbiter
        .set_level(LevelRequest::new(Level::Teleop, "takeover"));
    let output = sim.tick();
    assert_eq!(output.task.unwrap().reason, "level_changed");
    assert!(sim.arbiter.take_task_query().is_none());
}

#[test]
fn unanswered_queries_time_out() {
    let mut sim = Sim::new();
    sim.start("Explore.");
    let first = sim.next_query(5);
    let mut second = None;
    for _ in 0..((TASK_QUERY_TIMEOUT_SECONDS / 0.1) as usize + 5) {
        sim.tick();
        if let Some(q) = sim.arbiter.take_task_query() {
            second = Some(q);
            break;
        }
    }
    let second = second.expect("a new query after the timeout");
    assert!(second.query_id > first.query_id);
    assert!(second.prompt.contains("timeout"));
    assert!(
        !sim.arbiter
            .submit_task_response(first.query_id, "{}".into(), None)
    );
}
