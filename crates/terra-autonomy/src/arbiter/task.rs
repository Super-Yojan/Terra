//! L5 task runs: ask the host's language model, validate its waypoints, drive them on the L3 path.
use super::*;
use crate::task::*;

/// One task under the `task` level.
pub(super) struct TaskRun {
    spec: TaskSpec,
    started_at: f64,
    ended_at: Option<f64>,
    phase: TaskPhase,
    reason: String,
    steps: u32,
    failures: u32,
    /// Built and waiting for the host to take it.
    pending: Option<TaskQuery>,
    /// Query id, creation time and the ids it named.
    outstanding: Option<(u64, f64, Vec<TaskTarget>)>,
    /// Host answer waiting for the next tick: query id, text or error, latency.
    answer: Option<(u64, Result<String, String>, Option<f64>)>,
    last_latency: Option<f64>,
    plan: VecDeque<TaskWaypoint>,
    waypoint_started: f64,
    blocked_since: Option<f64>,
    question: Option<String>,
    rationale: Option<String>,
    last_error: Option<String>,
    history: VecDeque<TaskHistoryEntry>,
}

impl TaskRun {
    fn new(spec: TaskSpec, now: f64) -> Self {
        Self {
            spec,
            started_at: now,
            ended_at: None,
            phase: TaskPhase::Planning,
            reason: "awaiting_model".into(),
            steps: 0,
            failures: 0,
            pending: None,
            outstanding: None,
            answer: None,
            last_latency: None,
            plan: VecDeque::new(),
            waypoint_started: now,
            blocked_since: None,
            question: None,
            rationale: None,
            last_error: None,
            history: VecDeque::new(),
        }
    }
    fn end(&mut self, phase: TaskPhase, reason: &str, now: f64) {
        if self.phase.terminal() {
            return;
        }
        self.phase = phase;
        self.reason = reason.into();
        self.ended_at = Some(now);
        self.pending = None;
        self.outstanding = None;
        self.answer = None;
        self.plan.clear();
    }
    fn replan(&mut self, reason: &str) {
        self.phase = TaskPhase::Planning;
        self.reason = reason.into();
        self.plan.clear();
        self.blocked_since = None;
        self.question = None;
    }
    fn note(&mut self, kind: &str, text: String) {
        push_history(&mut self.history, self.steps, kind, text);
    }
    /// Count a failed step and end the task after too many in a row.
    fn failed_step(&mut self, kind: &str, why: String, now: f64) {
        self.failures += 1;
        self.last_error = Some(why.clone());
        self.note(kind, why);
        if self.failures >= TASK_MAX_CONSECUTIVE_FAILURES {
            self.end(TaskPhase::Failed, "model_failures", now);
        } else {
            self.replan("retrying_model");
        }
    }
    fn status(&self, run_id: &str, model: Option<&str>, now: f64) -> TaskStatus {
        let elapsed = (self.ended_at.unwrap_or(now) - self.started_at).max(0.);
        TaskStatus {
            run_id: run_id.into(),
            task_id: self.spec.task_id.clone(),
            instruction: self.spec.instruction.clone(),
            phase: self.phase,
            reason: self.reason.clone(),
            step: self.steps,
            max_steps: self.spec.steps(),
            elapsed_seconds: elapsed,
            budget_seconds: self.spec.budget(),
            remaining_seconds: (self.spec.budget() - elapsed).max(0.),
            model: model.map(str::to_owned),
            query_id: self
                .outstanding
                .as_ref()
                .map(|q| q.0)
                .or(self.pending.as_ref().map(|q| q.query_id)),
            waiting_seconds: self.outstanding.as_ref().map(|q| (now - q.1).max(0.)),
            last_latency_seconds: self.last_latency,
            plan: self.plan.iter().cloned().collect(),
            question: self.question.clone(),
            rationale: self.rationale.clone(),
            last_error: self.last_error.clone(),
            history: self.history.iter().cloned().collect(),
        }
    }
}

impl AutonomyArbiter {
    /// Advertise (or withdraw with `None`) the host's language model. The `task`
    /// level is accepted only while a model is advertised.
    pub fn set_task_model(&mut self, name: Option<String>) {
        self.task_model = name
            .map(|n| {
                n.chars()
                    .filter(|c| !c.is_control())
                    .take(64)
                    .collect::<String>()
            })
            .filter(|n| !n.trim().is_empty());
    }
    pub fn task_model(&self) -> Option<&str> {
        self.task_model.as_deref()
    }
    /// Record a named point the host recognised, in local metres. The newest 32 are kept.
    pub fn observe_task_landmark(
        &mut self,
        label: &str,
        x: f64,
        y: f64,
        confidence: f64,
        now: f64,
    ) {
        let label: String = label
            .chars()
            .filter(|c| !c.is_control() && *c != '"')
            .take(40)
            .collect();
        if label.trim().is_empty()
            || !x.is_finite()
            || !y.is_finite()
            || !confidence.is_finite()
            || !now.is_finite()
        {
            return;
        }
        // One entry per label within a metre: refresh it rather than piling up repeats.
        self.task_landmarks
            .retain(|l| !(l.label == label && (l.x - x).hypot(l.y - y) < 1.));
        self.task_landmarks.push(TaskLandmark {
            label,
            x,
            y,
            confidence: confidence.clamp(0., 1.),
            seen_at: now,
        });
        if self.task_landmarks.len() > 32 {
            self.task_landmarks.remove(0);
        }
    }
    /// A new GPS destination arrived: ask the model again so its next plan sees G1.
    /// A query already sent is dropped and its late answer ignored. A question
    /// waiting on the operator is left alone.
    pub fn replan_task_for_destination(&mut self) {
        if self.level != Level::Task {
            return;
        }
        let Some(task) = self.task.as_mut() else {
            return;
        };
        if task.phase.terminal() || task.phase == TaskPhase::WaitingOperator {
            return;
        }
        task.pending = None;
        task.outstanding = None;
        task.answer = None;
        task.note("operator", "new GPS destination G1".into());
        task.replan("gps_destination");
        self.follower.cancel();
        self.last_plan = None;
    }
    /// Current or last task, if one has started since the level was selected.
    pub fn task_status(&self, now: f64) -> Option<TaskStatus> {
        self.task
            .as_ref()
            .map(|t| t.status(&self.run_id, self.task_model.as_deref(), now))
    }
    /// Start a task under the `task` level. A running task is replaced.
    pub fn start_task(&mut self, r: TaskRequest, now: f64) -> Result<(), &'static str> {
        let fp = format!("task:{}", serde_json::to_string(&r).unwrap_or_default());
        if !valid_token(&r.token) {
            return Err("invalid_task");
        }
        if self.duplicate(&r.token, &fp) {
            return if self.receipt.as_ref().is_some_and(|r| r.1) {
                Ok(())
            } else {
                Err("conflicting_or_rejected_token")
            };
        }
        let reason = if !r.spec.valid() {
            Some("invalid_task")
        } else if r.run_id.as_ref().is_some_and(|run| run != &self.run_id) {
            Some("wrong_run")
        } else if self.level != Level::Task || self.stopped {
            Some("authority_required")
        } else if self.task_model.is_none() {
            Some("task_model_unavailable")
        } else if !now.is_finite() || now < 0. {
            Some("invalid_time")
        } else {
            None
        };
        if let Some(reason) = reason {
            self.acknowledge(r.token, fp, false, reason);
            return Err(reason);
        }
        self.begin_task(r.spec, now);
        self.acknowledge(r.token.clone(), fp, true, "task_started");
        self.event("task_started", Some(r.token), "task_started");
        Ok(())
    }
    pub(super) fn begin_task(&mut self, spec: TaskSpec, now: f64) {
        if let Some(old) = self.task.as_mut() {
            old.end(TaskPhase::Cancelled, "replaced", now);
        }
        self.follower.cancel();
        self.last_plan = None;
        self.planner_trace = None;
        self.paused = false;
        self.task = Some(TaskRun::new(spec, now));
    }
    /// Cancel or reply to the running task.
    pub fn apply_task_action(&mut self, a: TaskAction, now: f64) -> Result<(), &'static str> {
        let fp = format!(
            "task_action:{}",
            serde_json::to_string(&a).unwrap_or_default()
        );
        if !valid_token(&a.token) {
            return Err("invalid_task_action");
        }
        if self.duplicate(&a.token, &fp) {
            return if self.receipt.as_ref().is_some_and(|r| r.1) {
                Ok(())
            } else {
                Err("conflicting_or_rejected_token")
            };
        }
        let reason = match self.task.as_ref() {
            _ if a.run_id.as_ref().is_some_and(|run| run != &self.run_id) => Some("wrong_run"),
            _ if self.level != Level::Task => Some("authority_required"),
            None => Some("no_task"),
            Some(t) if t.spec.task_id != a.task_id => Some("wrong_task"),
            Some(t) if t.phase.terminal() => Some("task_ended"),
            Some(_) if a.action == TaskActionKind::Reply && self.stopped => {
                Some("authority_required")
            }
            Some(_) => None,
        };
        if let Some(reason) = reason {
            self.acknowledge(a.token, fp, false, reason);
            return Err(reason);
        }
        match a.action {
            TaskActionKind::Cancel => self.end_task(TaskPhase::Cancelled, "operator_cancel", now),
            TaskActionKind::Reply => {
                let text = a.text.clone().unwrap_or_default();
                let task = self.task.as_mut().expect("checked above");
                task.note("operator", text);
                // An answer or new guidance replaces the current plan.
                task.replan("operator_reply");
                task.failures = 0;
                self.follower.cancel();
            }
        }
        self.acknowledge(a.token.clone(), fp, true, "task_action");
        self.event(
            "task_action",
            Some(a.token),
            &format!("{:?}", a.action).to_lowercase(),
        );
        Ok(())
    }
    pub(super) fn end_task(&mut self, phase: TaskPhase, reason: &str, now: f64) {
        let now = if now.is_finite() {
            now
        } else {
            self.last_time.unwrap_or(0.)
        };
        if let Some(task) = self.task.as_mut()
            && !task.phase.terminal()
        {
            task.end(phase, reason, now);
            self.follower.cancel();
            self.event("task_ended", None, reason);
        }
    }
    /// Hand the next model query to the host. Each query is returned once.
    pub fn take_task_query(&mut self) -> Option<TaskQuery> {
        self.task.as_mut()?.pending.take()
    }
    /// Return the model's text for `query_id`. It is validated on the next tick.
    /// Returns false for an unknown or already-answered query.
    pub fn submit_task_response(
        &mut self,
        query_id: u64,
        text: String,
        latency_seconds: Option<f64>,
    ) -> bool {
        self.store_answer(query_id, Ok(text), latency_seconds)
    }
    /// Report that the model could not answer `query_id`.
    pub fn fail_task_query(&mut self, query_id: u64, reason: String) -> bool {
        self.store_answer(query_id, Err(reason), None)
    }
    fn store_answer(
        &mut self,
        query_id: u64,
        answer: Result<String, String>,
        latency: Option<f64>,
    ) -> bool {
        let Some(task) = self.task.as_mut() else {
            return false;
        };
        if task.answer.is_some() || task.outstanding.as_ref().is_none_or(|q| q.0 != query_id) {
            return false;
        }
        // An unread pending copy of the same query is no longer needed.
        if task
            .pending
            .as_ref()
            .is_some_and(|q| q.query_id == query_id)
        {
            task.pending = None;
        }
        task.answer = Some((
            query_id,
            answer,
            latency.filter(|l| l.is_finite() && *l >= 0.),
        ));
        true
    }

    /// Advance the task before motion is chosen. Holds unless a waypoint is being driven.
    pub(super) fn drive_task(&mut self, input: &ArbiterInput<'_>, decision: &mut TickDecision) {
        let now = input.now;
        let radius = self.planner.config.radius;
        let pose = input.pose;
        let fresh_map = input.map.filter(|_| self.map_is_fresh(input));
        let Some(task) = self.task.as_mut() else {
            decision.held = true;
            decision.reason = "task_idle";
            return;
        };
        if task.phase.terminal() {
            decision.held = true;
            decision.reason = match task.phase {
                TaskPhase::Completed => "task_completed",
                TaskPhase::Failed => "task_failed",
                _ => "task_cancelled",
            };
            return;
        }
        let mut events: Vec<(&'static str, String)> = Vec::new();
        if now - task.started_at >= task.spec.budget() {
            task.end(TaskPhase::Failed, "budget_exhausted", now);
            events.push(("task_ended", "budget_exhausted".into()));
        }

        // Validate an answer that arrived since the last tick.
        if let Some((id, answer, latency)) = task.answer.take()
            && let Some((_, _, targets)) = task.outstanding.take().filter(|q| q.0 == id)
        {
            task.last_latency = latency;
            let map = fresh_map;
            match answer {
                Err(why) => task.failed_step("model_error", why, now),
                Ok(text) => match parse_decision(&text) {
                    Err(why) => task.failed_step("invalid", why, now),
                    Ok(ParsedDecision::Go {
                        waypoints,
                        rationale,
                    }) => match map {
                        None => task.failed_step(
                            "invalid",
                            "map went stale before the reply was checked".into(),
                            now,
                        ),
                        Some(map) => match resolve_plan(map, pose, radius, &targets, &waypoints) {
                            Err(why) => task.failed_step("invalid", why, now),
                            Ok((plan, note)) => {
                                task.failures = 0;
                                task.rationale = rationale;
                                task.last_error = note.clone();
                                let names: Vec<_> = plan
                                    .iter()
                                    .map(|w| format!("{} ({:.1}, {:.1})", w.label, w.x, w.y))
                                    .collect();
                                task.note(
                                    "decision",
                                    format!(
                                        "go {}{}",
                                        names.join(" then "),
                                        note.map(|n| format!("; {n}")).unwrap_or_default()
                                    ),
                                );
                                task.plan = plan.into();
                                task.phase = TaskPhase::Navigating;
                                task.reason = "driving_waypoint".into();
                                task.waypoint_started = now;
                                task.blocked_since = None;
                                events.push(("task_plan", names.join(";")));
                            }
                        },
                    },
                    Ok(ParsedDecision::AskOperator {
                        question,
                        rationale,
                    }) => {
                        task.failures = 0;
                        task.rationale = rationale;
                        task.note("decision", format!("asked the operator: {question}"));
                        task.question = Some(question);
                        task.phase = TaskPhase::WaitingOperator;
                        task.reason = "awaiting_operator".into();
                    }
                    Ok(ParsedDecision::Done { rationale }) => {
                        task.note(
                            "decision",
                            format!("done: {}", rationale.clone().unwrap_or_default()),
                        );
                        task.rationale = rationale;
                        task.end(TaskPhase::Completed, "model_done", now);
                        events.push(("task_ended", "model_done".into()));
                    }
                    Ok(ParsedDecision::Fail { rationale }) => {
                        task.note(
                            "decision",
                            format!("gave up: {}", rationale.clone().unwrap_or_default()),
                        );
                        task.rationale = rationale;
                        task.end(TaskPhase::Failed, "model_gave_up", now);
                        events.push(("task_ended", "model_gave_up".into()));
                    }
                },
            }
        }

        // A query nobody answered in time counts as a failed step.
        if task
            .outstanding
            .as_ref()
            .is_some_and(|q| now - q.1 >= TASK_QUERY_TIMEOUT_SECONDS)
        {
            task.outstanding = None;
            task.pending = None;
            task.failed_step(
                "timeout",
                format!("no answer in {TASK_QUERY_TIMEOUT_SECONDS:.0} s"),
                now,
            );
        }

        if task.phase == TaskPhase::Navigating {
            let status = self.follower.step(pose.x, pose.y, pose.yaw).status;
            let active = task.plan.front().cloned();
            match active {
                None => task.replan("plan_finished"),
                Some(w) => {
                    let latched = status.state != GoalState::Idle
                        && (status.x - w.x).abs() < 1e-9
                        && (status.y - w.y).abs() < 1e-9;
                    if latched && status.state == GoalState::Arrived {
                        task.note(
                            "arrived",
                            format!("reached {} ({:.1}, {:.1})", w.label, w.x, w.y),
                        );
                        task.plan.pop_front();
                        task.waypoint_started = now;
                        task.blocked_since = None;
                        self.follower.cancel();
                        if task.plan.is_empty() {
                            task.replan("plan_finished");
                        }
                    } else if now - task.waypoint_started >= TASK_WAYPOINT_TIMEOUT_SECONDS {
                        task.note(
                            "timeout",
                            format!(
                                "gave up on {} ({:.1}, {:.1}) after {TASK_WAYPOINT_TIMEOUT_SECONDS:.0} s",
                                w.label, w.x, w.y
                            ),
                        );
                        self.follower.cancel();
                        task.replan("waypoint_timeout");
                    }
                }
            }
            // Latch the active waypoint on the follower; the L3 route takes it from there.
            if task.phase == TaskPhase::Navigating
                && let Some(w) = task.plan.front()
            {
                let status = self.follower.step(pose.x, pose.y, pose.yaw).status;
                let latched = status.state != GoalState::Idle
                    && (status.x - w.x).abs() < 1e-9
                    && (status.y - w.y).abs() < 1e-9;
                if !latched {
                    let accepted = self
                        .follower
                        .accept(
                            &GoalCommand::Local {
                                x: w.x,
                                y: w.y,
                                yaw: None,
                                token: None,
                            },
                            20_000.,
                        )
                        .unwrap_or(false);
                    self.last_plan = None;
                    self.planner_trace = None;
                    if accepted {
                        task.waypoint_started = now;
                        events.push(("route_selected", "task_waypoint".into()));
                    } else {
                        task.note(
                            "blocked",
                            format!("follower refused {} ({:.1}, {:.1})", w.label, w.x, w.y),
                        );
                        task.replan("waypoint_refused");
                    }
                }
            }
        }

        if task.phase == TaskPhase::Planning && task.outstanding.is_none() {
            if task.steps >= task.spec.steps() {
                task.end(TaskPhase::Failed, "step_limit", now);
                events.push(("task_ended", "step_limit".into()));
            } else if let Some(map) = fresh_map {
                task.steps += 1;
                self.next_task_query += 1;
                let summary = summarize(
                    map,
                    pose,
                    radius,
                    &self.task_landmarks,
                    self.outdoor_destination,
                    now,
                );
                let elapsed = now - task.started_at;
                let prompt = step_prompt(
                    &task.spec.instruction,
                    task.steps,
                    task.spec.steps(),
                    elapsed,
                    task.spec.budget(),
                    pose,
                    radius,
                    &summary.text,
                    &task.history,
                );
                task.pending = Some(TaskQuery {
                    query_id: self.next_task_query,
                    task_id: task.spec.task_id.clone(),
                    system: task_system_prompt(),
                    prompt,
                });
                task.outstanding = Some((self.next_task_query, now, summary.targets));
                task.reason = "awaiting_model".into();
                self.follower.cancel();
            }
        }

        let phase = task.phase;
        let reason: &'static str = match phase {
            TaskPhase::Navigating => "",
            TaskPhase::Planning => "task_planning",
            TaskPhase::WaitingOperator => "task_waiting_operator",
            TaskPhase::Completed => "task_completed",
            TaskPhase::Failed => "task_failed",
            TaskPhase::Cancelled => "task_cancelled",
        };
        if phase != TaskPhase::Navigating {
            self.follower.cancel();
            decision.held = true;
            decision.reason = reason;
        }
        decision.goal = self.follower.step(pose.x, pose.y, pose.yaw).status;
        for (kind, reason) in events {
            self.event(kind, None, &reason);
        }
    }

    /// After motion is chosen: a route failure that persists abandons the waypoint.
    pub(super) fn watch_task_route(&mut self, input: &ArbiterInput<'_>, decision: &TickDecision) {
        let Some(task) = self.task.as_mut() else {
            return;
        };
        if task.phase != TaskPhase::Navigating {
            return;
        }
        let blocked = decision.route_failure.is_some()
            || matches!(
                decision.reason,
                "obstacle_blocked"
                    | "route_no_path"
                    | "route_goal_blocked"
                    | "heading_requires_spin_or_replan"
            );
        if !blocked {
            task.blocked_since = None;
            if task.reason != "driving_waypoint" {
                task.reason = "driving_waypoint".into();
            }
            return;
        }
        let since = *task.blocked_since.get_or_insert(input.now);
        task.reason = "route_blocked".into();
        if input.now - since >= TASK_BLOCKED_SECONDS {
            let why = decision.route_failure.unwrap_or(decision.reason);
            if let Some(w) = task.plan.front().cloned() {
                task.note(
                    "blocked",
                    format!(
                        "could not drive to {} ({:.1}, {:.1}): {why}",
                        w.label, w.x, w.y
                    ),
                );
            }
            task.replan("route_blocked");
            self.follower.cancel();
            self.last_plan = None;
            self.planner_trace = None;
            self.event("task_waypoint_abandoned", None, why);
        }
    }

    pub(super) fn task_running(&self) -> bool {
        self.task.as_ref().is_some_and(|t| !t.phase.terminal())
    }
}
