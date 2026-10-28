//! L5 task through an MCP agent. The agent calls tools instead of replying with JSON,
//! and each call goes through the same checks as a built-in model reply.
use super::*;
use crate::task::*;

/// What one agent tool call did.
#[derive(Clone, Debug, PartialEq)]
pub enum AgentOutcome {
    /// Waypoints accepted and now driving.
    Following(Vec<TaskWaypoint>),
    /// Question posted; the rover holds until the operator answers.
    Asked,
    /// Task ended with this reason.
    Ended(&'static str),
    /// Drive request posted for the operator.
    DriveRequested,
    /// Drive command accepted until the returned time.
    Driving(f64),
}

impl AutonomyArbiter {
    /// Advertise (or withdraw with `None`) an MCP agent. While one is attached,
    /// the `task` level is offered and tasks may ask for `agent: mcp`.
    pub fn set_task_agent(&mut self, name: Option<String>) {
        self.task_agent = name
            .map(|n| {
                n.chars()
                    .filter(|c| !c.is_control())
                    .take(64)
                    .collect::<String>()
            })
            .filter(|n| !n.trim().is_empty());
    }
    pub fn task_agent(&self) -> Option<&str> {
        self.task_agent.as_deref()
    }

    /// The task, map and history as the agent should see them, and the ids it may name.
    /// Readable by any agent; `map` is the latest snapshot and its receipt time.
    pub fn agent_view(
        &mut self,
        map: Option<(&MapSnapshot, f64)>,
        pose: Pose,
        now: f64,
    ) -> Result<String, &'static str> {
        let radius = self.planner.config.radius;
        let map = self.fresh_agent_map(map, now);
        let landmarks = self.task_landmarks.clone();
        let destination = self.outdoor_destination;
        let task = self.task.as_mut().ok_or("no_task")?;
        let summary = map.map(|m| summarize(m, pose, radius, &landmarks, destination, now));
        let map_text = summary
            .as_ref()
            .map(|s| s.text.as_str())
            .unwrap_or("MAP: no fresh map yet; wait and look again.\n");
        let status = task.status(&self.run_id, None, now);
        let mut text = step_prompt(
            &status.instruction,
            status.step,
            status.max_steps,
            status.elapsed_seconds,
            status.budget_seconds,
            pose,
            radius,
            map_text,
            &task.history,
        );
        // The built-in prompt ends by asking for JSON; agents act through tools instead.
        if let Some(cut) = text.rfind("\nReply with one JSON object only.") {
            text.truncate(cut);
        }
        text.push_str(&format!(
            "\nSTATE: {} ({}).",
            status.phase.as_str(),
            status.reason
        ));
        if let Some(q) = &status.question {
            text.push_str(&format!(" Waiting for the operator to answer: {q}"));
        }
        if let Some(left) = status.drive_remaining_seconds {
            text.push_str(&format!(" Raw drive approved for {left:.0} more s."));
        }
        if let Some(summary) = summary
            && task.agent() == TaskAgent::Mcp
        {
            task.agent_targets = summary.targets;
        }
        Ok(text)
    }

    /// Drive to 1 to 5 waypoints. Each is checked against seen free space and A*,
    /// exactly as a built-in `go` reply. A refusal changes nothing.
    pub fn agent_go(
        &mut self,
        waypoints: &[RequestedWaypoint],
        rationale: Option<String>,
        map: Option<(&MapSnapshot, f64)>,
        pose: Pose,
        now: f64,
    ) -> Result<AgentOutcome, String> {
        self.agent_control()?;
        if waypoints.is_empty() {
            return Err("give 1 to 5 waypoints".into());
        }
        if waypoints.len() > TASK_MAX_WAYPOINTS {
            return Err(format!("at most {TASK_MAX_WAYPOINTS} waypoints"));
        }
        let task = self.task.as_ref().expect("checked by agent_control");
        if task.phase() == TaskPhase::WaitingOperator {
            return Err("awaiting_operator: wait for the operator's answer".into());
        }
        if !task.steps_left() {
            return Err("step_limit: no decisions left; call finish".into());
        }
        let radius = self.planner.config.radius;
        let map = self
            .fresh_agent_map(map, now)
            .ok_or("map_stale: no fresh map; wait and call get_map")?;
        if task.agent_targets.is_empty() {
            self.agent_view(Some((map, now)), pose, now)
                .map_err(str::to_owned)?;
        }
        let task = self.task.as_mut().expect("checked by agent_control");
        match resolve_plan(map, pose, radius, &task.agent_targets, waypoints) {
            Err(why) => {
                task.note("invalid", why.clone());
                Err(why)
            }
            Ok((plan, note)) => {
                task.count_step();
                let names = task.follow(plan.clone(), note, rationale, now);
                self.follower.cancel();
                self.last_plan = None;
                self.planner_trace = None;
                self.event("task_plan", None, &names);
                Ok(AgentOutcome::Following(plan))
            }
        }
    }

    /// Ask the operator a question. The rover holds until the reply arrives.
    pub fn agent_ask(&mut self, question: &str) -> Result<AgentOutcome, String> {
        self.agent_control()?;
        let question = clean(question, 300).ok_or("question is empty")?;
        let task = self.task.as_mut().expect("checked by agent_control");
        task.ask(question, None);
        self.follower.cancel();
        self.event("task_question", None, "agent");
        Ok(AgentOutcome::Asked)
    }

    /// End the task as done or failed.
    pub fn agent_finish(
        &mut self,
        done: bool,
        rationale: &str,
        now: f64,
    ) -> Result<AgentOutcome, String> {
        self.agent_control()?;
        let task = self.task.as_mut().expect("checked by agent_control");
        let reason = task.conclude(done, clean(rationale, 300), now);
        self.follower.cancel();
        self.event("task_ended", None, reason);
        Ok(AgentOutcome::Ended(reason))
    }

    /// Ask the operator for raw drive for `seconds` (at most 60).
    pub fn agent_request_drive(
        &mut self,
        reason: &str,
        seconds: f64,
        now: f64,
    ) -> Result<AgentOutcome, String> {
        self.agent_control()?;
        if !seconds.is_finite() || seconds <= 0. {
            return Err("seconds must be positive".into());
        }
        let seconds = seconds.min(TASK_DRIVE_MAX_GRANT_SECONDS);
        let reason = clean(reason, 200).ok_or("give a reason the operator can judge")?;
        let task = self.task.as_mut().expect("checked by agent_control");
        if task.phase() == TaskPhase::WaitingOperator {
            return Err("awaiting_operator: wait for the operator's answer".into());
        }
        task.note(
            "drive",
            format!("asked to drive for {seconds:.0} s: {reason}"),
        );
        task.drive_request = Some((reason, seconds, now));
        self.event("task_drive_requested", None, "agent");
        Ok(AgentOutcome::DriveRequested)
    }

    /// Drive at `linear` m/s and `angular` rad/s for up to 1 s, only during an approved
    /// grant. Speeds are capped, and the local planner still checks every command.
    /// A waypoint plan in progress is dropped.
    pub fn agent_drive(
        &mut self,
        linear: f64,
        angular: f64,
        seconds: f64,
        now: f64,
    ) -> Result<AgentOutcome, String> {
        self.agent_control()?;
        if !linear.is_finite() || !angular.is_finite() || !seconds.is_finite() || seconds <= 0. {
            return Err("linear, angular and seconds must be finite, seconds positive".into());
        }
        let task = self.task.as_mut().expect("checked by agent_control");
        if task.phase() == TaskPhase::WaitingOperator {
            return Err("awaiting_operator: wait for the operator's answer".into());
        }
        if !task.drive_until.is_some_and(|until| now < until) {
            return Err("drive_not_approved: call request_drive and wait for the operator".into());
        }
        if task.phase() == TaskPhase::Navigating {
            task.note("drive", "dropped the waypoint plan to drive".into());
            task.replan("agent_drive");
            self.follower.cancel();
        }
        let until = now + seconds.min(TASK_DRIVE_COMMAND_SECONDS);
        task.agent_twist = Some((
            Twist {
                linear: linear.clamp(-TASK_DRIVE_MAX_LINEAR, TASK_DRIVE_MAX_LINEAR),
                angular: angular.clamp(-TASK_DRIVE_MAX_ANGULAR, TASK_DRIVE_MAX_ANGULAR),
            },
            until,
        ));
        Ok(AgentOutcome::Driving(until))
    }

    /// Write tools need the `task` level, no stop, a running task and `agent: mcp`.
    fn agent_control(&self) -> Result<(), String> {
        if self.level != Level::Task {
            return Err("not_in_control: the rover is not at the task level".into());
        }
        if self.stopped {
            return Err("not_in_control: the rover is stopped".into());
        }
        let task = self.task.as_ref().ok_or("not_in_control: no task")?;
        if task.phase().terminal() {
            return Err("task_ended".into());
        }
        if task.agent() != TaskAgent::Mcp {
            return Err("not_in_control: this task is run by the built-in model".into());
        }
        Ok(())
    }

    /// Current task id when an MCP agent is in charge of it.
    pub fn agent_task_id(&self) -> Option<&str> {
        self.task
            .as_ref()
            .filter(|t| t.agent() == TaskAgent::Mcp && !t.phase().terminal())
            .map(|t| t.task_id())
    }

    fn fresh_agent_map<'m>(
        &self,
        map: Option<(&'m MapSnapshot, f64)>,
        now: f64,
    ) -> Option<&'m MapSnapshot> {
        map.filter(|(_, at)| fresh(now, Some(*at), self.map_freshness_timeout_seconds))
            .map(|(m, _)| m)
    }
}

fn clean(text: &str, limit: usize) -> Option<String> {
    let text: String = text
        .trim()
        .chars()
        .map(|c| if c.is_control() { ' ' } else { c })
        .take(limit)
        .collect();
    (!text.trim().is_empty()).then_some(text)
}
