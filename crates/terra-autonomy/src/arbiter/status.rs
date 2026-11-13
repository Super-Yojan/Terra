//! Build the telemetry snapshot without changing the selected command.
use super::*;

impl AutonomyArbiter {
    pub(super) fn status_for_tick(
        &self,
        input: &ArbiterInput<'_>,
        valid_time: bool,
        decision: &TickDecision,
    ) -> AutonomyStatus {
        AutonomyStatus {
            sensor_hold: None,
            global_route: if self.level.routed()
                && decision.route_failure.is_none()
                && valid_time
                && !self.stopped
                && !self.paused
                && input.healthy
                && self.map_is_fresh(input)
                && decision.goal.state == GoalState::Active
            {
                // About every 0.3 m, plus the end: enough to draw the route.
                let n = self.global_route.len();
                self.global_route
                    .iter()
                    .enumerate()
                    .skip(self.route_progress)
                    .filter(|(i, _)| i % 3 == 0 || *i + 1 == n)
                    .map(|(_, w)| [w.x, w.y])
                    .collect()
            } else {
                Vec::new()
            },
            planner_trace: self
                .plan
                .trace
                .as_ref()
                .filter(|(at, _)| {
                    input.now - at >= 0. && input.now - at < timing::PLANNER_TRACE_FRESHNESS_SECONDS
                })
                .map(|(at, trace)| {
                    let mut trace = trace.clone();
                    trace.age = input.now - at;
                    trace
                }),
            run_id: self.run_id.clone(),
            assigned_level: self.assigned_level,
            requested_level: self.level,
            effective_level: if decision.held {
                None
            } else {
                Some(self.level)
            },
            active_source: decision.source,
            safety: decision.safety,
            reason: decision.reason.as_str().into(),
            revision: self.revision,
            token: self.receipt.as_ref().map(|r| r.0.clone()),
            result: self
                .receipt
                .as_ref()
                .map(|r| if r.1 { "accepted" } else { "rejected" }.into()),
            request_reason: self.request_reason.clone(),
            supported_levels: Level::BASE
                .into_iter()
                .chain(
                    [Level::TargetSearch, Level::Task]
                        .into_iter()
                        .filter(|&l| self.level_unavailable(l, input.now).is_none()),
                )
                .collect(),
            task_agents: [
                (self.task_model.is_some(), crate::TaskAgent::Builtin),
                (self.task_agent.is_some(), crate::TaskAgent::Mcp),
            ]
            .into_iter()
            .filter_map(|(on, agent)| on.then_some(agent))
            .collect(),
            paused: self.paused,
            exploration: self.exploration_report(input.now),
            task: self.task_status(input.now),
            dock: self.dock_status(input.now),
        }
    }
}
