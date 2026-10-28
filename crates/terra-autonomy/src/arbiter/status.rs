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
            global_route: if matches!(self.level, Level::Waypoint | Level::Task)
                && decision.route_failure.is_none()
                && valid_time
                && !self.stopped
                && !self.paused
                && input.healthy
                && self.map_is_fresh(input)
                && decision.goal.state == GoalState::Active
            {
                self.global_route.clone()
            } else {
                Vec::new()
            },
            planner_trace: self
                .planner_trace
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
            active_source: decision.source.into(),
            safety: decision.safety.into(),
            reason: decision.reason.into(),
            revision: self.revision,
            token: self.receipt.as_ref().map(|r| r.0.clone()),
            result: self
                .receipt
                .as_ref()
                .map(|r| if r.1 { "accepted" } else { "rejected" }.into()),
            request_reason: self.request_reason.clone(),
            supported_levels: {
                let mut levels = vec![
                    Level::Teleop,
                    Level::AssistedTeleop,
                    Level::Waypoint,
                    Level::WaypointDirect,
                    Level::Supervised,
                    Level::Explore,
                ];
                if !self.detector_classes.is_empty()
                    && fresh(
                        input.now,
                        self.detector_time,
                        timing::DETECTOR_FRESHNESS_SECONDS,
                    )
                {
                    levels.push(Level::TargetSearch);
                }
                if self.task_model.is_some() || self.task_agent.is_some() {
                    levels.push(Level::Task);
                }
                levels
            },
            paused: self.paused,
            exploration: self.exploration_report(input.now),
            task: self.task_status(input.now),
        }
    }
}
