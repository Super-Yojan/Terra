//! L5 task intents from the phone: its language model and an MCP agent act
//! on the task through these calls, against the runtime's own pose and map.
use crate::*;
use terra_autonomy::{AgentOutcome, RequestedWaypoint, TaskQuery};

impl Runtime {
    /// Advertise the phone's language model by name, or withdraw it with `None`.
    pub fn set_task_model(&mut self, name: Option<String>) {
        self.autonomy.set_task_model(name);
    }
    /// Advertise an MCP agent by name, or withdraw it with `None`.
    pub fn set_task_agent(&mut self, name: Option<String>) {
        self.autonomy.set_task_agent(name);
    }
    /// The next model query, once.
    pub fn take_task_query(&mut self) -> Option<TaskQuery> {
        self.autonomy.take_task_query()
    }
    /// The model's raw reply to `query_id`; false for an unknown query.
    pub fn submit_task_response(
        &mut self,
        query_id: u64,
        text: String,
        latency_seconds: Option<f64>,
    ) -> bool {
        self.autonomy
            .submit_task_response(query_id, text, latency_seconds)
    }
    /// The model could not answer `query_id`; false for an unknown query.
    pub fn fail_task_query(&mut self, query_id: u64, reason: String) -> bool {
        self.autonomy.fail_task_query(query_id, reason)
    }
    /// A point the phone recognised, in local metres, that a task may target.
    pub fn observe_task_landmark(
        &mut self,
        label: &str,
        x: f64,
        y: f64,
        confidence: f64,
        now: f64,
    ) {
        self.autonomy
            .observe_task_landmark(label, x, y, confidence, now);
    }
    /// The agent's view of the map around the rover.
    pub fn agent_view(&mut self, now: f64) -> Result<String, &'static str> {
        let pose = self.pose.map(|p| p.0).unwrap_or_default();
        let map = self.map.as_ref().map(|(m, t, _)| (m, *t));
        self.autonomy.agent_view(map, pose, now)
    }
    /// Follow the agent's waypoints from the current pose.
    pub fn agent_go(
        &mut self,
        waypoints: &[RequestedWaypoint],
        rationale: Option<String>,
        now: f64,
    ) -> Result<AgentOutcome, String> {
        let pose = self.pose.map(|p| p.0).unwrap_or_default();
        let map = self.map.as_ref().map(|(m, t, _)| (m, *t));
        self.autonomy.agent_go(waypoints, rationale, map, pose, now)
    }
    pub fn agent_ask(&mut self, question: &str) -> Result<AgentOutcome, String> {
        self.autonomy.agent_ask(question)
    }
    pub fn agent_finish(
        &mut self,
        done: bool,
        rationale: &str,
        now: f64,
    ) -> Result<AgentOutcome, String> {
        self.autonomy.agent_finish(done, rationale, now)
    }
    pub fn agent_request_drive(
        &mut self,
        reason: &str,
        seconds: f64,
        now: f64,
    ) -> Result<AgentOutcome, String> {
        self.autonomy.agent_request_drive(reason, seconds, now)
    }
    pub fn agent_drive(
        &mut self,
        linear: f64,
        angular: f64,
        seconds: f64,
        now: f64,
    ) -> Result<AgentOutcome, String> {
        self.autonomy.agent_drive(linear, angular, seconds, now)
    }
}
