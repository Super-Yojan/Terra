//! Validated, session-scoped planning geometry. Configuration never grants motor authority.
use serde::{Deserialize, Serialize};
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct Parameters {
    pub length: f64,
    pub width: f64,
    pub clearance: f64,
    pub max_linear: f64,
    pub max_angular: f64,
    pub horizon: f64,
}
impl Default for Parameters {
    fn default() -> Self {
        // Preserve the existing 0.65 m planner radius. Geometry is a planning assumption,
        // not a hardware measurement; operators must enter the actual vehicle dimensions.
        Self {
            length: 0.85,
            width: 0.85,
            clearance: 0.65 - 0.85_f64.hypot(0.85) / 2.,
            max_linear: 2.,
            max_angular: 2.,
            horizon: 2.,
        }
    }
}
impl Parameters {
    pub fn valid(&self) -> bool {
        [
            (self.length, 0.1, 3.),
            (self.width, 0.1, 3.),
            (self.clearance, 0., 1.),
            (self.max_linear, 0.05, 2.),
            (self.max_angular, 0.1, 2.),
            (self.horizon, 0.5, 5.),
        ]
        .iter()
        .all(|&(v, min, max)| v.is_finite() && (min..=max).contains(&v))
    }
    pub fn planner(&self) -> terra_navigation::PlannerConfig {
        terra_navigation::PlannerConfig {
            radius: self.length.hypot(self.width) / 2. + self.clearance,
            max_linear: self.max_linear,
            max_angular: self.max_angular,
            horizon: self.horizon,
            ..Default::default()
        }
    }
}
#[derive(Default)]
pub(crate) struct State {
    pub parameters: Parameters,
    pub revision: u64,
    pub token: Option<String>,
    pub result: Option<String>,
    seen: std::collections::VecDeque<String>,
}
impl State {
    pub fn status(&self) -> serde_json::Value {
        serde_json::json!({"schema_version":1,"revision":self.revision,"parameters":self.parameters,"footprint_radius":self.parameters.planner().radius,"token":self.token,"result":self.result})
    }
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Request {
    token: String,
    run_id: String,
    authority_revision: u64,
    settings_revision: u64,
    parameters: Parameters,
}
pub(crate) fn apply(b: &mut super::Brain, bytes: &[u8], age: f64) {
    if bytes.len() > 2048 {
        return;
    }
    let Ok(r) = serde_json::from_slice::<Request>(bytes) else {
        return;
    };
    if !terra_autonomy::valid_token(&r.token) || b.navigation.seen.contains(&r.token) {
        return;
    }
    if b.navigation.seen.len() >= 64 {
        b.navigation.seen.pop_front();
    }
    b.navigation.seen.push_back(r.token.clone());
    b.navigation.token = Some(r.token);
    let status: serde_json::Value = serde_json::from_str(&b.autonomy_json).unwrap_or_default();
    let a = &status["status"];
    let pose = b.pose.map(|p| p.0).unwrap_or_default();
    let reason = if !r.parameters.valid() {
        Some("invalid dimensions or avoidance limits")
    } else if !age.is_finite() || !(0.0..0.5).contains(&age) {
        Some("settings request expired")
    } else if r.run_id != b.run_id
        || a["run_id"] != r.run_id
        || a["revision"] != r.authority_revision
        || r.settings_revision != b.navigation.revision
    {
        Some("vehicle run or configuration revision changed")
    } else if !b
        .autonomy
        .navigation_configuration_allowed(&r.run_id, r.authority_revision, pose)
    {
        Some("live vehicle authority changed or a goal is active")
    } else if !matches!(a["safety"].as_str(), Some("clear" | "hold")) {
        Some("vehicle safety state does not permit configuration")
    } else if !b.hardware_observation.as_ref().is_some_and(|(h, at)| {
        at.elapsed().as_secs_f64() < 0.5
            && h["ready"] == true
            && h["armed"] == false
            && h["arming"] == false
    }) {
        Some("fresh ready and disarmed hardware required")
    } else if !matches!(
        status["goal"]["state"].as_str(),
        Some("idle" | "arrived" | "cancelled")
    ) {
        Some("cancel the active goal before changing settings")
    } else if b.hardware_requests.iter().any(|(raw, _)| {
        serde_json::from_str::<serde_json::Value>(raw).is_ok_and(|r| r["action"] == "arm")
    }) {
        Some("an arming request is pending")
    } else if b.navigation.revision == u64::MAX {
        Some("configuration revision exhausted")
    } else {
        None
    };
    if let Some(reason) = reason {
        b.navigation.result = Some(format!("rejected: {reason}"));
        return;
    }
    b.autonomy.configure_planner(r.parameters.planner());
    b.controller.reset();
    b.navigation.parameters = r.parameters;
    b.navigation.revision += 1;
    b.navigation.result = Some("accepted".into());
}
