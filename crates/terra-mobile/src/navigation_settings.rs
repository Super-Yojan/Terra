//! Validated, session-scoped planning geometry. Configuration never grants motor authority.
use serde::{Deserialize, Serialize};
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
/// Session-scoped vehicle geometry and avoidance limits; configuration grants no motion authority.
pub(crate) struct Parameters {
    /// Managed turn constraints; omitted by older clients.
    #[serde(default)]
    pub turning: terra_control::TurningConfig,
    /// Planner reverse cap and penalty; defaults support older clients.
    #[serde(default = "default_reverse")]
    pub max_reverse: f64,
    #[serde(default = "default_reverse_penalty")]
    pub reverse_penalty: f64,
    /// Assumed body length, metres, validated in 0.1..=3.
    pub length: f64,
    /// Assumed body width, metres, validated in 0.1..=3.
    pub width: f64,
    /// Additional radial clearance, metres, validated in 0..=1.
    pub clearance: f64,
    /// Planner forward speed limit, m/s, validated in 0.05..=2.
    pub max_linear: f64,
    /// Planner yaw-rate limit, rad/s, validated in 0.1..=2.
    pub max_angular: f64,
    /// Planner rollout horizon, seconds, validated in 0.5..=5.
    pub horizon: f64,
}
fn default_reverse() -> f64 {
    0.15
}
fn default_reverse_penalty() -> f64 {
    0.05
}
impl Default for Parameters {
    fn default() -> Self {
        // Preserve the existing 0.65 m planner radius. Geometry is a planning assumption,
        // not a hardware measurement; operators must enter the actual vehicle dimensions.
        Self {
            turning: Default::default(),
            max_reverse: default_reverse(),
            reverse_penalty: default_reverse_penalty(),
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
    /// Return whether every geometry and avoidance parameter is finite and within its supported range.
    pub fn valid(&self) -> bool {
        self.turning.validate().is_ok()
            && self.max_reverse.is_finite()
            && self.max_reverse > 0.
            && self.max_reverse <= 2.
            && self.reverse_penalty.is_finite()
            && (0.0..=10.).contains(&self.reverse_penalty)
            && [
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
    /// Build planner settings with radius equal to half the body diagonal plus clearance.
    /// Other planner parameters retain portable defaults; this conversion assumes prior validation.
    pub fn planner(&self) -> terra_navigation::PlannerConfig {
        terra_navigation::PlannerConfig {
            radius: self.length.hypot(self.width) / 2. + self.clearance,
            max_reverse: self.max_reverse,
            reverse_penalty: self.reverse_penalty,
            max_linear: self.max_linear,
            max_angular: self.max_angular,
            horizon: self.horizon,
            ..Default::default()
        }
    }
}
#[derive(Default)]
/// Applied geometry revision, latest result, and bounded request replay history.
pub(crate) struct State {
    /// Currently applied geometry and planner limits.
    pub parameters: Parameters,
    /// Monotonic accepted-settings revision within this controller session.
    pub revision: u64,
    /// Most recently processed unique settings token, including rejected requests.
    pub token: Option<String>,
    /// Latest acceptance or rejection diagnostic.
    pub result: Option<String>,
    seen: std::collections::VecDeque<String>,
}
impl State {
    /// Return versioned navigation configuration status JSON, including radius, revision, and latest result.
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
/// Apply one bounded, fresh, run/revision-bound settings request only while live
/// authority, goal, and hardware state permit reconfiguration. `age` is receipt
/// age in seconds. Invalid envelopes are ignored; unique valid tokens record
/// an acceptance or rejection result and cannot be replayed.
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
            && h["output_active"] == false
    }) {
        Some("fresh ready hardware with neutral output required")
    } else if !matches!(
        status["goal"]["state"].as_str(),
        Some("idle" | "arrived" | "cancelled")
    ) {
        Some("cancel the active goal before changing settings")
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
    b.autonomy
        .configure_turning(r.parameters.turning)
        .expect("validated turning settings");
    b.controller.reset();
    b.navigation.parameters = r.parameters;
    b.navigation.revision += 1;
    b.outdoor.revision = b.outdoor.revision.saturating_add(1);
    b.navigation.result = Some("accepted".into());
}
