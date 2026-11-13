//! Validated, session-scoped planning geometry. Configuration never grants motor authority.
use serde::{Deserialize, Serialize};
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
/// Session-scoped vehicle geometry and avoidance limits; configuration grants no motion authority.
pub struct Parameters {
    /// Managed turn constraints; omitted by older clients.
    #[serde(default)]
    pub turning: terra_types::turning::TurningConfig,
    /// Planner reverse cap and penalty; defaults support older clients.
    #[serde(default = "default_reverse")]
    pub max_reverse: f64,
    #[serde(default = "default_reverse_penalty")]
    pub reverse_penalty: f64,
    /// Assumed body length, metres, validated in 0.1..=3 like a rover profile.
    pub length: f64,
    /// Assumed body width, metres, validated in 0.1..=3.
    pub width: f64,
    /// Safety margin kept on every side of the body rectangle, metres, validated in 0..=1.
    pub clearance: f64,
    /// Planner forward speed limit, m/s, validated in 0.05..=2.
    pub max_linear: f64,
    /// Planner yaw-rate limit, rad/s, validated in 0.1..=2.
    pub max_angular: f64,
    /// Planner rollout horizon, seconds, validated in 0.5..=5.
    pub horizon: f64,
    /// L4 Explore plans through unknown space; occupied cells still block. Off when omitted.
    #[serde(default)]
    pub explore_through_unknown: bool,
}
fn default_reverse() -> f64 {
    0.15
}
fn default_reverse_penalty() -> f64 {
    0.05
}
impl Default for Parameters {
    /// Terra's body, turning and speed limits from
    /// [`RoverProfile::terra`](crate::rover_profile::RoverProfile::terra).
    /// Operators must enter the actual vehicle dimensions for other rovers.
    fn default() -> Self {
        let terra = crate::rover_profile::RoverProfile::terra();
        Self {
            turning: terra_types::turning::TurningConfig {
                min_turn_radius: terra.min_turn_radius,
                allow_in_place: terra.allow_in_place,
                ..Default::default()
            },
            max_reverse: default_reverse(),
            reverse_penalty: default_reverse_penalty(),
            length: terra.length,
            width: terra.width,
            clearance: terra.clearance,
            max_linear: terra.max_linear,
            max_angular: terra.max_angular,
            horizon: 2.,
            explore_through_unknown: false,
        }
    }
}
impl Parameters {
    /// Return whether every geometry and avoidance parameter is finite and within its supported range.
    pub fn valid(&self) -> bool {
        use crate::rover_profile::{
            BODY_SIZE_M, CLEARANCE_M, MAX_ANGULAR_RADPS, MAX_LINEAR_MPS, within,
        };
        self.turning.validate().is_ok()
            && self.max_reverse.is_finite()
            && self.max_reverse > 0.
            && self.max_reverse <= 2.
            && self.reverse_penalty.is_finite()
            && (0.0..=10.).contains(&self.reverse_penalty)
            && [
                (self.length, BODY_SIZE_M),
                (self.width, BODY_SIZE_M),
                (self.clearance, CLEARANCE_M),
                (self.max_linear, MAX_LINEAR_MPS),
                (self.max_angular, MAX_ANGULAR_RADPS),
                (self.horizon, 0.5..=5.),
            ]
            .iter()
            .all(|(v, range)| within(*v, range))
    }
    /// The body rectangle the planners check, with `clearance` as the margin on every side.
    pub fn footprint(&self) -> terra_navigation::Footprint {
        terra_navigation::Footprint {
            length: self.length,
            width: self.width,
            margin: self.clearance,
        }
    }
    /// Build planner settings around the body rectangle.
    /// Other planner parameters retain portable defaults; this conversion assumes prior validation.
    /// Everything the autonomy arbiter takes from these parameters.
    pub fn arbiter_settings(&self) -> terra_autonomy::ArbiterSettings {
        terra_autonomy::ArbiterSettings {
            planner: self.planner(),
            turning: self.turning,
            explore_through_unknown: self.explore_through_unknown,
        }
    }
    pub fn planner(&self) -> terra_navigation::PlannerConfig {
        terra_navigation::PlannerConfig {
            footprint: self.footprint(),
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
pub struct State {
    /// Currently applied geometry and planner limits.
    pub parameters: Parameters,
    /// Settings revision within this controller session, bumped by
    /// [`Runtime::settings_changed`](crate::Runtime): settings and outdoor
    /// requests must name it.
    pub revision: u64,
    /// Most recently processed unique settings token, including rejected requests.
    pub token: Option<String>,
    /// Latest acceptance or rejection diagnostic.
    pub result: Option<String>,
    /// Rover profile that last set these parameters, if any.
    pub profile: Option<String>,
    seen: crate::replay::ReplayGuard<64>,
}
impl State {
    /// Return versioned navigation configuration status JSON, including the footprint, revision, and latest result.
    /// `footprint_radius` is the circle around the body, kept for older dashboards.
    pub fn status(&self) -> serde_json::Value {
        let footprint = self.parameters.footprint();
        serde_json::json!({"schema_version":1,"revision":self.revision,"parameters":self.parameters,"footprint":footprint,"footprint_radius":footprint.circumradius(),"profile":self.profile,"token":self.token,"result":self.result})
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
/// Apply one run/revision-bound settings request, already admitted for size
/// and age, only while live authority, goal, and hardware state permit
/// reconfiguration. Invalid envelopes are ignored; unique valid tokens record
/// an acceptance or rejection result and cannot be replayed.
pub(crate) fn apply(b: &mut super::Runtime, bytes: &[u8], now: f64) -> bool {
    let Ok(r) = serde_json::from_slice::<Request>(bytes) else {
        return false;
    };
    if !terra_autonomy::valid_token(&r.token) || !b.navigation.seen.admit(&r.token) {
        return false;
    }
    b.navigation.token = Some(r.token);
    // The last published tick must already show this authority, safety and goal.
    let tick = b.brief.as_ref();
    let pose = b.pose.map(|p| p.0).unwrap_or_default();
    let reason = if !r.parameters.valid() {
        Some("invalid dimensions or avoidance limits")
    } else if r.run_id != b.run_id
        || tick.is_none_or(|t| t.run_id != r.run_id || t.revision != r.authority_revision)
        || r.settings_revision != b.navigation.revision
    {
        Some("vehicle run or configuration revision changed")
    } else if !b
        .autonomy
        .navigation_configuration_allowed(&r.run_id, r.authority_revision, pose)
    {
        Some("live vehicle authority changed or a goal is active")
    } else if !tick.is_some_and(|t| {
        matches!(
            t.safety,
            terra_autonomy::Safety::Clear | terra_autonomy::Safety::Hold
        )
    }) {
        Some("vehicle safety state does not permit configuration")
    } else if !b.hardware.is_some_and(|h| h.fresh_and_idle(now)) {
        Some("fresh ready hardware with neutral output required")
    } else if tick.is_some_and(|t| t.goal.state == terra_waypoint::GoalState::Active) {
        Some("cancel the active goal before changing settings")
    } else if b.navigation.revision == u64::MAX {
        Some("configuration revision exhausted")
    } else {
        None
    };
    if let Some(reason) = reason {
        b.navigation.result = Some(format!("rejected: {reason}"));
        return false;
    }
    b.autonomy
        .configure(&r.parameters.arbiter_settings())
        .expect("validated turning settings");
    b.controller.reset();
    b.navigation.parameters = r.parameters;
    b.settings_changed();
    b.navigation.result = Some("accepted".into());
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{HardwareStatus, Runtime, RuntimeEvent};
    fn runtime() -> Runtime {
        Runtime::new(Default::default(), "settings".into()).unwrap()
    }
    fn hardware(r: &mut Runtime, output_active: bool) {
        r.ingest(RuntimeEvent::Hardware {
            status: HardwareStatus::from_json(
                &serde_json::json!({"ready":true,"output_active":output_active}),
            ),
            received_at: 0.0,
        })
        .unwrap();
    }
    fn send(r: &mut Runtime, request: &serde_json::Value) {
        apply(r, &serde_json::to_vec(request).unwrap(), 0.0);
    }
    #[test]
    /// Checks the dimensions become the planner's body rectangle (clearance as the
    /// margin on every side) and rejects negative clearance or nonfinite dimensions.
    fn navigation_dimensions_validate_and_derive_a_conservative_footprint() {
        let mut p = Parameters::default();
        p.length = 0.8;
        p.width = 0.6;
        p.clearance = 0.1;
        let footprint = p.planner().footprint;
        assert_eq!(
            (footprint.length, footprint.width, footprint.margin),
            (0.8, 0.6, 0.1)
        );
        assert!((footprint.circumradius() - 0.6).abs() < 1e-10);
        assert!(p.valid());
        p.clearance = -0.1;
        assert!(!p.valid());
        p.clearance = 0.1;
        p.length = f64::NAN;
        assert!(!p.valid());
    }
    #[test]
    /// Exercises absent hardware, replayed tokens, a successful geometry update,
    /// stale authority in a batch, and active-output rejection.
    fn navigation_update_requires_fresh_neutral_idle_hardware_and_current_revisions() {
        let mut r = runtime();
        r.step(0.0).unwrap();
        let payload = serde_json::json!({"token":"settings-one","run_id":r.run_id(),"authority_revision":0,"settings_revision":0,"parameters":Parameters::default()});
        send(&mut r, &payload);
        assert_eq!(r.navigation().revision, 0);
        hardware(&mut r, false);
        send(&mut r, &payload); // rejected tokens cannot be replayed
        assert_eq!(r.navigation().revision, 0);
        let mut p = payload.clone();
        p["token"] = "settings-two".into();
        p["parameters"]["length"] = 0.8.into();
        send(&mut r, &p);
        assert_eq!(r.navigation().revision, 1);
        assert_eq!(r.navigation().parameters.length, 0.8);
        assert_eq!(
            r.autonomy.footprint(),
            r.navigation().parameters.planner().footprint
        );
        let mut stale = payload.clone();
        stale["token"] = "second-in-batch".into();
        stale["settings_revision"] = 1.into();
        send(&mut r, &stale);
        assert_eq!(
            r.navigation().revision,
            1,
            "old authority cannot configure twice in one batch"
        );
        send(&mut r, &p);
        assert_eq!(r.navigation().revision, 1);
        let mut p = payload;
        p["token"] = "settings-three".into();
        p["settings_revision"] = 1.into();
        p["authority_revision"] = 1.into();
        hardware(&mut r, true);
        send(&mut r, &p);
        assert_eq!(r.navigation().revision, 1);
        assert!(
            r.navigation()
                .result
                .as_deref()
                .unwrap()
                .starts_with("rejected")
        );
    }
    #[test]
    /// Applies either a new goal or mode change before settings in the same tick
    /// and verifies live authority prevents configuration based on stale cached status.
    fn preceding_goal_or_mode_change_cannot_use_an_old_status_snapshot() {
        for change in ["goal", "mode"] {
            let mut r = runtime();
            let run = r.run_id().to_owned();
            r.autonomy.set_level(
                terra_autonomy::decode_level(br#"{"level":"waypoint_direct","token":"setup"}"#)
                    .unwrap(),
            );
            r.step(0.0).unwrap();
            hardware(&mut r, false);
            if change == "goal" {
                assert!(r.autonomy.accept_goal(
                    &terra_waypoint::decode_goal(br#"{"frame":"local","x":5,"y":0}"#).unwrap()
                ));
            } else {
                r.autonomy.set_level(
                    terra_autonomy::decode_level(br#"{"level":"teleop","token":"new-level"}"#)
                        .unwrap(),
                );
            }
            let request = serde_json::json!({"token":"mixed-settings","run_id":run,"authority_revision":1,"settings_revision":0,"parameters":Parameters::default()});
            send(&mut r, &request);
            assert_eq!(
                r.navigation().revision,
                0,
                "must reject after preceding {change}"
            );
        }
    }
}
