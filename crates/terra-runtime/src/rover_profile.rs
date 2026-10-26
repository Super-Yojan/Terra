//! Per-rover body and drive model. One profile sets the planner footprint,
//! managed turning policy and the velocity loop's gains together, so a small
//! rover is neither planned as a large one nor driven with a large one's gains.
use serde::{Deserialize, Serialize};
use terra_control::ControllerConfig;

/// Body size, speed limits and motor response of one rover.
///
/// Feedforward follows from `top_speed`: full effort past the deadband is
/// assumed to reach it. Feedback gains scale with that feedforward, so every
/// profile keeps the loop dynamics tuned on Terra. Yaw feedforward is
/// `skid_factor · track_width / 2 / top_speed`; skid-steer chassis scrub, so
/// the factor is above 1.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RoverProfile {
    /// Short identifier, for example `terra_mini`.
    pub name: String,
    /// Body length and width, metres, 0.1..=3.
    pub length: f64,
    pub width: f64,
    /// Wheel centre to centre across the axle, metres, 0.05..=3.
    pub track_width: f64,
    /// Planner clearance added to half the body diagonal, metres, 0..=1.
    pub clearance: f64,
    /// Planner and controller forward speed limit, m/s, 0.05..=2.
    pub max_linear: f64,
    /// Planner and controller yaw-rate limit, rad/s, 0.1..=2.
    pub max_angular: f64,
    /// Managed minimum turn radius while moving, metres, (0, 5].
    pub min_turn_radius: f64,
    /// Whether autonomy may spin in place.
    pub allow_in_place: bool,
    /// Ground speed at full wheel effort, m/s, 0.1..=10.
    pub top_speed: f64,
    /// Yaw effort multiplier for wheel scrub, 1..=5.
    pub skid_factor: f64,
    /// Wheel effort below which the motors stall, 0..=0.9.
    pub wheel_deadband: f64,
    /// Open-loop wheel effort for a managed in-place spin, 0..=1.
    pub spin_effort: f64,
}

impl RoverProfile {
    /// Terra (CrabTrapBot, two ESCs). Reproduces the runtime defaults exactly.
    pub fn terra() -> Self {
        Self {
            name: "terra".into(),
            length: 0.85,
            width: 0.85,
            track_width: 0.5,
            clearance: 0.65 - 0.85_f64.hypot(0.85) / 2.,
            max_linear: 2.,
            max_angular: 2.,
            min_turn_radius: 0.5,
            allow_in_place: false,
            top_speed: 3.,
            skid_factor: 2.4,
            wheel_deadband: 0.,
            spin_effort: 0.35,
        }
    }
    /// terra-mini (four DC motors on the Fusion HAT): 7 in (0.178 m) square
    /// over the wheels, 1 in wheels, so a 6 in (0.152 m) track. Speed,
    /// deadband and spin effort are estimates until measured on the rover.
    pub fn terra_mini() -> Self {
        Self {
            name: "terra_mini".into(),
            length: 0.178,
            width: 0.178,
            track_width: 0.152,
            clearance: 0.1,
            max_linear: 0.4,
            max_angular: 1.5,
            min_turn_radius: 0.15,
            allow_in_place: true,
            top_speed: 0.8,
            skid_factor: 2.,
            wheel_deadband: 0.3,
            spin_effort: 0.6,
        }
    }
    pub fn presets() -> Vec<Self> {
        vec![Self::terra(), Self::terra_mini()]
    }
    pub fn preset(name: &str) -> Option<Self> {
        Self::presets().into_iter().find(|p| p.name == name)
    }
    pub fn valid(&self) -> bool {
        !self.name.is_empty()
            && self.name.len() <= 64
            && [
                (self.length, 0.1, 3.),
                (self.width, 0.1, 3.),
                (self.track_width, 0.05, 3.),
                (self.clearance, 0., 1.),
                (self.max_linear, 0.05, 2.),
                (self.max_angular, 0.1, 2.),
                (self.top_speed, 0.1, 10.),
                (self.skid_factor, 1., 5.),
                (self.wheel_deadband, 0., 0.9),
                (self.spin_effort, 0., 1.),
            ]
            .iter()
            .all(|&(v, min, max)| v.is_finite() && (min..=max).contains(&v))
            && self.min_turn_radius.is_finite()
            && self.min_turn_radius > 0.
            && self.min_turn_radius <= 5.
            && (self.spin_effort == 0. || self.spin_effort > self.wheel_deadband)
    }
    /// Planner geometry and turning from this profile; horizon and reverse
    /// settings stay as in `base`.
    pub fn navigation(
        &self,
        base: &crate::navigation::Parameters,
    ) -> crate::navigation::Parameters {
        crate::navigation::Parameters {
            turning: terra_control::TurningConfig {
                min_turn_radius: self.min_turn_radius,
                allow_in_place: self.allow_in_place,
                ..base.turning
            },
            length: self.length,
            width: self.width,
            clearance: self.clearance,
            max_linear: self.max_linear,
            max_angular: self.max_angular,
            ..base.clone()
        }
    }
    /// Velocity-loop settings from this profile; timing settings stay as in `base`.
    pub fn controller(&self, base: ControllerConfig) -> ControllerConfig {
        let linear_feedforward = 1. / self.top_speed;
        let yaw_feedforward = self.skid_factor * self.track_width / 2. / self.top_speed;
        ControllerConfig {
            linear_kp: 0.9 * linear_feedforward,
            linear_ki: 1.8 * linear_feedforward,
            yaw_kp: 0.5 * yaw_feedforward,
            yaw_ki: 1.5 * yaw_feedforward,
            linear_feedforward,
            yaw_feedforward,
            max_forward: self.max_linear,
            max_yaw_rate: self.max_angular,
            spin_effort: self.spin_effort,
            wheel_deadband: self.wheel_deadband,
            ..base
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    /// Selecting Terra changes nothing: same planner, turning and gains as before profiles.
    #[test]
    fn terra_profile_matches_runtime_defaults() {
        let terra = RoverProfile::terra();
        assert!(terra.valid());
        let base = crate::navigation::Parameters::default();
        assert_eq!(terra.navigation(&base), base);
        let c = terra.controller(ControllerConfig::default());
        let d = ControllerConfig::default();
        for (a, b) in [
            (c.linear_kp, d.linear_kp),
            (c.linear_ki, d.linear_ki),
            (c.yaw_kp, d.yaw_kp),
            (c.yaw_ki, d.yaw_ki),
            (c.linear_feedforward, d.linear_feedforward),
            (c.yaw_feedforward, d.yaw_feedforward),
            (c.max_forward, d.max_forward),
            (c.max_yaw_rate, d.max_yaw_rate),
            (c.spin_effort, d.spin_effort),
            (c.wheel_deadband, d.wheel_deadband),
        ] {
            assert!(close(a, b), "{a} != {b}");
        }
    }

    /// terra-mini plans with its own small footprint and a valid controller.
    #[test]
    fn terra_mini_profile_is_small_and_valid() {
        let mini = RoverProfile::preset("terra_mini").unwrap();
        assert!(mini.valid());
        let nav = mini.navigation(&Default::default());
        assert!(nav.valid());
        assert!(
            nav.planner().radius < 0.3,
            "radius={}",
            nav.planner().radius
        );
        assert!(
            mini.controller(ControllerConfig::default())
                .validate()
                .is_ok()
        );
        assert!(RoverProfile::preset("unknown").is_none());
        let mut bad = mini.clone();
        bad.spin_effort = 0.2;
        assert!(!bad.valid(), "spin below the deadband would stall");
        bad = mini;
        bad.top_speed = f64::NAN;
        assert!(!bad.valid());
    }
}
