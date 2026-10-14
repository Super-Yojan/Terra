//! Decision JSON, actuator rejection, and the four #49 pass checks.
//!
//! Waypoint coordinates are copied from the object list. A waypoint is accepted
//! only at autonomy level `waypoint` (L3) and only when `astar_route` finds a path.

use serde::{Deserialize, Serialize};
use terra_navigation::{Pose, astar_route};

use crate::scene::{FOOTPRINT_RADIUS, Scenario, SceneObject, WAYPOINT_TOLERANCE_M};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DecisionKind {
    Waypoint,
    Hold,
    AskOperator,
}

impl DecisionKind {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Waypoint => "waypoint",
            Self::Hold => "hold",
            Self::AskOperator => "ask_operator",
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct WaypointCoord {
    pub x: f64,
    pub y: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ModelDecision {
    pub decision: DecisionKind,
    #[serde(default)]
    pub target_id: Option<String>,
    #[serde(default)]
    pub waypoint: Option<WaypointCoord>,
    pub dump_proposal: bool,
    pub rationale: String,
    pub confidence: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Score {
    pub valid_json: bool,
    pub no_actuator: bool,
    pub action_ok: bool,
    pub waypoint_ok: bool,
    pub notes: Vec<String>,
}

impl Score {
    pub fn passed(&self) -> bool {
        self.valid_json && self.no_actuator && self.action_ok && self.waypoint_ok
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ValidatedWaypoint {
    pub target_id: String,
    pub x: f64,
    pub y: f64,
}

const ACTUATOR_KEYS: &[&str] = &[
    "linear",
    "angular",
    "cmd_vel",
    "cmdvel",
    "twist",
    "effort",
    "pwm",
    "servo",
    "servos",
    "motor",
    "motors",
    "left_effort",
    "right_effort",
    "wheel",
    "wheels",
    "throttle",
    "steer",
    "steering",
    "dump_servo",
    "actuator",
    "actuators",
    "wheel_speed",
    "left_speed",
    "right_speed",
    "left_pwm",
    "right_pwm",
];

pub fn allows_model_waypoint(level: terra_autonomy::Level) -> bool {
    level == terra_autonomy::Level::Waypoint
}

pub fn score_output(raw: &str, scenario: &Scenario) -> Score {
    let mut notes = Vec::new();
    let Some(json_text) = extract_json_object(raw) else {
        return Score {
            valid_json: false,
            no_actuator: !raw_has_actuator_key(raw),
            action_ok: false,
            waypoint_ok: false,
            notes: vec!["no JSON object".to_string()],
        };
    };
    if json_text.trim() != raw.trim() {
        notes.push("JSON extracted from surrounding text".to_string());
    }
    let value: serde_json::Value = match serde_json::from_str(json_text) {
        Ok(value) => value,
        Err(err) => {
            notes.push(format!("JSON parse error: {err}"));
            return Score {
                valid_json: false,
                no_actuator: !raw_has_actuator_key(raw),
                action_ok: false,
                waypoint_ok: false,
                notes,
            };
        }
    };
    if let Some(key) = find_actuator_key(&value) {
        notes.push(format!("actuator field {key}"));
        return Score {
            valid_json: false,
            no_actuator: false,
            action_ok: false,
            waypoint_ok: false,
            notes,
        };
    }
    let decision: ModelDecision = match serde_json::from_value(value) {
        Ok(decision) => decision,
        Err(err) => {
            notes.push(format!("schema: {err}"));
            return Score {
                valid_json: false,
                no_actuator: true,
                action_ok: false,
                waypoint_ok: false,
                notes,
            };
        }
    };
    if let Some(reason) = schema_reason(&decision) {
        notes.push(reason);
        return Score {
            valid_json: false,
            no_actuator: true,
            action_ok: false,
            waypoint_ok: false,
            notes,
        };
    }
    let action_ok = scenario.expected.accept.contains(&decision.decision)
        && decision.dump_proposal == scenario.expected.dump_proposal;
    if !action_ok {
        notes.push(format!(
            "action {} dump={} not in the label",
            decision.decision.as_str(),
            decision.dump_proposal
        ));
    }
    let waypoint_ok = if decision.decision == DecisionKind::Waypoint {
        match validate_waypoint(&decision, scenario) {
            Ok(goal) => {
                let near = within_tolerance(
                    [goal.x, goal.y],
                    &scenario.expected.waypoints,
                    WAYPOINT_TOLERANCE_M,
                );
                if !near {
                    notes.push(
                        "list coordinate is farther than 0.5 m from the labelled point".to_string(),
                    );
                }
                if decision
                    .waypoint
                    .as_ref()
                    .is_some_and(|point| (point.x - goal.x).hypot(point.y - goal.y) > 0.05)
                {
                    notes.push(
                        "model waypoint ignored; coordinates came from the object list".to_string(),
                    );
                }
                near
            }
            Err(reason) => {
                notes.push(reason);
                false
            }
        }
    } else if scenario.expected.waypoint_required {
        notes.push("label requires a waypoint".to_string());
        false
    } else {
        true
    };
    if let Some(note) = rationale_note(&decision.rationale, scenario) {
        notes.push(note);
    }
    Score {
        valid_json: true,
        no_actuator: true,
        action_ok,
        waypoint_ok,
        notes,
    }
}

pub fn validate_waypoint(
    decision: &ModelDecision,
    scenario: &Scenario,
) -> Result<ValidatedWaypoint, String> {
    if !allows_model_waypoint(scenario.level) {
        return Err("waypoints are accepted only at autonomy level waypoint (L3)".to_string());
    }
    let Some(target_id) = decision
        .target_id
        .as_deref()
        .map(str::trim)
        .filter(|id| !id.is_empty())
    else {
        return Err("waypoint requires target_id from the object list".to_string());
    };
    let Some(object) = scenario
        .objects
        .iter()
        .find(|object| object.id == target_id)
    else {
        return Err(format!("target_id {target_id} is not in the object list"));
    };
    if !object.goal_eligible {
        return Err(format!("target {target_id} is not a goal"));
    }
    let pose = Pose {
        x: scenario.rover.x,
        y: scenario.rover.y,
        yaw: scenario.rover.yaw,
    };
    astar_route(
        &scenario.map,
        pose,
        FOOTPRINT_RADIUS,
        object.approach_x,
        object.approach_y,
    )
    .map_err(|err| format!("astar_route: {err}"))?;
    Ok(ValidatedWaypoint {
        target_id: object.id.clone(),
        x: object.approach_x,
        y: object.approach_y,
    })
}

pub fn within_tolerance(point: [f64; 2], labels: &[[f64; 2]], tolerance: f64) -> bool {
    labels
        .iter()
        .any(|label| (label[0] - point[0]).hypot(label[1] - point[1]) <= tolerance)
}

pub fn mock_decision(scenario: &Scenario) -> String {
    let decision = ModelDecision {
        decision: scenario.expected.decision,
        target_id: scenario.expected.target_id.clone(),
        waypoint: None,
        dump_proposal: scenario.expected.dump_proposal,
        rationale: format!("mock label for {}", scenario.id),
        confidence: 0.5,
    };
    serde_json::to_string(&decision).expect("mock decision serializes")
}

pub fn extract_json_object(text: &str) -> Option<&str> {
    let start = text.find('{')?;
    let bytes = text.as_bytes();
    let mut depth = 0;
    let mut in_string = false;
    let mut escape = false;
    for (offset, byte) in bytes.iter().enumerate().skip(start) {
        if in_string {
            if escape {
                escape = false;
            } else if *byte == b'\\' {
                escape = true;
            } else if *byte == b'"' {
                in_string = false;
            }
            continue;
        }
        match *byte {
            b'"' => in_string = true,
            b'{' => depth += 1,
            b'}' => {
                depth -= 1;
                if depth == 0 {
                    return Some(&text[start..=offset]);
                }
            }
            _ => {}
        }
    }
    None
}

fn schema_reason(decision: &ModelDecision) -> Option<String> {
    if !decision.confidence.is_finite() || !(0.0..=1.0).contains(&decision.confidence) {
        return Some("confidence must be finite and in 0..=1".to_string());
    }
    if decision.rationale.len() > 4_000 {
        return Some("rationale is longer than 4000 bytes".to_string());
    }
    if let Some(point) = &decision.waypoint {
        if !point.x.is_finite() || !point.y.is_finite() {
            return Some("waypoint is not finite".to_string());
        }
    }
    None
}

fn find_actuator_key(value: &serde_json::Value) -> Option<String> {
    match value {
        serde_json::Value::Object(map) => {
            for (key, child) in map {
                if ACTUATOR_KEYS
                    .iter()
                    .any(|name| key.eq_ignore_ascii_case(name))
                {
                    return Some(key.clone());
                }
                if let Some(found) = find_actuator_key(child) {
                    return Some(found);
                }
            }
            None
        }
        serde_json::Value::Array(items) => items.iter().find_map(find_actuator_key),
        _ => None,
    }
}

fn raw_has_actuator_key(raw: &str) -> bool {
    let lower = raw.to_ascii_lowercase();
    ACTUATOR_KEYS
        .iter()
        .any(|key| lower.contains(&format!("\"{key}\"")))
}

pub fn rationale_note(rationale: &str, scenario: &Scenario) -> Option<String> {
    let expected = scenario.expected.target_id.as_deref();
    let extras: Vec<&SceneObject> = scenario
        .objects
        .iter()
        .filter(|object| contains_id(rationale, &object.id) && Some(object.id.as_str()) != expected)
        .collect();
    if extras.is_empty() {
        None
    } else {
        let names: Vec<&str> = extras.iter().map(|object| object.id.as_str()).collect();
        Some(format!("rationale also names {}", names.join(", ")))
    }
}

fn contains_id(text: &str, id: &str) -> bool {
    let bytes = text.as_bytes();
    let needle = id.as_bytes();
    if needle.is_empty() || bytes.len() < needle.len() {
        return false;
    }
    bytes
        .windows(needle.len())
        .enumerate()
        .any(|(index, window)| {
            if window != needle {
                return false;
            }
            let before_ok = index == 0 || !bytes[index - 1].is_ascii_alphanumeric();
            let after = index + needle.len();
            let after_ok = after >= bytes.len() || !bytes[after].is_ascii_alphanumeric();
            before_ok && after_ok
        })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tolerance_is_half_a_metre() {
        assert!(within_tolerance([0.0, 0.0], &[[0.4, 0.0]], 0.5));
        assert!(!within_tolerance([0.0, 0.0], &[[0.6, 0.0]], 0.5));
    }

    #[test]
    fn extracts_fenced_json() {
        let text = "```json\n{\"decision\":\"hold\"}\n```";
        assert_eq!(extract_json_object(text), Some("{\"decision\":\"hold\"}"));
    }
}
