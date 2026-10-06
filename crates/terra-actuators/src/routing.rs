use crate::{ActuatorError, ActuatorValue, Layout, Route, SafeOutput, validate_structure};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RoutingInput {
    pub left_effort: f32,
    pub right_effort: f32,
    pub forward: f32,
    /// Positive yaw is a left turn: left coefficient negative, right positive.
    pub turn: f32,
    /// JSON object keys are decimal actuator IDs.
    #[serde(default)]
    pub servo_positions: BTreeMap<u8, f32>,
}

/// Return complete normalized commands. Hardware adapters apply inversion exactly once.
/// This function does not authorize arming or certify physical resource availability.
pub fn route_commands(layout: &Layout, input: &RoutingInput) -> Result<Vec<ActuatorValue>, ActuatorError> {
    validate_structure(layout).map_err(ActuatorError::InvalidLayout)?;
    if ![input.left_effort, input.right_effort, input.forward, input.turn].iter().all(|v| v.is_finite() && (-1.0..=1.0).contains(v)) {
        return Err(ActuatorError::InvalidInput("efforts and manual inputs must be finite and in [-1, 1]".into()));
    }
    for (id, value) in &input.servo_positions {
        if !value.is_finite() || !(-1.0..=1.0).contains(value) || !layout.actuators.iter().any(|a| a.id == *id && matches!(a.route, Route::Servo)) {
            return Err(ActuatorError::InvalidInput(format!("invalid servo target for ID {id}")));
        }
    }
    Ok(layout.actuators.iter().map(|a| {
        let value = match &a.route {
            Route::LeftEffort => input.left_effort,
            Route::RightEffort => input.right_effort,
            Route::Manual { forward_coefficient, turn_coefficient } => forward_coefficient * input.forward + turn_coefficient * input.turn,
            Route::Servo => input.servo_positions.get(&a.id).copied().unwrap_or_else(|| match a.safe {
                SafeOutput::Position { value } => value,
                // Disabled is a loss-of-link policy, not a command value. With no saved
                // safe position, use normalized center bounded by the configured limits.
                _ => 0.0,
            }),
        };
        ActuatorValue { id: a.id, value: value.clamp(a.limits.min, a.limits.max) }
    }).collect())
}
