//! Convert normalized chassis intent into one value for each configured actuator.
//! Inversion is deliberately left to the hardware adapter so it is applied once.

use crate::{ActuatorError, ActuatorValue, Layout, Route, SafeOutput, validate_structure};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// Normalized chassis and servo intent. All supplied numbers must be finite and
/// in `[-1, 1]`; unknown servo IDs are rejected. Defaults are all zero with no overrides.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RoutingInput {
    /// Left wheel effort consumed by left-effort routes.
    pub left_effort: f32,
    /// Right wheel effort consumed by right-effort routes.
    pub right_effort: f32,
    /// Manual forward input consumed by manual mixing routes.
    pub forward: f32,
    /// Positive yaw is a left turn: left coefficient negative, right positive.
    pub turn: f32,
    /// JSON object keys are decimal actuator IDs.
    #[serde(default)]
    pub servo_positions: BTreeMap<u8, f32>,
}

fn check_input(input: &RoutingInput) -> Result<(), ActuatorError> {
    if ![
        input.left_effort,
        input.right_effort,
        input.forward,
        input.turn,
    ]
    .iter()
    .all(|v| v.is_finite() && (-1.0..=1.0).contains(v))
    {
        return Err(ActuatorError::InvalidInput(
            "efforts and manual inputs must be finite and in [-1, 1]".into(),
        ));
    }
    Ok(())
}

fn routed_value(route: &Route, safe: &SafeOutput, input: &RoutingInput, id: u8) -> f32 {
    match route {
        Route::LeftEffort => input.left_effort,
        Route::RightEffort => input.right_effort,
        Route::Manual {
            forward_coefficient,
            turn_coefficient,
        } => forward_coefficient * input.forward + turn_coefficient * input.turn,
        Route::Servo => input
            .servo_positions
            .get(&id)
            .copied()
            .unwrap_or_else(|| match safe {
                SafeOutput::Position { value } => *value,
                // Disabled is a loss-of-link policy, not a command value. With no saved
                // safe position, use normalized center bounded by the configured limits.
                _ => 0.0,
            }),
    }
}

/// Return complete normalized commands. Hardware adapters apply inversion exactly once.
/// This function does not authorize arming or certify physical resource availability.
/// Return values in layout order, clipping every routed value to its actuator limits.
///
/// Missing servo targets use their safe position, or normalized zero clipped to limits
/// for disabled servos. Returns [`ActuatorError::InvalidLayout`] for structural errors
/// and [`ActuatorError::InvalidInput`] for invalid chassis values or servo overrides.
pub fn route_commands(
    layout: &Layout,
    input: &RoutingInput,
) -> Result<Vec<ActuatorValue>, ActuatorError> {
    validate_structure(layout).map_err(ActuatorError::InvalidLayout)?;
    check_input(input)?;
    for (id, value) in &input.servo_positions {
        if !value.is_finite()
            || !(-1.0..=1.0).contains(value)
            || !layout
                .actuators
                .iter()
                .any(|actuator| actuator.id == *id && matches!(actuator.route, Route::Servo))
        {
            return Err(ActuatorError::InvalidInput(format!(
                "invalid servo target for ID {id}"
            )));
        }
    }
    Ok(layout
        .actuators
        .iter()
        .map(|actuator| ActuatorValue {
            id: actuator.id,
            value: routed_value(&actuator.route, &actuator.safe, input, actuator.id)
                .clamp(actuator.limits.min, actuator.limits.max),
        })
        .collect())
}

/// Runtime drive profile: the fields a rover returns from `read_drive_profile`.
///
/// Calibration and port assignment stay on the rover. Routing uses limits, route,
/// and the safe policy only.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DriveProfile {
    /// Actuator ID.
    pub id: u8,
    /// Output kind. Servo profiles must use a servo route.
    pub kind: crate::OutputKind,
    /// Inclusive normalized command bounds.
    pub limits: crate::CommandLimits,
    /// Command source before clipping.
    pub route: Route,
    /// Loss-of-link policy. Unused servo targets fall back to it.
    pub safe: SafeOutput,
}

fn invalid_profile(message: &str) -> ActuatorError {
    ActuatorError::InvalidInput(message.into())
}

/// Route normalized intent through drive profiles, in profile order.
///
/// The same mixing rules as [`route_commands`] apply. Structural layout fields
/// are not required. Returns [`ActuatorError::InvalidInput`] for an empty or
/// oversized profile list, duplicate IDs, inconsistent servo routes, or inputs
/// outside `[-1, 1]`.
pub fn route_drive_profiles(
    profiles: &[DriveProfile],
    input: &RoutingInput,
) -> Result<Vec<ActuatorValue>, ActuatorError> {
    if profiles.is_empty() || profiles.len() > 16 {
        return Err(invalid_profile("expected one to sixteen drive profiles"));
    }
    check_input(input)?;
    let mut ids = BTreeMap::new();
    for profile in profiles {
        if ids.insert(profile.id, ()).is_some() {
            return Err(invalid_profile("actuator IDs must be unique"));
        }
        let servo = profile.kind == crate::OutputKind::PositionalServo;
        if servo != matches!(profile.route, Route::Servo) {
            return Err(invalid_profile(
                "servo routes require positional servos; propulsion routes require propulsion outputs",
            ));
        }
        let lower = if profile.kind == crate::OutputKind::UnidirectionalEsc {
            0.0
        } else {
            -1.0
        };
        if !profile.limits.min.is_finite()
            || !profile.limits.max.is_finite()
            || profile.limits.min > profile.limits.max
            || profile.limits.min < lower
            || profile.limits.max > 1.0
        {
            return Err(invalid_profile(
                "profile limits must be finite, ordered, and in range",
            ));
        }
        if let Route::Manual {
            forward_coefficient,
            turn_coefficient,
        } = &profile.route
        {
            if ![forward_coefficient, turn_coefficient]
                .iter()
                .all(|v| v.is_finite() && (-1.0..=1.0).contains(*v))
            {
                return Err(invalid_profile(
                    "manual coefficients must be finite and in [-1, 1]",
                ));
            }
        }
        match &profile.safe {
            SafeOutput::Zero => {
                if servo || profile.limits.min > 0.0 || profile.limits.max < 0.0 {
                    return Err(invalid_profile(
                        "propulsion safe output is zero and zero must lie within limits",
                    ));
                }
            }
            SafeOutput::Position { value } => {
                if !servo
                    || !value.is_finite()
                    || *value < profile.limits.min
                    || *value > profile.limits.max
                {
                    return Err(invalid_profile(
                        "safe position must lie within servo limits",
                    ));
                }
            }
            SafeOutput::Disabled if !servo => {
                return Err(invalid_profile("only a servo can disable its safe output"));
            }
            SafeOutput::Disabled => {}
        }
    }
    for (id, value) in &input.servo_positions {
        if !value.is_finite()
            || !(-1.0..=1.0).contains(value)
            || !profiles
                .iter()
                .any(|profile| profile.id == *id && matches!(profile.route, Route::Servo))
        {
            return Err(invalid_profile(&format!(
                "invalid servo target for ID {id}"
            )));
        }
    }
    Ok(profiles
        .iter()
        .map(|profile| ActuatorValue {
            id: profile.id,
            value: routed_value(&profile.route, &profile.safe, input, profile.id)
                .clamp(profile.limits.min, profile.limits.max),
        })
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{CommandLimits, OutputKind};
    use std::collections::BTreeMap;

    fn motor(id: u8, route: Route) -> DriveProfile {
        DriveProfile {
            id,
            kind: OutputKind::DcMotor,
            limits: CommandLimits {
                min: -1.0,
                max: 1.0,
            },
            route,
            safe: SafeOutput::Zero,
        }
    }

    #[test]
    fn positive_turn_slows_the_left_profile() {
        let profiles = vec![motor(0, Route::LeftEffort), motor(1, Route::RightEffort)];
        let values = route_drive_profiles(
            &profiles,
            &RoutingInput {
                left_effort: -0.3,
                right_effort: 0.7,
                forward: 0.2,
                turn: 0.5,
                servo_positions: BTreeMap::new(),
            },
        )
        .unwrap();
        assert_eq!(values[0].value, -0.3);
        assert_eq!(values[1].value, 0.7);
    }

    #[test]
    fn servo_without_a_target_uses_its_safe_position() {
        let profiles = vec![DriveProfile {
            id: 4,
            kind: OutputKind::PositionalServo,
            limits: CommandLimits {
                min: -1.0,
                max: 1.0,
            },
            route: Route::Servo,
            safe: SafeOutput::Position { value: 0.25 },
        }];
        let values = route_drive_profiles(&profiles, &RoutingInput::default()).unwrap();
        assert_eq!(values[0].value, 0.25);
    }
}
