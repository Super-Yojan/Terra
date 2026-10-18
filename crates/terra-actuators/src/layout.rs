//! Actuator assignments, calibration, safe outputs, and validation.
//! Validation separates schema correctness from backend port/resource compatibility.

use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

/// Versioned hardware assignment document; validate before routing or installing.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Layout {
    /// Schema format version; currently must equal 1.
    pub schema_version: u8,
    /// Application-assigned revision used to reject commands for another layout.
    pub revision: u32,
    /// Ordered actuator assignments; at most 16, with unique IDs.
    pub actuators: Vec<Actuator>,
}
/// One named physical output and its routing, calibration, and loss-of-link policy.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Actuator {
    /// Unique command identifier within this layout.
    pub id: u8,
    /// Human-readable label; structural validation imposes no content restriction.
    pub name: String,
    /// Nonempty backend port identifier, matched exactly against capabilities.
    pub port: String,
    /// Physical output technology determining valid calibration and command range.
    pub kind: OutputKind,
    /// Whether the hardware adapter reverses normalized polarity; forbidden for unidirectional ESCs.
    pub inverted: bool,
    /// Normalized clipping bounds applied after routing.
    pub limits: CommandLimits,
    /// Backend conversion settings matching the output kind.
    pub calibration: Calibration,
    /// Source of the normalized command.
    pub route: Route,
    /// Output policy the hardware adapter should apply on loss of authorization.
    pub safe: SafeOutput,
}
/// Supported hardware output technologies, serialized as snake-case names.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OutputKind {
    /// Signed PWM motor power.
    DcMotor,
    /// Signed ESC power with a neutral pulse.
    BidirectionalEsc,
    /// Nonnegative ESC power with a stop pulse.
    UnidirectionalEsc,
    /// Normalized positional servo command.
    PositionalServo,
}
/// Inclusive normalized command bounds, finite and ordered.
///
/// Both bounds must lie in `[-1, 1]`, or `[0, 1]` for unidirectional ESCs.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CommandLimits {
    /// Smallest permitted normalized command.
    pub min: f32,
    /// Largest permitted normalized command.
    pub max: f32,
}
/// Physical conversion settings; pulse widths are microseconds and must be
/// positive, strictly ordered, and below 20,000 at their upper endpoint.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum Calibration {
    /// PWM power scaling for a DC motor.
    DcMotor {
        /// Finite fraction of available power in `(0, 1]`.
        max_power_fraction: f32,
    },
    /// Reverse, neutral, and forward endpoints for a reversible ESC.
    BidirectionalEsc {
        /// Full reverse pulse width in microseconds.
        reverse_us: u32,
        /// Neutral pulse width in microseconds.
        neutral_us: u32,
        /// Full forward pulse width in microseconds.
        forward_us: u32,
        /// Backend-required arming interval in milliseconds; structural validation imposes no bound.
        arming_duration_ms: u32,
    },
    /// Stop and full-power endpoints for a one-direction ESC.
    UnidirectionalEsc {
        /// Stopped pulse width in microseconds.
        stop_us: u32,
        /// Full-power pulse width in microseconds.
        full_power_us: u32,
        /// Backend-required arming interval in milliseconds; structural validation imposes no bound.
        arming_duration_ms: u32,
    },
    /// Negative, center, and positive positional endpoints.
    PositionalServo {
        /// Pulse width at normalized position -1, in microseconds.
        min_us: u32,
        /// Pulse width at normalized position 0, in microseconds.
        center_us: u32,
        /// Pulse width at normalized position +1, in microseconds.
        max_us: u32,
    },
}
/// Source of actuator intent before clipping; inversion is applied by hardware.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum Route {
    /// Use the normalized left wheel effort.
    LeftEffort,
    /// Use the normalized right wheel effort.
    RightEffort,
    /// Mix manual forward and turn inputs linearly, then clip to command limits.
    Manual {
        /// Finite forward-input multiplier in `[-1, 1]`.
        forward_coefficient: f32,
        /// Finite turn-input multiplier in `[-1, 1]`.
        turn_coefficient: f32,
    },
    /// Use the actuator-specific servo position or the configured fallback.
    Servo,
}
/// Loss-of-link policy: propulsion requires zero; servos may hold a position or disable.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum SafeOutput {
    /// Command zero propulsion effort; zero must be inside the configured limits.
    Zero,
    /// Hold a finite servo position within its limits and `[-1, 1]`.
    Position {
        /// Normalized servo position, finite and within the configured limits.
        value: f32,
    },
    /// Disable servo output rather than specifying a position.
    Disabled,
}

/// Backend supplied ownership and timer constraints; names are opaque identities.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PortCapability {
    /// Output kinds that this port can generate.
    pub kinds: Vec<OutputKind>,
    /// Nonempty unique physical resource identifiers consumed by this port.
    pub resources: Vec<String>,
    /// Shared timer identity, if any; users of one timer must agree on frequency.
    pub timer: Option<String>,
    /// PWM frequency in hertz; positive for motors and exactly 50 for ESCs/servos.
    pub frequency_hz: u32,
}
/// Backend-advertised board, library, ports, and occupied physical resources.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Capabilities {
    /// Board identity supplied by the backend.
    pub board: String,
    /// Hardware driver/library name.
    pub library: String,
    /// Hardware driver/library version string.
    pub library_version: String,
    /// Output kinds implemented by this backend.
    pub supported_kinds: Vec<OutputKind>,
    /// Port identifiers and their output/resource constraints.
    pub ports: BTreeMap<String, PortCapability>,
    /// Resources already unavailable before this layout is assigned.
    pub occupied_resources: BTreeSet<String>,
}
/// One collected validation failure with a stable category and human-readable detail.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LayoutError {
    /// Affected actuator, or `None` for a document-wide failure.
    pub actuator_id: Option<u8>,
    /// Machine-readable validation category such as `calibration` or `resource_conflict`.
    pub code: String,
    /// Explanation of the violated constraint.
    pub message: String,
}

/// Structural validation is also used before routing, without claiming hardware availability.
/// Collect all schema, identity, range, calibration, routing, and safe-policy errors.
///
/// Returns `Ok(())` for a structurally valid document, otherwise all discovered
/// errors. This check does not consult hardware capabilities or detect port conflicts.
pub fn validate_structure(layout: &Layout) -> Result<(), Vec<LayoutError>> {
    let mut errors = Vec::new();
    let mut add = |id, code: &str, message: &str| {
        errors.push(LayoutError {
            actuator_id: id,
            code: code.into(),
            message: message.into(),
        })
    };
    if layout.schema_version != 1 {
        add(None, "schema_version", "schema version must be 1");
    }
    if layout.actuators.len() > 16 {
        add(None, "actuator_count", "at most 16 actuators are supported");
    }
    let mut ids = BTreeSet::new();
    for a in &layout.actuators {
        if !ids.insert(a.id) {
            add(Some(a.id), "duplicate_id", "actuator IDs must be unique");
        }
        if a.port.trim().is_empty() {
            add(Some(a.id), "port", "port must not be empty");
        }
        let lower = if a.kind == OutputKind::UnidirectionalEsc {
            0.0
        } else {
            -1.0
        };
        if !a.limits.min.is_finite()
            || !a.limits.max.is_finite()
            || a.limits.min < lower
            || a.limits.max > 1.0
            || a.limits.min > a.limits.max
        {
            add(
                Some(a.id),
                "command_limits",
                "limits must be finite, ordered, and within the kind's normalized range",
            );
        }
        let calibration_ok = match (&a.kind, &a.calibration) {
            (OutputKind::DcMotor, Calibration::DcMotor { max_power_fraction }) => {
                max_power_fraction.is_finite()
                    && *max_power_fraction > 0.0
                    && *max_power_fraction <= 1.0
            }
            (
                OutputKind::BidirectionalEsc,
                Calibration::BidirectionalEsc {
                    reverse_us,
                    neutral_us,
                    forward_us,
                    ..
                },
            ) => {
                *reverse_us > 0
                    && reverse_us < neutral_us
                    && neutral_us < forward_us
                    && *forward_us < 20_000
            }
            (
                OutputKind::UnidirectionalEsc,
                Calibration::UnidirectionalEsc {
                    stop_us,
                    full_power_us,
                    ..
                },
            ) => *stop_us > 0 && stop_us < full_power_us && *full_power_us < 20_000,
            (
                OutputKind::PositionalServo,
                Calibration::PositionalServo {
                    min_us,
                    center_us,
                    max_us,
                },
            ) => *min_us > 0 && min_us < center_us && center_us < max_us && *max_us < 20_000,
            _ => false,
        };
        if !calibration_ok {
            add(
                Some(a.id),
                "calibration",
                "calibration must match the kind and contain valid power or ordered microsecond pulses",
            );
        }
        if a.kind == OutputKind::UnidirectionalEsc && a.inverted {
            add(
                Some(a.id),
                "inversion",
                "unidirectional ESC cannot be inverted",
            );
        }
        let servo = a.kind == OutputKind::PositionalServo;
        if servo != matches!(a.route, Route::Servo) {
            add(
                Some(a.id),
                "route_kind",
                "servo routes require positional servos; propulsion routes require propulsion outputs",
            );
        }
        if let Route::Manual {
            forward_coefficient,
            turn_coefficient,
        } = &a.route
        {
            if ![forward_coefficient, turn_coefficient]
                .iter()
                .all(|v| v.is_finite() && (-1.0..=1.0).contains(*v))
            {
                add(
                    Some(a.id),
                    "route_coefficients",
                    "manual coefficients must be finite and in [-1, 1]",
                );
            }
        }
        let safe_ok = match &a.safe {
            SafeOutput::Zero => !servo && a.limits.min <= 0.0 && a.limits.max >= 0.0,
            SafeOutput::Position { value } => {
                servo
                    && value.is_finite()
                    && *value >= a.limits.min
                    && *value <= a.limits.max
                    && (-1.0..=1.0).contains(value)
            }
            SafeOutput::Disabled => servo,
        };
        if !safe_ok {
            add(
                Some(a.id),
                "safe_output",
                "safe policy must match kind and lie within limits; propulsion requires zero",
            );
        }
    }
    if errors.is_empty() {
        Ok(())
    } else {
        Err(errors)
    }
}

/// Validate structure and advertised backend compatibility, collecting all failures.
///
/// Checks port uniqueness, supported output kinds, resource ownership, frequencies,
/// and shared timer compatibility. Returns all errors rather than stopping at the first.
pub fn validate_layout(
    layout: &Layout,
    capabilities: &Capabilities,
) -> Result<(), Vec<LayoutError>> {
    let mut errors = validate_structure(layout).err().unwrap_or_default();
    let mut resources = capabilities.occupied_resources.clone();
    let mut ports = BTreeSet::new();
    let mut timers = BTreeMap::new();
    for a in &layout.actuators {
        let mut add = |code: &str, message: &str| {
            errors.push(LayoutError {
                actuator_id: Some(a.id),
                code: code.into(),
                message: message.into(),
            })
        };
        if !ports.insert(&a.port) {
            add("resource_conflict", "port is already assigned");
        }
        let Some(port) = capabilities.ports.get(&a.port) else {
            add("unsupported_port", "port is not advertised by backend");
            continue;
        };
        if !capabilities.supported_kinds.contains(&a.kind) || !port.kinds.contains(&a.kind) {
            add(
                "unsupported_kind",
                "output kind is unsupported on this port",
            );
        }
        // Require ownership metadata: an empty map cannot safely establish alias availability.
        if port.resources.is_empty() {
            add(
                "resource_map",
                "port must advertise occupied physical resources",
            );
        }
        let mut own_resources = BTreeSet::new();
        for resource in &port.resources {
            if resource.is_empty() || !own_resources.insert(resource) {
                add(
                    "resource_map",
                    "resource identifiers must be nonempty and unique per port",
                );
                continue;
            }
            if !resources.insert(resource.clone()) {
                add(
                    "resource_conflict",
                    "physical resource is occupied or shared by another actuator",
                );
            }
        }
        if port.frequency_hz == 0 || (a.kind != OutputKind::DcMotor && port.frequency_hz != 50) {
            add(
                "frequency",
                "ESC and servo channels require 50 Hz; motor frequency must be nonzero",
            );
        }
        if let Some(timer) = &port.timer {
            if let Some(previous) = timers.insert(timer, port.frequency_hz) {
                if previous != port.frequency_hz {
                    add(
                        "timer_conflict",
                        "shared timer channels require equal frequencies",
                    );
                }
            }
        }
    }
    if errors.is_empty() {
        Ok(())
    } else {
        Err(errors)
    }
}

/// Return whether a structurally valid layout has both left/right effort routes
/// and no manual propulsion mixes. This does not certify hardware compatibility.
pub fn supports_feedback(layout: &Layout) -> bool {
    validate_structure(layout).is_ok()
        && layout
            .actuators
            .iter()
            .any(|a| matches!(a.route, Route::LeftEffort))
        && layout
            .actuators
            .iter()
            .any(|a| matches!(a.route, Route::RightEffort))
        && !layout
            .actuators
            .iter()
            .any(|a| matches!(a.route, Route::Manual { .. }))
}
