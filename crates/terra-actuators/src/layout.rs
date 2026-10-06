use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Layout {
    pub schema_version: u8,
    pub revision: u32,
    pub actuators: Vec<Actuator>,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Actuator {
    pub id: u8,
    pub name: String,
    pub port: String,
    pub kind: OutputKind,
    pub inverted: bool,
    pub limits: CommandLimits,
    pub calibration: Calibration,
    pub route: Route,
    pub safe: SafeOutput,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OutputKind { DcMotor, BidirectionalEsc, UnidirectionalEsc, PositionalServo }
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CommandLimits { pub min: f32, pub max: f32 }
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum Calibration {
    DcMotor { max_power_fraction: f32 },
    BidirectionalEsc { reverse_us: u32, neutral_us: u32, forward_us: u32, arming_duration_ms: u32 },
    UnidirectionalEsc { stop_us: u32, full_power_us: u32, arming_duration_ms: u32 },
    PositionalServo { min_us: u32, center_us: u32, max_us: u32 },
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum Route {
    LeftEffort,
    RightEffort,
    Manual { forward_coefficient: f32, turn_coefficient: f32 },
    Servo,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum SafeOutput { Zero, Position { value: f32 }, Disabled }

/// Backend supplied ownership and timer constraints; names are opaque identities.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PortCapability {
    pub kinds: Vec<OutputKind>,
    pub resources: Vec<String>,
    pub timer: Option<String>,
    pub frequency_hz: u32,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Capabilities {
    pub board: String,
    pub library: String,
    pub library_version: String,
    pub supported_kinds: Vec<OutputKind>,
    pub ports: BTreeMap<String, PortCapability>,
    pub occupied_resources: BTreeSet<String>,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LayoutError { pub actuator_id: Option<u8>, pub code: String, pub message: String }

/// Structural validation is also used before routing, without claiming hardware availability.
pub fn validate_structure(layout: &Layout) -> Result<(), Vec<LayoutError>> {
    let mut errors = Vec::new();
    let mut add = |id, code: &str, message: &str| errors.push(LayoutError {
        actuator_id: id, code: code.into(), message: message.into(),
    });
    if layout.schema_version != 1 { add(None, "schema_version", "schema version must be 1"); }
    if layout.actuators.len() > 16 { add(None, "actuator_count", "at most 16 actuators are supported"); }
    let mut ids = BTreeSet::new();
    for a in &layout.actuators {
        if !ids.insert(a.id) { add(Some(a.id), "duplicate_id", "actuator IDs must be unique"); }
        if a.port.trim().is_empty() { add(Some(a.id), "port", "port must not be empty"); }
        let lower = if a.kind == OutputKind::UnidirectionalEsc { 0.0 } else { -1.0 };
        if !a.limits.min.is_finite() || !a.limits.max.is_finite() || a.limits.min < lower || a.limits.max > 1.0 || a.limits.min > a.limits.max {
            add(Some(a.id), "command_limits", "limits must be finite, ordered, and within the kind's normalized range");
        }
        let calibration_ok = match (&a.kind, &a.calibration) {
            (OutputKind::DcMotor, Calibration::DcMotor { max_power_fraction }) => max_power_fraction.is_finite() && *max_power_fraction > 0.0 && *max_power_fraction <= 1.0,
            (OutputKind::BidirectionalEsc, Calibration::BidirectionalEsc { reverse_us, neutral_us, forward_us, .. }) => *reverse_us > 0 && reverse_us < neutral_us && neutral_us < forward_us && *forward_us < 20_000,
            (OutputKind::UnidirectionalEsc, Calibration::UnidirectionalEsc { stop_us, full_power_us, .. }) => *stop_us > 0 && stop_us < full_power_us && *full_power_us < 20_000,
            (OutputKind::PositionalServo, Calibration::PositionalServo { min_us, center_us, max_us }) => *min_us > 0 && min_us < center_us && center_us < max_us && *max_us < 20_000,
            _ => false,
        };
        if !calibration_ok { add(Some(a.id), "calibration", "calibration must match the kind and contain valid power or ordered microsecond pulses"); }
        let servo = a.kind == OutputKind::PositionalServo;
        if servo != matches!(a.route, Route::Servo) { add(Some(a.id), "route_kind", "servo routes require positional servos; propulsion routes require propulsion outputs"); }
        if let Route::Manual { forward_coefficient, turn_coefficient } = &a.route {
            if ![forward_coefficient, turn_coefficient].iter().all(|v| v.is_finite() && (-1.0..=1.0).contains(*v)) {
                add(Some(a.id), "route_coefficients", "manual coefficients must be finite and in [-1, 1]");
            }
        }
        let safe_ok = match &a.safe {
            SafeOutput::Zero => !servo && a.limits.min <= 0.0 && a.limits.max >= 0.0,
            SafeOutput::Position { value } => servo && value.is_finite() && *value >= a.limits.min && *value <= a.limits.max && (-1.0..=1.0).contains(value),
            SafeOutput::Disabled => servo,
        };
        if !safe_ok { add(Some(a.id), "safe_output", "safe policy must match kind and lie within limits; propulsion requires zero"); }
    }
    if errors.is_empty() { Ok(()) } else { Err(errors) }
}

pub fn validate_layout(layout: &Layout, capabilities: &Capabilities) -> Result<(), Vec<LayoutError>> {
    let mut errors = validate_structure(layout).err().unwrap_or_default();
    let mut resources = capabilities.occupied_resources.clone();
    let mut ports = BTreeSet::new();
    let mut timers = BTreeMap::new();
    for a in &layout.actuators {
        let mut add = |code: &str, message: &str| errors.push(LayoutError { actuator_id: Some(a.id), code: code.into(), message: message.into() });
        if !ports.insert(&a.port) { add("resource_conflict", "port is already assigned"); }
        let Some(port) = capabilities.ports.get(&a.port) else { add("unsupported_port", "port is not advertised by backend"); continue; };
        if !capabilities.supported_kinds.contains(&a.kind) || !port.kinds.contains(&a.kind) { add("unsupported_kind", "output kind is unsupported on this port"); }
        // Require ownership metadata: an empty map cannot safely establish alias availability.
        if port.resources.is_empty() { add("resource_map", "port must advertise occupied physical resources"); }
        let mut own_resources = BTreeSet::new();
        for resource in &port.resources {
            if resource.is_empty() || !own_resources.insert(resource) { add("resource_map", "resource identifiers must be nonempty and unique per port"); continue; }
            if !resources.insert(resource.clone()) { add("resource_conflict", "physical resource is occupied or shared by another actuator"); }
        }
        if port.frequency_hz == 0 || (a.kind != OutputKind::DcMotor && port.frequency_hz != 50) { add("frequency", "ESC and servo channels require 50 Hz; motor frequency must be nonzero"); }
        if let Some(timer) = &port.timer {
            if let Some(previous) = timers.insert(timer, port.frequency_hz) {
                if previous != port.frequency_hz { add("timer_conflict", "shared timer channels require equal frequencies"); }
            }
        }
    }
    if errors.is_empty() { Ok(()) } else { Err(errors) }
}

pub fn supports_feedback(layout: &Layout) -> bool {
    validate_structure(layout).is_ok()
        && layout.actuators.iter().any(|a| matches!(a.route, Route::LeftEffort))
        && layout.actuators.iter().any(|a| matches!(a.route, Route::RightEffort))
        && !layout.actuators.iter().any(|a| matches!(a.route, Route::Manual { .. }))
}
