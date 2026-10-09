//! Strict JSON and portable wire encoding at the UniFFI boundary.
use serde::Deserialize;
use terra_actuators::protocol::{self, CommandFrame, CommandKind};
use terra_actuators::{ActuatorValue, Capabilities, Layout, RoutingInput};
#[derive(Debug, thiserror::Error, uniffi::Error)]
pub enum MobileActuatorError {
    #[error("{message}")]
    InvalidInput { message: String },
}
fn invalid(error: impl std::fmt::Display) -> MobileActuatorError {
    MobileActuatorError::InvalidInput {
        message: error.to_string(),
    }
}
fn parse<T: serde::de::DeserializeOwned>(json: &str) -> Result<T, MobileActuatorError> {
    if json.len() > protocol::JSON_LIMIT {
        return Err(invalid("JSON document too large"));
    }
    serde_json::from_str(json).map_err(invalid)
}
#[uniffi::export]
pub fn actuator_validate_layout(
    layout_json: String,
    capabilities_json: String,
) -> Result<String, MobileActuatorError> {
    let layout: Layout = parse(&layout_json)?;
    let capabilities: Capabilities = parse(&capabilities_json)?;
    let errors = terra_actuators::validate_layout(&layout, &capabilities)
        .err()
        .unwrap_or_default();
    serde_json::to_string(&errors).map_err(invalid)
}
#[uniffi::export]
pub fn actuator_route(
    layout_json: String,
    input_json: String,
) -> Result<String, MobileActuatorError> {
    let layout: Layout = parse(&layout_json)?;
    let input: RoutingInput = parse(&input_json)?;
    serde_json::to_string(&terra_actuators::route_commands(&layout, &input).map_err(invalid)?)
        .map_err(invalid)
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct FrameWire {
    kind: String,
    session: u32,
    revision: u32,
    sequence: u32,
    values: Vec<ActuatorValue>,
}
#[uniffi::export]
pub fn actuator_encode_frame(frame_json: String) -> Result<Vec<u8>, MobileActuatorError> {
    let wire: FrameWire = parse(&frame_json)?;
    if wire.sequence == u32::MAX {
        return Err(invalid("sequence exhausted"));
    }
    let kind = match wire.kind.as_str() {
        "drive" => CommandKind::Drive,
        "arm" => CommandKind::Arm,
        "disarm" => CommandKind::Disarm,
        "emergency_stop" => CommandKind::EmergencyStop,
        _ => return Err(invalid("unknown command kind")),
    };
    protocol::encode_frame(&CommandFrame {
        kind,
        session: wire.session,
        revision: wire.revision,
        sequence: wire.sequence,
        values: wire.values,
    })
    .map_err(invalid)
}
#[uniffi::export]
pub fn actuator_fragment(
    message_id: u16,
    payload: Vec<u8>,
    maximum_write_length: u32,
) -> Result<Vec<Vec<u8>>, MobileActuatorError> {
    protocol::fragment_message(
        &payload,
        message_id,
        maximum_write_length as usize,
        protocol::JSON_LIMIT,
    )
    .map_err(invalid)
}

#[uniffi::export]
pub fn actuator_supports_feedback(layout_json: String) -> Result<bool, MobileActuatorError> {
    let layout: Layout = parse(&layout_json)?;
    Ok(terra_actuators::supports_feedback(&layout))
}
