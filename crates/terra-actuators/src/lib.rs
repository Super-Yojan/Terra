//! Normalized actuator configuration and command routing.
//!
//! Start with [`Layout`] for hardware assignments, then [`route_commands`] for
//! turning left/right or manual intent into per-actuator values. The `protocol`
//! module defines the binary command envelope. Routing does not authorize motion;
//! the hardware adapter owns inversion, arming, and physical output.

/// Hardware layout schema, calibration, and backend capability validation.
pub mod layout;
/// Normalized intent routing without hardware arming or inversion.
pub mod routing;
pub use layout::*;
pub use routing::*;

/// One normalized command associated with a layout actuator ID.
///
/// The containing protocol or routing function validates range and finiteness;
/// constructing this record alone performs no validation.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ActuatorValue {
    /// Actuator ID from the active layout.
    pub id: u8,
    /// Normalized command: `[-1, 1]`, or `[0, 1]` for a unidirectional ESC.
    pub value: f32,
}

/// Layout or command validation failure; no hardware output is performed.
#[derive(Clone, Debug, PartialEq)]
pub enum ActuatorError {
    /// All structural layout errors found before routing.
    InvalidLayout(Vec<LayoutError>),
    /// Human-readable explanation of a malformed or out-of-range input.
    InvalidInput(String),
}
impl std::fmt::Display for ActuatorError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidLayout(errors) => write!(f, "invalid layout: {errors:?}"),
            Self::InvalidInput(message) => write!(f, "invalid routing input: {message}"),
        }
    }
}
impl std::error::Error for ActuatorError {}
/// Binary command and JSON control framing with bounded fragment assembly.
pub mod protocol;
