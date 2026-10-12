//! Normalized actuator configuration and command routing.
//!
//! Start with [`Layout`] for hardware assignments, then [`route_commands`] for
//! turning left/right or manual intent into per-actuator values. The `protocol`
//! module defines the binary command envelope. Routing does not authorize motion;
//! the hardware adapter owns inversion, arming, and physical output.

pub mod layout;
pub mod routing;
pub use layout::*;
pub use routing::*;

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ActuatorValue {
    pub id: u8,
    pub value: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub enum ActuatorError {
    InvalidLayout(Vec<LayoutError>),
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
pub mod protocol;
