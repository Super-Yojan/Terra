//! Zenoh connections for remote intent, telemetry, and dashboard control.
//!
//! [`ControlPlane`] queues operator requests and exposes onboard status.
//! [`RoverConnection`] is the simulator client: leased velocity out, remote depth in.
//! Transport carries intent; the onboard autonomy and hardware layers still decide
//! whether it may cause motion. Disconnects must not replay old commands.

#[derive(Debug)]
/// Endpoint validation, command admission, or Zenoh session failure.
pub enum TransportError {
    /// Endpoint or key prefix violates the transport configuration contract.
    InvalidSettings,
    /// Velocity, action topic, JSON, or payload size is invalid.
    InvalidTarget,
    /// The connection worker has stopped or disconnect was requested.
    Closed,
    /// Zenoh, worker creation, or bounded action-queue failure, with diagnostic text.
    Network(String),
}
impl std::fmt::Display for TransportError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for TransportError {}
fn validate(endpoint: &str, prefix: &str) -> Result<(), TransportError> {
    if endpoint.len() > 512
        || !(endpoint.starts_with("tcp/") || endpoint.starts_with("tls/"))
        || endpoint.contains(['\n', '\r', '#', '?'])
        || endpoint.parse::<zenoh::config::EndPoint>().is_err()
        || prefix.len() > 256
        || prefix.contains('*')
        || zenoh::key_expr::KeyExpr::new(prefix).is_err()
    {
        return Err(TransportError::InvalidSettings);
    }
    Ok(())
}
mod client;
mod control_plane;
mod inbox;
mod signaling;
pub use client::{RemoteDepthFrame, RoverConnection, decode_depth_frame};
pub use control_plane::ControlPlane;
pub use signaling::WEBRTC_SIGNALING_MAX_BYTES;

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    /// Checks TCP/TLS endpoint admission and wildcard prefix rejection.
    fn validates_endpoint_and_prefix() {
        assert!(validate("tcp/127.0.0.1:7447", "terra/rover").is_ok());
        assert!(validate("udp/127.0.0.1:7447", "terra/rover").is_err());
        assert!(validate("tcp/127.0.0.1:7447", "terra/*").is_err());
    }
}
