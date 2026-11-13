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
/// Check a router endpoint and topic prefix before any session opens.
///
/// The endpoint is `tcp/host:port` or `tls/host:port` (IPv6 hosts in brackets,
/// nonzero port, at most 512 bytes, no whitespace). The prefix is a Zenoh key
/// of at most 256 bytes without wildcards or leading/trailing slashes.
/// Returns `InvalidSettings` for anything else. Callers trim user input first.
pub fn validate(endpoint: &str, prefix: &str) -> Result<(), TransportError> {
    let address = endpoint
        .strip_prefix("tcp/")
        .or_else(|| endpoint.strip_prefix("tls/"));
    let host_port = address.and_then(|a| a.rsplit_once(':'));
    let host_ok = host_port.is_some_and(|(host, _)| {
        !host.is_empty()
            && !host.contains('/')
            && if host.contains(':') {
                host.len() > 2 && host.starts_with('[') && host.ends_with(']')
            } else {
                !host.contains(['[', ']'])
            }
    });
    let port_ok = host_port.is_some_and(|(_, port)| {
        port.chars().all(|c| c.is_ascii_digit()) && port.parse::<u16>().is_ok_and(|p| p != 0)
    });
    if endpoint.len() > 512
        || !host_ok
        || !port_ok
        || endpoint.chars().any(char::is_whitespace)
        || endpoint.contains(['#', '?'])
        || endpoint.parse::<zenoh::config::EndPoint>().is_err()
        || prefix.len() > 256
        || prefix.contains(['*', '#'])
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
        assert!(validate("tcp/[::1]:7447", "terra/rover").is_ok());
        for endpoint in [
            "",
            "tcp/:7448",
            "tcp/host:0",
            "tcp/host:7448:",
            "tcp/host:65536",
            "tcp/a b:7448",
            "tcp/::1:7448",
        ] {
            assert!(validate(endpoint, "terra/rover").is_err(), "{endpoint}");
        }
        assert!(validate("tcp/host:7448", "/terra").is_err());
        assert!(validate("tcp/host:7448", "terra/").is_err());
        assert!(validate("tcp/host:7448", "").is_err());
    }
}
