//! Zenoh runs in Rust; Swift only refreshes the command lease and displays status.
use std::sync::Arc;
use terra_transport::{RoverConnection, TransportError};
#[derive(Debug, thiserror::Error, uniffi::Error)]
/// Phone-facing wrapper for transport validation, closure, and network failures.
pub enum ZenohError {
    #[error("{message}")]
    /// Transport operation failed; `message` contains its diagnostic.
    Failed {
        /// Human-readable diagnostic from the rejected operation.
        message: String,
    },
}
impl From<TransportError> for ZenohError {
    fn from(error: TransportError) -> Self {
        Self::Failed {
            message: error.to_string(),
        }
    }
}
#[derive(Clone, uniffi::Record)]
/// Newest remote posed depth image; camera and body poses share the exposure time.
/// Intrinsics follow integer-pixel principal-point coordinates.
pub struct MobileDepthFrame {
    /// Source depth-frame sequence number.
    pub sequence: u64,
    /// Exposure time in source-clock seconds.
    pub timestamp: f64,
    /// Depth image width, pixels.
    pub width: u32,
    /// Depth image height, pixels.
    pub height: u32,
    /// Horizontal focal length, pixels.
    pub fx: f64,
    /// Vertical focal length, pixels.
    pub fy: f64,
    /// Horizontal principal point in integer-pixel coordinates.
    pub cx: f64,
    /// Vertical principal point in integer-pixel coordinates.
    pub cy: f64,
    /// Camera origin world +X coordinate, metres.
    pub camera_x: f64,
    /// Camera origin world +Y coordinate, metres.
    pub camera_y: f64,
    /// Camera origin world +Z coordinate, metres.
    pub camera_z: f64,
    /// Optical-camera-to-world quaternion X component.
    pub quaternion_x: f64,
    /// Optical-camera-to-world quaternion Y component.
    pub quaternion_y: f64,
    /// Optical-camera-to-world quaternion Z component.
    pub quaternion_z: f64,
    /// Optical-camera-to-world quaternion scalar component.
    pub quaternion_w: f64,
    /// Body origin world +X coordinate, metres.
    pub body_x: f64,
    /// Body origin world +Y coordinate, metres.
    pub body_y: f64,
    /// Body heading from world +X toward +Y, radians.
    pub body_yaw: f64,
    /// Row-major axial optical depths in metres; length must equal width × height.
    pub depth_metres: Vec<f32>,
}
#[derive(uniffi::Object)]
/// UniFFI client for leased remote velocity intent and latest remote telemetry.
pub struct MobileZenohClient {
    connection: RoverConnection,
}
#[uniffi::export]
impl MobileZenohClient {
    #[uniffi::constructor]
    /// Open a direct Zenoh client for `prefix/rover_id`. Run off the UI thread.
    /// TCP/TLS endpoints and literal valid key prefixes are required; connection
    /// timeout is two seconds. Returns `Failed` on validation or network failure.
    pub fn new(endpoint: String, prefix: String, rover_id: u64) -> Result<Arc<Self>, ZenohError> {
        Ok(Arc::new(Self {
            connection: RoverConnection::connect(&endpoint, &prefix, rover_id)?,
        }))
    }
    /// Refresh the 250 ms velocity lease: `linear` in m/s, `angular` in rad/s.
    /// Both must be finite and within ±2. Invalid targets revoke the previous lease.
    /// Returns `Failed` for invalid targets or a closed connection.
    pub fn set_target(&self, linear: f64, angular: f64) -> Result<(), ZenohError> {
        self.connection.set_target(linear, angular)?;
        Ok(())
    }
    /// Queue a JSON action for an allowed autonomy, safety, goal, or search topic.
    /// Payloads are limited to 2048 bytes and the queue to 64 actions.
    /// Returns `Failed` for invalid topic/JSON, a full queue, or a closed connection.
    pub fn send_action(&self, kind: String, payload: String) -> Result<(), ZenohError> {
        self.connection.send_action(&kind, &payload)?;
        Ok(())
    }
    /// Return latest authority JSON augmented with receipt ages in seconds,
    /// search state, and reports younger than 2.5 seconds. This is cached telemetry.
    pub fn autonomy_status(&self) -> String {
        self.connection.autonomy_status()
    }
    /// Return human-readable transport health and accepted depth-frame count.
    pub fn status(&self) -> String {
        self.connection.status()
    }
    /// Newest unconsumed simulator depth frame, including its exposure pose.
    pub fn take_depth(&self) -> Option<MobileDepthFrame> {
        self.connection.take_depth().map(|frame| MobileDepthFrame {
            sequence: frame.sequence,
            timestamp: frame.timestamp,
            width: frame.width,
            height: frame.height,
            fx: frame.fx,
            fy: frame.fy,
            cx: frame.cx,
            cy: frame.cy,
            camera_x: frame.camera_x,
            camera_y: frame.camera_y,
            camera_z: frame.camera_z,
            quaternion_x: frame.quaternion_x,
            quaternion_y: frame.quaternion_y,
            quaternion_z: frame.quaternion_z,
            quaternion_w: frame.quaternion_w,
            body_x: frame.body_x,
            body_y: frame.body_y,
            body_yaw: frame.body_yaw,
            depth_metres: frame.depth_metres,
        })
    }
    /// Request a final zero target, join the network worker, and close the session.
    /// May block; repeated calls are harmless. Network failure can prevent delivery.
    pub fn disconnect(&self) {
        self.connection.disconnect();
    }
}
