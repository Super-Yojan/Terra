//! Zenoh runs in Rust; Swift only refreshes the command lease and displays status.
use std::sync::Arc;
use terra_transport::{RoverConnection, TransportError};
#[derive(Debug, thiserror::Error, uniffi::Error)]
pub enum ZenohError {
    #[error("{message}")]
    Failed { message: String },
}
impl From<TransportError> for ZenohError {
    fn from(error: TransportError) -> Self {
        Self::Failed {
            message: error.to_string(),
        }
    }
}
#[derive(Clone, uniffi::Record)]
pub struct MobileDepthFrame {
    pub sequence: u64,
    pub timestamp: f64,
    pub width: u32,
    pub height: u32,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub camera_x: f64,
    pub camera_y: f64,
    pub camera_z: f64,
    pub quaternion_x: f64,
    pub quaternion_y: f64,
    pub quaternion_z: f64,
    pub quaternion_w: f64,
    pub body_x: f64,
    pub body_y: f64,
    pub body_yaw: f64,
    pub depth_metres: Vec<f32>,
}
#[derive(uniffi::Object)]
pub struct MobileZenohClient {
    connection: RoverConnection,
}
#[uniffi::export]
impl MobileZenohClient {
    #[uniffi::constructor]
    pub fn new(endpoint: String, prefix: String, rover_id: u64) -> Result<Arc<Self>, ZenohError> {
        Ok(Arc::new(Self {
            connection: RoverConnection::connect(&endpoint, &prefix, rover_id)?,
        }))
    }
    pub fn set_target(&self, linear: f64, angular: f64) -> Result<(), ZenohError> {
        self.connection.set_target(linear, angular)?;
        Ok(())
    }
    pub fn send_action(&self, kind: String, payload: String) -> Result<(), ZenohError> {
        self.connection.send_action(&kind, &payload)?;
        Ok(())
    }
    pub fn autonomy_status(&self) -> String {
        self.connection.autonomy_status()
    }
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
    pub fn disconnect(&self) {
        self.connection.disconnect();
    }
}
