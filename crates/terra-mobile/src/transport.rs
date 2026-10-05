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
    pub fn status(&self) -> String {
        self.connection.status()
    }
    pub fn disconnect(&self) {
        self.connection.disconnect();
    }
}
