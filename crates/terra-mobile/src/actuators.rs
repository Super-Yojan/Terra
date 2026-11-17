//! Bluetooth fragmenting for the iOS rover link. Routing and encoding of
//! drive frames happen in `terra-robot`.
use terra_actuators::protocol;

#[derive(Debug, thiserror::Error, uniffi::Error)]
/// A payload that cannot be fragmented within protocol limits.
pub enum MobileActuatorError {
    #[error("{message}")]
    /// Rejected input; `message` contains the underlying diagnostic.
    InvalidInput {
        /// Human-readable diagnostic from the rejected operation.
        message: String,
    },
}

#[uniffi::export]
/// Split payload bytes into protocol fragments for a maximum write size in bytes.
/// `message_id` identifies the reassembled message. The protocol JSON size cap applies.
///
/// # Errors
/// Returns `InvalidInput` if the payload or write budget cannot be fragmented
/// within protocol limits.
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
    .map_err(|e| MobileActuatorError::InvalidInput {
        message: e.to_string(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fragments_fit_the_write_length() {
        let parts = actuator_fragment(7, b"hello".to_vec(), 7).unwrap();
        assert_eq!(parts.len(), 2);
        assert!(parts.iter().all(|p| p.len() <= 7));
    }
}
