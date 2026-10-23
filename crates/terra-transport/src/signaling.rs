//! Zenoh inbox policy for WebRTC signaling.
//!
//! The message types and session rules live in `fpv-signaling`. This module only
//! decides which samples may enter the phone's signaling queue, and drops the
//! oldest dashboard ICE candidate when that queue is full so a start or answer
//! can still land. Nothing here is a drive or arm command.

use std::collections::VecDeque;

pub use fpv_signaling::MAX_BYTES as WEBRTC_SIGNALING_MAX_BYTES;

pub(crate) const SIGNALING_QUEUE_CAPACITY: usize = 64;

/// Dashboard-to-phone topics. The phone's own publications are not in this set.
pub fn webrtc_inbound(kind: &str) -> bool {
    fpv_signaling::Topic::parse(kind).is_some_and(fpv_signaling::Topic::is_phone_inbound)
}

/// Phone-to-dashboard topics. `publish_signaling` rejects every other suffix.
pub fn webrtc_outbound(kind: &str) -> bool {
    fpv_signaling::Topic::parse(kind).is_some_and(fpv_signaling::Topic::is_phone_outbound)
}

/// Either direction. Telemetry replay refuses these suffixes.
pub fn is_webrtc_topic(kind: &str) -> bool {
    fpv_signaling::is_signaling_topic(kind)
}

/// True when `value` is a valid phone-to-dashboard signaling sample.
pub fn accept_phone_outbound(kind: &str, value: &str) -> bool {
    webrtc_outbound(kind) && fpv_signaling::decode(kind, value).is_some()
}

/// Append one signaling sample. When the queue is full, drop the oldest trickle
/// candidate first so a start, stop, or answer can still land. The caller has
/// already enforced [`WEBRTC_SIGNALING_MAX_BYTES`].
pub(crate) fn enqueue_signaling(
    queue: &mut VecDeque<(String, Vec<u8>)>,
    kind: String,
    bytes: Vec<u8>,
) {
    if queue.len() >= SIGNALING_QUEUE_CAPACITY {
        let ice = fpv_signaling::Topic::IceDashboard.suffix();
        if let Some(index) = queue.iter().position(|(queued, _)| queued == ice) {
            queue.remove(index);
        } else {
            queue.pop_front();
        }
    }
    queue.push_back((kind, bytes));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn webrtc_topics_stay_out_of_the_control_allowlist() {
        for kind in [
            "webrtc/request",
            "webrtc/answer",
            "webrtc/ice/dashboard",
            "webrtc/offer",
            "webrtc/ice/phone",
            "webrtc/state",
        ] {
            assert!(is_webrtc_topic(kind), "{kind}");
            assert!(!crate::is_control_topic(kind), "{kind}");
        }
        assert!(!webrtc_outbound("hardware"));
        assert!(!webrtc_outbound("teleop"));
        assert!(!webrtc_outbound("cmd_vel"));
        assert!(!webrtc_inbound("webrtc/offer"));
        assert!(!webrtc_inbound("webrtc/state"));
        assert!(!accept_phone_outbound(
            "hardware",
            r#"{"action":"arm","token":"no"}"#
        ));
        assert!(!accept_phone_outbound(
            "teleop",
            r#"{"linear":1,"angular":0}"#
        ));
    }

    #[test]
    fn a_full_queue_drops_trickle_ice_before_a_start_request() {
        let mut queue = VecDeque::new();
        for index in 0..SIGNALING_QUEUE_CAPACITY {
            enqueue_signaling(&mut queue, "webrtc/ice/dashboard".into(), vec![index as u8]);
        }
        enqueue_signaling(&mut queue, "webrtc/request".into(), b"start".to_vec());
        assert_eq!(queue.len(), SIGNALING_QUEUE_CAPACITY);
        assert!(
            queue
                .iter()
                .any(|(kind, bytes)| kind == "webrtc/request" && bytes == b"start")
        );
    }
}
