//! WebRTC signaling beside the control inbox.
//!
//! SDP offers do not fit the 2048-byte control cap. These topics have their own
//! 65536-byte cap and their own queue. They are not control topics, so a video
//! message cannot arm, disarm, or drive.

use std::collections::VecDeque;

/// Largest accepted signaling payload, in bytes. The control inbox stays at 2048.
pub const WEBRTC_SIGNALING_MAX_BYTES: usize = 65_536;
pub(crate) const SIGNALING_QUEUE_CAPACITY: usize = 64;

/// Dashboard-to-phone topics. The phone's own publications are not in this set,
/// so a local echo of an offer or candidate is not treated as a command.
pub fn webrtc_inbound(kind: &str) -> bool {
    matches!(
        kind,
        "webrtc/request" | "webrtc/answer" | "webrtc/ice/dashboard"
    )
}

/// Phone-to-dashboard topics. `publish_signaling` rejects every other suffix.
pub fn webrtc_outbound(kind: &str) -> bool {
    matches!(kind, "webrtc/offer" | "webrtc/ice/phone" | "webrtc/state")
}

/// Either direction. Telemetry replay refuses these suffixes.
pub fn is_webrtc_topic(kind: &str) -> bool {
    webrtc_inbound(kind) || webrtc_outbound(kind)
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
        if let Some(index) = queue
            .iter()
            .position(|(queued, _)| queued == "webrtc/ice/dashboard")
        {
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
        assert!(
            queue
                .iter()
                .all(|(kind, _)| kind == "webrtc/ice/dashboard" || kind == "webrtc/request")
        );
    }
}
