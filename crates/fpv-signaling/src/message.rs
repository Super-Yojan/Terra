//! JSON messages for the six FPV topics.
//!
//! Unknown fields are ignored. `v` must be the JSON number 1. Session ids are
//! UUID strings; comparison is case-insensitive and the original spelling is kept.

use serde_json::{Map, Number, Value};

/// Largest signaling payload, in bytes. The fleet control inbox stays at 2048.
pub const MAX_BYTES: usize = 65_536;

/// Fleet prefix used when the phone has not been given another one.
pub const DEFAULT_PREFIX: &str = "terra/phone";

/// One of the six signaling suffixes. None of these is a drive or arm topic.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Topic {
    /// Dashboard to phone. `start` or `stop`.
    Request,
    /// Phone to dashboard. SDP offer.
    Offer,
    /// Dashboard to phone. SDP answer.
    Answer,
    /// Phone to dashboard. Trickle ICE, or an empty candidate when gathering is done.
    IcePhone,
    /// Dashboard to phone. Trickle ICE, or an empty candidate when gathering is done.
    IceDashboard,
    /// Phone to dashboard. `connecting`, `connected`, `failed`, or `closed`.
    State,
}

impl Topic {
    /// Topic suffix after `{prefix}/{rover_id}/`.
    pub fn suffix(self) -> &'static str {
        match self {
            Self::Request => "webrtc/request",
            Self::Offer => "webrtc/offer",
            Self::Answer => "webrtc/answer",
            Self::IcePhone => "webrtc/ice/phone",
            Self::IceDashboard => "webrtc/ice/dashboard",
            Self::State => "webrtc/state",
        }
    }

    /// Parse a topic suffix. Drive, arm, and telemetry suffixes are `None`.
    pub fn parse(suffix: &str) -> Option<Self> {
        Some(match suffix {
            "webrtc/request" => Self::Request,
            "webrtc/offer" => Self::Offer,
            "webrtc/answer" => Self::Answer,
            "webrtc/ice/phone" => Self::IcePhone,
            "webrtc/ice/dashboard" => Self::IceDashboard,
            "webrtc/state" => Self::State,
            _ => return None,
        })
    }

    /// Dashboard `request`, `answer`, and `ice/dashboard`.
    pub fn is_phone_inbound(self) -> bool {
        matches!(self, Self::Request | Self::Answer | Self::IceDashboard)
    }

    /// Phone `offer`, `ice/phone`, and `state`.
    pub fn is_phone_outbound(self) -> bool {
        matches!(self, Self::Offer | Self::IcePhone | Self::State)
    }

    pub fn is_dashboard_inbound(self) -> bool {
        self.is_phone_outbound()
    }

    pub fn is_dashboard_outbound(self) -> bool {
        self.is_phone_inbound()
    }
}

/// True for any of the six signaling suffixes.
pub fn is_signaling_topic(suffix: &str) -> bool {
    Topic::parse(suffix).is_some()
}

/// `{prefix}/{rover_id}/{suffix}`, with one slash between the parts.
pub fn topic_key(prefix: &str, rover_id: impl std::fmt::Display, topic: Topic) -> String {
    format!("{}/{rover_id}/{}", prefix.trim_matches('/'), topic.suffix())
}

/// UUID text as it appeared on the `start` that created the call.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SessionId(String);

impl SessionId {
    /// Accept an 8-4-4-4-12 hex UUID. The stored spelling is unchanged.
    pub fn parse(text: &str) -> Option<Self> {
        is_uuid(text).then(|| Self(text.to_string()))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }

    /// Case-insensitive UUID match. Hyphens are required on both sides.
    pub fn matches(&self, text: &str) -> bool {
        self.0.eq_ignore_ascii_case(text)
    }
}

/// `start` or `stop` on `webrtc/request`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RequestAction {
    Start,
    Stop,
}

impl RequestAction {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Start => "start",
            Self::Stop => "stop",
        }
    }

    fn parse(text: &str) -> Option<Self> {
        match text {
            "start" => Some(Self::Start),
            "stop" => Some(Self::Stop),
            _ => None,
        }
    }
}

/// Phone-published connection state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CallState {
    Connecting,
    Connected,
    Failed,
    Closed,
}

impl CallState {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Connecting => "connecting",
            Self::Connected => "connected",
            Self::Failed => "failed",
            Self::Closed => "closed",
        }
    }

    pub fn parse(text: &str) -> Option<Self> {
        match text {
            "connecting" => Some(Self::Connecting),
            "connected" => Some(Self::Connected),
            "failed" => Some(Self::Failed),
            "closed" => Some(Self::Closed),
            _ => None,
        }
    }
}

/// One decoded signaling sample. The topic and the body must agree.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Message {
    pub topic: Topic,
    pub session: SessionId,
    pub body: Body,
}

/// Payload besides the shared version and session fields.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Body {
    Request(RequestAction),
    Sdp(String),
    Ice {
        candidate: String,
        sdp_mid: String,
        sdp_m_line_index: i32,
    },
    State(CallState),
}

impl Body {
    /// An empty ICE candidate means the sender is done gathering.
    pub fn is_end_of_candidates(&self) -> bool {
        matches!(self, Self::Ice { candidate, .. } if candidate.is_empty())
    }
}

/// Decode one sample. `None` leaves the active session unchanged.
/// Extra JSON fields are ignored. A payload over [`MAX_BYTES`] is rejected.
pub fn decode(topic: &str, payload: &str) -> Option<Message> {
    if payload.len() > MAX_BYTES {
        return None;
    }
    let topic = Topic::parse(topic)?;
    let value: Value = serde_json::from_str(payload).ok()?;
    let object = value.as_object()?;
    if !version_is_one(object.get("v")?) {
        return None;
    }
    let session = SessionId::parse(object.get("session")?.as_str()?)?;
    let body = match topic {
        Topic::Request => Body::Request(RequestAction::parse(object.get("action")?.as_str()?)?),
        Topic::Offer | Topic::Answer => {
            let sdp = object.get("sdp")?.as_str()?;
            if sdp.is_empty() {
                return None;
            }
            Body::Sdp(sdp.to_string())
        }
        Topic::IcePhone | Topic::IceDashboard => Body::Ice {
            candidate: object.get("candidate")?.as_str()?.to_string(),
            sdp_mid: sdp_mid(object),
            sdp_m_line_index: line_index(object.get("sdpMLineIndex")?)?,
        },
        Topic::State => Body::State(CallState::parse(object.get("state")?.as_str()?)?),
    };
    Some(Message {
        topic,
        session,
        body,
    })
}

pub fn encode_request(session: &SessionId, action: RequestAction) -> Option<String> {
    bounded(&serde_json::json!({
        "v": 1,
        "session": session.as_str(),
        "action": action.as_str(),
    }))
}

pub fn encode_offer(session: &SessionId, sdp: &str) -> Option<String> {
    encode_sdp(session, sdp)
}

pub fn encode_answer(session: &SessionId, sdp: &str) -> Option<String> {
    encode_sdp(session, sdp)
}

pub fn encode_ice(
    session: &SessionId,
    candidate: &str,
    sdp_mid: &str,
    sdp_m_line_index: i32,
) -> Option<String> {
    if sdp_m_line_index < 0 {
        return None;
    }
    bounded(&serde_json::json!({
        "v": 1,
        "session": session.as_str(),
        "candidate": candidate,
        "sdpMid": sdp_mid,
        "sdpMLineIndex": sdp_m_line_index,
    }))
}

pub fn encode_state(session: &SessionId, state: CallState) -> Option<String> {
    bounded(&serde_json::json!({
        "v": 1,
        "session": session.as_str(),
        "state": state.as_str(),
    }))
}

fn encode_sdp(session: &SessionId, sdp: &str) -> Option<String> {
    if sdp.is_empty() {
        return None;
    }
    bounded(&serde_json::json!({
        "v": 1,
        "session": session.as_str(),
        "sdp": sdp,
    }))
}

fn bounded(value: &Value) -> Option<String> {
    let text = serde_json::to_string(value).ok()?;
    (text.len() <= MAX_BYTES).then_some(text)
}

fn version_is_one(value: &Value) -> bool {
    match value {
        Value::Number(number) => number.as_u64() == Some(1),
        _ => false,
    }
}

fn sdp_mid(object: &Map<String, Value>) -> String {
    match object.get("sdpMid") {
        Some(Value::String(text)) => text.clone(),
        _ => String::new(),
    }
}

fn line_index(value: &Value) -> Option<i32> {
    let number = value.as_number()?;
    let index = integer_number(number)?;
    i32::try_from(index).ok()
}

fn integer_number(number: &Number) -> Option<u64> {
    if let Some(value) = number.as_u64() {
        return Some(value);
    }
    let value = number.as_f64()?;
    if !value.is_finite() || value < 0.0 || value.fract() != 0.0 {
        return None;
    }
    Some(value as u64)
}

fn is_uuid(text: &str) -> bool {
    let bytes = text.as_bytes();
    if bytes.len() != 36 {
        return false;
    }
    bytes.iter().enumerate().all(|(index, byte)| match index {
        8 | 13 | 18 | 23 => *byte == b'-',
        _ => byte.is_ascii_hexdigit(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIRST: &str = "11111111-1111-4111-8111-111111111111";
    const SECOND: &str = "22222222-2222-4222-8222-222222222222";

    fn session(text: &str) -> SessionId {
        SessionId::parse(text).unwrap()
    }

    #[test]
    fn drive_and_arm_topics_are_not_signaling() {
        for topic in [
            "hardware",
            "teleop",
            "cmd_vel",
            "autonomy",
            "safety",
            "webrtc",
            "webrtc/ice",
        ] {
            assert!(
                decode(
                    topic,
                    &format!(r#"{{"v":1,"action":"start","session":"{FIRST}"}}"#)
                )
                .is_none()
            );
            assert!(!is_signaling_topic(topic), "{topic}");
        }
        for topic in [
            "webrtc/request",
            "webrtc/offer",
            "webrtc/answer",
            "webrtc/ice/phone",
            "webrtc/ice/dashboard",
            "webrtc/state",
        ] {
            assert!(is_signaling_topic(topic), "{topic}");
        }
    }

    #[test]
    fn version_session_and_size_rules() {
        assert!(
            decode(
                "webrtc/request",
                &format!(r#"{{"v":2,"action":"start","session":"{FIRST}"}}"#)
            )
            .is_none()
        );
        assert!(
            decode(
                "webrtc/request",
                &format!(r#"{{"v":true,"action":"start","session":"{FIRST}"}}"#)
            )
            .is_none()
        );
        assert!(
            decode(
                "webrtc/request",
                &format!(r#"{{"v":"1","action":"start","session":"{FIRST}"}}"#)
            )
            .is_none()
        );
        assert!(
            decode(
                "webrtc/request",
                r#"{"v":1,"action":"start","session":"not-a-uuid"}"#
            )
            .is_none()
        );
        assert!(
            decode(
                "webrtc/request",
                r#"{"v":1,"action":"arm","session":"not"}"#
            )
            .is_none()
        );
        let huge = "{".repeat(MAX_BYTES + 1);
        assert!(decode("webrtc/request", &huge).is_none());
        assert!(encode_offer(&session(FIRST), &"a".repeat(MAX_BYTES)).is_none());
        assert!(CallState::parse("armed").is_none());
        assert!(encode_state(&session(SECOND), CallState::Connected).is_some());
    }

    #[test]
    fn unknown_fields_are_ignored_and_spelling_is_kept() {
        let start = decode(
            "webrtc/request",
            &format!(r#"{{"v":1,"action":"start","session":"{FIRST}","note":"optional"}}"#),
        )
        .unwrap();
        assert_eq!(start.session.as_str(), FIRST);
        assert_eq!(start.body, Body::Request(RequestAction::Start));
        let upper = FIRST.to_ascii_uppercase();
        let answer = decode(
            "webrtc/answer",
            &format!(r#"{{"v":1,"session":"{upper}","sdp":"v=0\r\n"}}"#),
        )
        .unwrap();
        assert!(start.session.matches(answer.session.as_str()));
        assert_eq!(answer.session.as_str(), &upper);
        let offer = encode_offer(&start.session, "v=0\r\n").unwrap();
        let object: Value = serde_json::from_str(&offer).unwrap();
        let mut keys: Vec<_> = object.as_object().unwrap().keys().cloned().collect();
        keys.sort();
        assert_eq!(keys, ["sdp", "session", "v"]);
        assert_eq!(object["session"], FIRST);
    }

    #[test]
    fn ice_mid_index_and_end_of_candidates() {
        let ice = decode(
            "webrtc/ice/dashboard",
            &format!(
                r#"{{"v":1,"session":"{FIRST}","candidate":"candidate:1","sdpMid":"0","sdpMLineIndex":0,"extra":true}}"#
            ),
        )
        .unwrap();
        assert_eq!(
            ice.body,
            Body::Ice {
                candidate: "candidate:1".into(),
                sdp_mid: "0".into(),
                sdp_m_line_index: 0,
            }
        );
        let end = decode(
            "webrtc/ice/dashboard",
            &format!(
                r#"{{"v":1,"session":"{FIRST}","candidate":"","sdpMid":null,"sdpMLineIndex":0}}"#
            ),
        )
        .unwrap();
        assert!(end.body.is_end_of_candidates());
        assert!(matches!(end.body, Body::Ice { sdp_mid, .. } if sdp_mid.is_empty()));
        assert!(decode(
            "webrtc/ice/dashboard",
            &format!(r#"{{"v":1,"session":"{FIRST}","candidate":"x","sdpMid":"0","sdpMLineIndex":1.5}}"#),
        )
        .is_none());
        assert!(
            decode(
                "webrtc/ice/dashboard",
                &format!(
                    r#"{{"v":1,"session":"{FIRST}","candidate":"x","sdpMLineIndex":2147483648}}"#
                ),
            )
            .is_none()
        );
        assert!(
            decode(
                "webrtc/ice/phone",
                &format!(r#"{{"v":1,"session":"{FIRST}","candidate":"x","sdpMLineIndex":true}}"#),
            )
            .is_none()
        );
        let missing_mid = decode(
            "webrtc/ice/phone",
            &format!(r#"{{"v":1,"session":"{FIRST}","candidate":"x","sdpMLineIndex":1}}"#),
        )
        .unwrap();
        assert!(
            matches!(missing_mid.body, Body::Ice { sdp_mid, sdp_m_line_index: 1, .. } if sdp_mid.is_empty())
        );
    }

    #[test]
    fn topic_keys_use_the_fleet_prefix_and_rover_id() {
        assert_eq!(
            topic_key(DEFAULT_PREFIX, 3, Topic::Offer),
            "terra/phone/3/webrtc/offer"
        );
        assert_eq!(
            topic_key("/terra/phone/", 0, Topic::Request),
            "terra/phone/0/webrtc/request"
        );
        for topic in [
            Topic::Request,
            Topic::Offer,
            Topic::Answer,
            Topic::IcePhone,
            Topic::IceDashboard,
            Topic::State,
        ] {
            let key = topic_key("terra/phone", 1, topic);
            assert!(key.contains("/webrtc/"), "{key}");
            assert!(!key.contains("teleop"), "{key}");
            assert!(!key.contains("hardware"), "{key}");
        }
    }
}
