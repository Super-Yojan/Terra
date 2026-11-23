//! One FPV call at a time, for the phone (offerer) or the dashboard (answerer).
//!
//! Effects are video signaling and media cues. None of them is a velocity,
//! arm, or disarm command.

use std::time::Duration;

use crate::{
    Body, CallState, Message, RequestAction, SessionId, Topic, decode, encode_answer, encode_ice,
    encode_offer, encode_request, encode_state,
};

/// How long the offer may take to appear after `start`.
pub const OFFER_TIMEOUT: Duration = Duration::from_secs(15);
/// How long the answer may take after the offer.
pub const ANSWER_TIMEOUT: Duration = Duration::from_secs(20);
/// How long ICE may take to connect after the answer is available.
pub const CONNECT_TIMEOUT: Duration = Duration::from_secs(30);

/// Which peer owns this machine. The phone creates the offer. The dashboard creates the answer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Role {
    Phone,
    Dashboard,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Phase {
    AwaitingOffer,
    AwaitingAnswer,
    Connecting,
    Connected,
}

/// What the caller should do with one accepted transition.
/// `Publish` carries a signaling sample. The other variants are media cues.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Effect {
    Publish {
        topic: Topic,
        payload: String,
    },
    /// Phone only. Open the send-only camera call and create an offer.
    CreateOffer {
        session: String,
    },
    /// Dashboard only. The phone's offer arrived; create an answer.
    AcceptOffer {
        session: String,
        sdp: String,
    },
    /// Phone only. Apply the dashboard's answer.
    ApplyAnswer {
        session: String,
        sdp: String,
    },
    AddIce {
        session: String,
        candidate: String,
        sdp_mid: String,
        sdp_m_line_index: i32,
    },
    EndOfCandidates {
        session: String,
    },
    StopMedia {
        session: String,
    },
    /// `state` is the connection word. `detail` is empty or a stable failure code.
    Show {
        session: String,
        state: CallState,
        detail: &'static str,
    },
}

#[derive(Clone, Debug)]
struct Active {
    session: SessionId,
    phase: Phase,
    failed: bool,
    progress_ms: u64,
    last_published: Option<CallState>,
}

/// Role-aware session. Stale ids are ignored. A failed call can be started again.
#[derive(Clone, Debug)]
pub struct SignalingSession {
    role: Role,
    active: Option<Active>,
}

impl SignalingSession {
    pub fn phone() -> Self {
        Self::new(Role::Phone)
    }

    pub fn dashboard() -> Self {
        Self::new(Role::Dashboard)
    }

    pub fn new(role: Role) -> Self {
        Self { role, active: None }
    }

    pub fn role(&self) -> Role {
        self.role
    }

    pub fn has_session(&self) -> bool {
        self.active.is_some()
    }

    pub fn is_current(&self, session: &str) -> bool {
        self.active
            .as_ref()
            .is_some_and(|active| active.session.matches(session))
    }

    pub fn is_failed(&self) -> bool {
        self.active.as_ref().is_some_and(|active| active.failed)
    }

    /// Inbound sample. Invalid JSON and the wrong direction produce no effects.
    pub fn receive(&mut self, topic: &str, payload: &str, now_ms: u64) -> Vec<Effect> {
        let Some(message) = decode(topic, payload) else {
            return Vec::new();
        };
        if !self.accepts(message.topic) {
            return Vec::new();
        }
        self.apply(message, now_ms)
    }

    /// Dashboard only. The phone starts when a `request` arrives.
    pub fn start(&mut self, session: &str, now_ms: u64) -> Vec<Effect> {
        if self.role != Role::Dashboard {
            return Vec::new();
        }
        let Some(session) = SessionId::parse(session) else {
            return Vec::new();
        };
        self.begin(session, now_ms)
    }

    /// End the call. `publish` writes `state: closed` from the phone, or `request: stop` from the dashboard.
    pub fn stop(&mut self, publish: bool) -> Vec<Effect> {
        self.finish(publish)
    }

    /// Phone: the local offer. Dashboard: the local answer.
    pub fn note_local_description(&mut self, session: &str, sdp: &str, now_ms: u64) -> Vec<Effect> {
        if sdp.is_empty() {
            return self.fail_if_current(session, "empty_sdp", true);
        }
        let Some(active) = self.active.as_ref() else {
            return Vec::new();
        };
        if active.failed || !active.session.matches(session) {
            return Vec::new();
        }
        let stored = active.session.clone();
        let phase = active.phase;
        match self.role {
            Role::Phone if phase == Phase::AwaitingOffer => {
                let Some(payload) = encode_offer(&stored, sdp) else {
                    return self.fail("offer_too_large", true);
                };
                if let Some(active) = self.active.as_mut() {
                    active.phase = Phase::AwaitingAnswer;
                    active.progress_ms = now_ms;
                }
                vec![Effect::Publish {
                    topic: Topic::Offer,
                    payload,
                }]
            }
            Role::Dashboard if phase == Phase::AwaitingAnswer => {
                let Some(payload) = encode_answer(&stored, sdp) else {
                    return self.fail("answer_too_large", true);
                };
                if let Some(active) = self.active.as_mut() {
                    active.phase = Phase::Connecting;
                    active.progress_ms = now_ms;
                }
                vec![Effect::Publish {
                    topic: Topic::Answer,
                    payload,
                }]
            }
            _ => Vec::new(),
        }
    }

    /// Local trickle candidate. An empty candidate is the end-of-candidates sample.
    pub fn note_local_ice(
        &mut self,
        session: &str,
        candidate: &str,
        sdp_mid: &str,
        sdp_m_line_index: i32,
        _now_ms: u64,
    ) -> Vec<Effect> {
        let Some(active) = self.active.as_ref() else {
            return Vec::new();
        };
        if active.failed || !active.session.matches(session) || sdp_m_line_index < 0 {
            return Vec::new();
        }
        let stored = active.session.clone();
        let topic = match self.role {
            Role::Phone => Topic::IcePhone,
            Role::Dashboard => Topic::IceDashboard,
        };
        encode_ice(&stored, candidate, sdp_mid, sdp_m_line_index)
            .map(|payload| vec![Effect::Publish { topic, payload }])
            .unwrap_or_default()
    }

    /// Local media state from the peer connection. This is not a wire sample.
    pub fn note_media(&mut self, session: &str, state: CallState, now_ms: u64) -> Vec<Effect> {
        self.apply_state(session, state, now_ms, true)
    }

    /// Fail a call that has sat in one phase longer than its timeout.
    pub fn tick(&mut self, now_ms: u64) -> Vec<Effect> {
        let Some(active) = self.active.as_ref() else {
            return Vec::new();
        };
        if active.failed {
            return Vec::new();
        }
        let limit = match active.phase {
            Phase::AwaitingOffer => OFFER_TIMEOUT,
            Phase::AwaitingAnswer => ANSWER_TIMEOUT,
            Phase::Connecting => CONNECT_TIMEOUT,
            Phase::Connected => return Vec::new(),
        };
        if now_ms.saturating_sub(active.progress_ms) >= limit.as_millis() as u64 {
            self.fail("timed_out", true)
        } else {
            Vec::new()
        }
    }

    fn accepts(&self, topic: Topic) -> bool {
        match self.role {
            Role::Phone => topic.is_phone_inbound(),
            Role::Dashboard => topic.is_dashboard_inbound(),
        }
    }

    fn apply(&mut self, message: Message, now_ms: u64) -> Vec<Effect> {
        match (self.role, message.topic, message.body) {
            (Role::Phone, Topic::Request, Body::Request(RequestAction::Start)) => {
                self.begin(message.session, now_ms)
            }
            (Role::Phone, Topic::Request, Body::Request(RequestAction::Stop)) => {
                if self.is_current(message.session.as_str()) {
                    self.finish(true)
                } else {
                    Vec::new()
                }
            }
            (Role::Phone, Topic::Answer, Body::Sdp(sdp)) => {
                self.apply_answer(message.session.as_str(), sdp, now_ms)
            }
            (Role::Dashboard, Topic::Offer, Body::Sdp(sdp)) => {
                self.apply_remote_offer(message.session.as_str(), sdp, now_ms)
            }
            (
                _,
                Topic::IcePhone | Topic::IceDashboard,
                Body::Ice {
                    candidate,
                    sdp_mid,
                    sdp_m_line_index,
                },
            ) => self.apply_remote_ice(
                message.session.as_str(),
                candidate,
                sdp_mid,
                sdp_m_line_index,
            ),
            (Role::Dashboard, Topic::State, Body::State(state)) => {
                self.apply_state(message.session.as_str(), state, now_ms, false)
            }
            _ => Vec::new(),
        }
    }

    fn begin(&mut self, session: SessionId, now_ms: u64) -> Vec<Effect> {
        if let Some(active) = &self.active {
            if active.session.matches(session.as_str()) && !active.failed {
                return Vec::new();
            }
        }
        let mut effects = Vec::new();
        if let Some(previous) = self.active.take() {
            effects.push(Effect::StopMedia {
                session: previous.session.as_str().to_string(),
            });
            if self.role == Role::Phone && !previous.session.matches(session.as_str()) {
                if let Some(payload) = encode_state(&previous.session, CallState::Closed) {
                    effects.push(Effect::Publish {
                        topic: Topic::State,
                        payload,
                    });
                }
            }
        }
        self.active = Some(Active {
            session: session.clone(),
            phase: Phase::AwaitingOffer,
            failed: false,
            progress_ms: now_ms,
            last_published: None,
        });
        match self.role {
            Role::Phone => {
                self.push_state(&mut effects, CallState::Connecting);
                effects.push(Effect::Show {
                    session: session.as_str().to_string(),
                    state: CallState::Connecting,
                    detail: "",
                });
                effects.push(Effect::CreateOffer {
                    session: session.as_str().to_string(),
                });
            }
            Role::Dashboard => {
                if let Some(payload) = encode_request(&session, RequestAction::Start) {
                    effects.push(Effect::Publish {
                        topic: Topic::Request,
                        payload,
                    });
                }
                effects.push(Effect::Show {
                    session: session.as_str().to_string(),
                    state: CallState::Connecting,
                    detail: "",
                });
            }
        }
        effects
    }

    fn apply_answer(&mut self, session: &str, sdp: String, now_ms: u64) -> Vec<Effect> {
        let Some(active) = self.active.as_mut() else {
            return Vec::new();
        };
        if active.failed || !active.session.matches(session) {
            return Vec::new();
        }
        if !matches!(active.phase, Phase::AwaitingAnswer | Phase::Connecting) {
            return Vec::new();
        }
        active.phase = Phase::Connecting;
        active.progress_ms = now_ms;
        vec![Effect::ApplyAnswer {
            session: active.session.as_str().to_string(),
            sdp,
        }]
    }

    fn apply_remote_offer(&mut self, session: &str, sdp: String, now_ms: u64) -> Vec<Effect> {
        let Some(active) = self.active.as_mut() else {
            return Vec::new();
        };
        if active.failed || !active.session.matches(session) || active.phase != Phase::AwaitingOffer
        {
            return Vec::new();
        }
        active.phase = Phase::AwaitingAnswer;
        active.progress_ms = now_ms;
        vec![Effect::AcceptOffer {
            session: active.session.as_str().to_string(),
            sdp,
        }]
    }

    fn apply_remote_ice(
        &mut self,
        session: &str,
        candidate: String,
        sdp_mid: String,
        sdp_m_line_index: i32,
    ) -> Vec<Effect> {
        let Some(active) = self.active.as_ref() else {
            return Vec::new();
        };
        if active.failed || !active.session.matches(session) {
            return Vec::new();
        }
        let session = active.session.as_str().to_string();
        if candidate.is_empty() {
            vec![Effect::EndOfCandidates { session }]
        } else {
            vec![Effect::AddIce {
                session,
                candidate,
                sdp_mid,
                sdp_m_line_index,
            }]
        }
    }

    fn apply_state(
        &mut self,
        session: &str,
        state: CallState,
        now_ms: u64,
        local: bool,
    ) -> Vec<Effect> {
        if !self.is_current(session) {
            return Vec::new();
        }
        match state {
            CallState::Connecting => {
                if self.is_failed() {
                    return Vec::new();
                }
                if let Some(active) = self.active.as_mut() {
                    if active.phase == Phase::Connected {
                        active.phase = Phase::Connecting;
                        active.progress_ms = now_ms;
                    }
                }
                let stored = self.spelling();
                let mut effects = Vec::new();
                if local {
                    self.push_state(&mut effects, CallState::Connecting);
                }
                effects.push(Effect::Show {
                    session: stored,
                    state: CallState::Connecting,
                    detail: "",
                });
                effects
            }
            CallState::Connected => {
                let stored = {
                    let Some(active) = self.active.as_mut() else {
                        return Vec::new();
                    };
                    if active.failed {
                        return Vec::new();
                    }
                    active.phase = Phase::Connected;
                    active.progress_ms = now_ms;
                    active.session.as_str().to_string()
                };
                let mut effects = Vec::new();
                if local {
                    self.push_state(&mut effects, CallState::Connected);
                }
                effects.push(Effect::Show {
                    session: stored,
                    state: CallState::Connected,
                    detail: "",
                });
                effects
            }
            CallState::Failed => {
                self.fail(if local { "media_failed" } else { "peer_failed" }, local)
            }
            CallState::Closed => self.finish(local),
        }
    }

    fn finish(&mut self, publish: bool) -> Vec<Effect> {
        let Some(active) = self.active.take() else {
            return Vec::new();
        };
        let session = active.session.as_str().to_string();
        let mut effects = vec![Effect::StopMedia {
            session: session.clone(),
        }];
        if publish {
            let payload = match self.role {
                Role::Phone => encode_state(&active.session, CallState::Closed)
                    .map(|payload| (Topic::State, payload)),
                Role::Dashboard => encode_request(&active.session, RequestAction::Stop)
                    .map(|payload| (Topic::Request, payload)),
            };
            if let Some((topic, payload)) = payload {
                effects.push(Effect::Publish { topic, payload });
            }
        }
        effects.push(Effect::Show {
            session,
            state: CallState::Closed,
            detail: "",
        });
        effects
    }

    fn fail(&mut self, detail: &'static str, inform_peer: bool) -> Vec<Effect> {
        let session = {
            let Some(active) = self.active.as_mut() else {
                return Vec::new();
            };
            if active.failed {
                return Vec::new();
            }
            active.failed = true;
            active.session.clone()
        };
        let mut effects = vec![Effect::StopMedia {
            session: session.as_str().to_string(),
        }];
        if inform_peer {
            let published = match self.role {
                Role::Phone => {
                    encode_state(&session, CallState::Failed).map(|payload| (Topic::State, payload))
                }
                Role::Dashboard => encode_request(&session, RequestAction::Stop)
                    .map(|payload| (Topic::Request, payload)),
            };
            if let Some((topic, payload)) = published {
                if let Some(active) = self.active.as_mut() {
                    if self.role == Role::Phone {
                        active.last_published = Some(CallState::Failed);
                    }
                }
                effects.push(Effect::Publish { topic, payload });
            }
        }
        effects.push(Effect::Show {
            session: session.as_str().to_string(),
            state: CallState::Failed,
            detail,
        });
        effects
    }

    fn fail_if_current(
        &mut self,
        session: &str,
        detail: &'static str,
        inform_peer: bool,
    ) -> Vec<Effect> {
        if self.is_current(session) && !self.is_failed() {
            self.fail(detail, inform_peer)
        } else {
            Vec::new()
        }
    }

    fn push_state(&mut self, effects: &mut Vec<Effect>, state: CallState) {
        let Some(active) = self.active.as_mut() else {
            return;
        };
        if active.last_published == Some(state) {
            return;
        }
        let Some(payload) = encode_state(&active.session, state) else {
            return;
        };
        active.last_published = Some(state);
        effects.push(Effect::Publish {
            topic: Topic::State,
            payload,
        });
    }

    fn spelling(&self) -> String {
        self.active
            .as_ref()
            .map(|active| active.session.as_str().to_string())
            .unwrap_or_default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIRST: &str = "11111111-1111-4111-8111-111111111111";
    const SECOND: &str = "22222222-2222-4222-8222-222222222222";

    fn kinds(effects: &[Effect]) -> Vec<&'static str> {
        effects
            .iter()
            .map(|effect| match effect {
                Effect::Publish { topic, .. } => topic.suffix(),
                Effect::CreateOffer { .. } => "create_offer",
                Effect::AcceptOffer { .. } => "accept_offer",
                Effect::ApplyAnswer { .. } => "apply_answer",
                Effect::AddIce { .. } => "add_ice",
                Effect::EndOfCandidates { .. } => "end",
                Effect::StopMedia { .. } => "stop_media",
                Effect::Show { state, .. } => state.as_str(),
            })
            .collect()
    }

    fn assert_no_drive(effects: &[Effect]) {
        for effect in effects {
            if let Effect::Publish { topic, payload } = effect {
                assert!(topic.suffix().starts_with("webrtc/"), "{}", topic.suffix());
                assert!(!payload.contains("\"linear\""));
                assert!(!payload.contains("\"action\":\"arm\""));
                assert!(!payload.contains("\"action\":\"disarm\""));
            }
        }
    }

    #[test]
    fn phone_start_is_idempotent_until_failure_and_echoes_spelling() {
        let mut phone = SignalingSession::phone();
        let upper = FIRST.to_ascii_uppercase();
        let start = format!(r#"{{"v":1,"action":"start","session":"{upper}","note":"optional"}}"#);
        let effects = phone.receive("webrtc/request", &start, 0);
        assert_no_drive(&effects);
        assert!(kinds(&effects).contains(&"create_offer"));
        assert!(phone.receive("webrtc/request", &start, 1).is_empty());
        let offer = phone.note_local_description(FIRST, "v=0\r\n", 2);
        match &offer[0] {
            Effect::Publish { payload, topic } => {
                assert_eq!(*topic, Topic::Offer);
                assert!(payload.contains(&upper), "{payload}");
            }
            other => panic!("expected offer, got {other:?}"),
        }
        let stale = format!(r#"{{"v":1,"session":"{SECOND}","sdp":"v=0"}}"#);
        assert!(phone.receive("webrtc/answer", &stale, 3).is_empty());
        let answer = format!(r#"{{"v":1,"session":"{FIRST}","sdp":"v=0\r\n"}}"#);
        assert!(kinds(&phone.receive("webrtc/answer", &answer, 4)).contains(&"apply_answer"));
        let ice = format!(
            r#"{{"v":1,"session":"{FIRST}","candidate":"candidate:1","sdpMid":"0","sdpMLineIndex":0,"extra":true}}"#
        );
        assert!(kinds(&phone.receive("webrtc/ice/dashboard", &ice, 5)).contains(&"add_ice"));
        let end = format!(
            r#"{{"v":1,"session":"{FIRST}","candidate":"","sdpMid":null,"sdpMLineIndex":0}}"#
        );
        assert_eq!(
            kinds(&phone.receive("webrtc/ice/dashboard", &end, 6)),
            ["end"]
        );
    }

    #[test]
    fn phone_replaces_a_session_ignores_stale_stop_and_restarts_after_failure() {
        let mut phone = SignalingSession::phone();
        phone.receive(
            "webrtc/request",
            &format!(r#"{{"v":1,"action":"start","session":"{FIRST}"}}"#),
            0,
        );
        let replaced = phone.receive(
            "webrtc/request",
            &format!(r#"{{"v":1,"action":"start","session":"{SECOND}"}}"#),
            10,
        );
        assert!(replaced.iter().any(|effect| matches!(effect, Effect::Publish { payload, .. } if payload.contains(FIRST) && payload.contains("closed"))));
        assert!(kinds(&replaced).contains(&"create_offer"));
        assert!(
            phone
                .receive(
                    "webrtc/request",
                    &format!(r#"{{"v":1,"action":"stop","session":"{FIRST}"}}"#),
                    11,
                )
                .is_empty()
        );
        assert!(
            phone
                .receive(
                    "webrtc/answer",
                    &format!(r#"{{"v":1,"session":"{FIRST}","sdp":"old"}}"#),
                    12,
                )
                .is_empty()
        );
        let stopped = phone.receive(
            "webrtc/request",
            &format!(r#"{{"v":1,"action":"stop","session":"{SECOND}"}}"#),
            13,
        );
        assert!(kinds(&stopped).contains(&"webrtc/state"));
        assert!(!phone.has_session());
        assert!(
            phone
                .receive(
                    "webrtc/answer",
                    &format!(r#"{{"v":1,"session":"{SECOND}","sdp":"late"}}"#),
                    14,
                )
                .is_empty()
        );

        phone.receive(
            "webrtc/request",
            &format!(r#"{{"v":1,"action":"start","session":"{FIRST}"}}"#),
            20,
        );
        phone.note_local_description(FIRST, "v=0", 21);
        let failed = phone.note_media(FIRST, CallState::Failed, 22);
        assert!(kinds(&failed).contains(&"failed"));
        assert!(phone.is_failed());
        assert!(phone.note_media(FIRST, CallState::Failed, 23).is_empty());
        assert!(
            phone
                .receive(
                    "webrtc/answer",
                    &format!(r#"{{"v":1,"session":"{FIRST}","sdp":"after-fail"}}"#),
                    24,
                )
                .is_empty()
        );
        let restarted = phone.receive(
            "webrtc/request",
            &format!(r#"{{"v":1,"action":"start","session":"{FIRST}"}}"#),
            25,
        );
        assert!(kinds(&restarted).contains(&"create_offer"));
        assert!(!restarted.iter().any(|effect| matches!(effect, Effect::Publish { payload, .. } if payload.contains("\"closed\""))));
        assert!(!phone.is_failed());
    }

    #[test]
    fn phone_timeout_fails_without_a_drive_topic_and_a_later_stop_still_matches() {
        let mut phone = SignalingSession::phone();
        phone.receive(
            "webrtc/request",
            &format!(r#"{{"v":1,"action":"start","session":"{FIRST}"}}"#),
            1_000,
        );
        assert!(phone.tick(1_000 + 14_000).is_empty());
        let timed = phone.tick(1_000 + OFFER_TIMEOUT.as_millis() as u64);
        assert!(kinds(&timed).contains(&"failed"));
        assert!(timed.iter().any(|effect| matches!(
            effect,
            Effect::Show {
                detail: "timed_out",
                ..
            }
        )));
        assert_no_drive(&timed);
        assert!(phone.is_failed() && phone.has_session());
        let stopped = phone.stop(true);
        assert!(
            kinds(&stopped)
                .iter()
                .any(|kind| kind.contains("closed") || *kind == "webrtc/state")
        );
        assert!(!phone.has_session());
    }

    #[test]
    fn dashboard_waits_for_the_offer_and_times_out_with_a_stop() {
        let mut dash = SignalingSession::dashboard();
        assert!(dash.start("not-a-uuid", 0).is_empty());
        assert!(SignalingSession::phone().start(FIRST, 0).is_empty());
        let started = dash.start(FIRST, 100);
        assert!(kinds(&started).contains(&"webrtc/request"));
        assert!(dash.start(FIRST, 101).is_empty());
        let offer = format!(r#"{{"v":1,"session":"{FIRST}","sdp":"v=0"}}"#);
        assert!(kinds(&dash.receive("webrtc/offer", &offer, 200)).contains(&"accept_offer"));
        let answer = dash.note_local_description(FIRST, "v=0\r\n", 300);
        assert!(kinds(&answer).contains(&"webrtc/answer"));
        assert!(
            dash.tick(300 + CONNECT_TIMEOUT.as_millis() as u64 - 1)
                .is_empty()
        );
        let timed = dash.tick(300 + CONNECT_TIMEOUT.as_millis() as u64);
        assert!(timed.iter().any(|effect| matches!(
            effect,
            Effect::Publish { topic: Topic::Request, payload } if payload.contains("\"stop\"")
        )));
        assert_no_drive(&timed);
        assert!(dash.is_failed());
        let again = dash.start(FIRST, 400);
        assert!(kinds(&again).contains(&"webrtc/request"));
        assert!(!dash.is_failed());
    }

    #[test]
    fn dashboard_follows_phone_state_and_rejects_a_stale_offer() {
        let mut dash = SignalingSession::dashboard();
        dash.start(FIRST, 0);
        dash.receive(
            "webrtc/offer",
            &format!(r#"{{"v":1,"session":"{SECOND}","sdp":"v=0"}}"#),
            1,
        );
        assert!(
            dash.receive(
                "webrtc/offer",
                &format!(r#"{{"v":1,"session":"{SECOND}","sdp":"v=0"}}"#),
                1
            )
            .is_empty()
        );
        dash.receive(
            "webrtc/offer",
            &format!(r#"{{"v":1,"session":"{FIRST}","sdp":"v=0"}}"#),
            2,
        );
        let connected = dash.receive(
            "webrtc/state",
            &format!(r#"{{"v":1,"session":"{FIRST}","state":"connected"}}"#),
            3,
        );
        assert!(kinds(&connected).contains(&"connected"));
        assert!(dash.tick(3 + CONNECT_TIMEOUT.as_millis() as u64).is_empty());
        let failed = dash.receive(
            "webrtc/state",
            &format!(r#"{{"v":1,"session":"{FIRST}","state":"failed"}}"#),
            4,
        );
        assert!(kinds(&failed).contains(&"failed"));
        assert!(failed.iter().all(|effect| !matches!(
            effect,
            Effect::Publish {
                topic: Topic::Request,
                ..
            }
        )));
        let closed = dash.receive(
            "webrtc/state",
            &format!(r#"{{"v":1,"session":"{FIRST}","state":"closed"}}"#),
            5,
        );
        assert!(kinds(&closed).contains(&"closed"));
        assert!(!dash.has_session());
    }
}
