//! WebRTC first-person-view signaling shared by TerraPhone and ARGOS.
//!
//! The crate owns the six JSON topics, the size cap, and the session rules.
//! It does not open a peer connection, encode video, or publish a drive command.
//! Zenoh stays in the transport crate. Camera capture stays in the phone and the
//! dashboard player.

mod machine;
mod message;

pub use machine::{ANSWER_TIMEOUT, CONNECT_TIMEOUT, Effect, OFFER_TIMEOUT, Role, SignalingSession};
pub use message::{
    Body, CallState, DEFAULT_PREFIX, MAX_BYTES, Message, RequestAction, SessionId, Topic, decode,
    encode_answer, encode_ice, encode_offer, encode_request, encode_state, is_signaling_topic,
    topic_key,
};
