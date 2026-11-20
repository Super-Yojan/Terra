//! Encodings.
//!
//! - Small messages are JSON objects whose first field is `version`, so they
//!   can be read with `z_sub`.
//! - `Depth` is a one-line JSON header (`version`, `t`, `sequence`, `count`),
//!   a `\n`, then `count × 3` little-endian `f32`s.
//!
//! A decoder rejects a version it doesn't know instead of guessing.
use std::fmt;

use serde::Serialize;
use serde::de::DeserializeOwned;

use crate::messages::*;
use crate::topics::Topic;

/// Wire format version, written first in every payload.
pub const WIRE_VERSION: u64 = 1;

pub trait Message: Sized + Clone + Send + Sync + 'static {
    const TOPIC: Topic;
    fn encode(&self) -> Vec<u8>;
    fn decode(bytes: &[u8]) -> Result<Self, DecodeError>;
}

/// A rover → base message: published by `RoverLink`, read by `BaseLink`.
pub trait Uplink: Message {}

/// A base → rover command: sent by `BaseLink`, read by `RoverLink`.
pub trait Downlink: Message {}

/// A payload that couldn't be decoded. Dropped and counted by the links;
/// it never reaches the caller.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum DecodeError {
    /// No `version` field.
    MissingVersion,
    UnknownVersion(u64),
    Malformed(String),
}

impl fmt::Display for DecodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingVersion => f.write_str("payload has no version"),
            Self::UnknownVersion(v) => write!(f, "unknown payload version {v} (expected {WIRE_VERSION})"),
            Self::Malformed(why) => write!(f, "malformed payload: {why}"),
        }
    }
}

impl std::error::Error for DecodeError {}

fn malformed(e: impl fmt::Display) -> DecodeError {
    DecodeError::Malformed(e.to_string())
}

fn encode_json<T: Serialize>(body: &T) -> Vec<u8> {
    #[derive(Serialize)]
    struct Envelope<'a, T> {
        version: u64,
        #[serde(flatten)]
        body: &'a T,
    }
    // Only plain structs with string-keyed maps: serialization can't fail.
    serde_json::to_vec(&Envelope { version: WIRE_VERSION, body }).expect("message serializes to JSON")
}

fn decode_json<T: DeserializeOwned>(bytes: &[u8]) -> Result<T, DecodeError> {
    let mut value: serde_json::Value = serde_json::from_slice(bytes).map_err(malformed)?;
    let object = value.as_object_mut().ok_or_else(|| malformed("not a JSON object"))?;
    check_version(object.remove("version"))?;
    serde_json::from_value(value).map_err(malformed)
}

fn check_version(version: Option<serde_json::Value>) -> Result<(), DecodeError> {
    let version = version.ok_or(DecodeError::MissingVersion)?;
    let version = version.as_u64().ok_or_else(|| malformed("version is not an integer"))?;
    if version != WIRE_VERSION {
        return Err(DecodeError::UnknownVersion(version));
    }
    Ok(())
}

macro_rules! json_message {
    ($($ty:ty => $topic:ident, $dir:ident;)*) => {$(
        impl Message for $ty {
            const TOPIC: Topic = Topic::$topic;
            fn encode(&self) -> Vec<u8> {
                encode_json(self)
            }
            fn decode(bytes: &[u8]) -> Result<Self, DecodeError> {
                decode_json(bytes)
            }
        }
        impl $dir for $ty {}
    )*};
}

json_message! {
    Capabilities => Capabilities, Uplink;
    TaskStatus => TaskStatus, Uplink;
    Pose => Pose, Uplink;
    Status => Status, Uplink;
    RemoteState => RemoteState, Uplink;
    Task => Task, Downlink;
    TaskCommand => TaskCommand, Downlink;
    Teleop => Teleop, Downlink;
    RemoteCmd => RemoteCmd, Downlink;
    Estop => Estop, Downlink;
}

#[derive(Serialize, serde::Deserialize)]
struct DepthHeader {
    t: f64,
    sequence: u64,
    count: usize,
}

impl Message for Depth {
    const TOPIC: Topic = Topic::Depth;

    fn encode(&self) -> Vec<u8> {
        let header = DepthHeader { t: self.t, sequence: self.sequence, count: self.points.len() };
        let mut out = encode_json(&header);
        out.reserve(1 + self.points.len() * 12);
        out.push(b'\n');
        for point in &self.points {
            for c in point {
                out.extend_from_slice(&c.to_le_bytes());
            }
        }
        out
    }

    fn decode(bytes: &[u8]) -> Result<Self, DecodeError> {
        let newline = bytes.iter().position(|&b| b == b'\n').ok_or_else(|| malformed("no depth header line"))?;
        let header: DepthHeader = decode_json(&bytes[..newline])?;
        let body = &bytes[newline + 1..];
        if Some(body.len()) != header.count.checked_mul(12) {
            return Err(malformed(format!("depth: {} bytes for {} points", body.len(), header.count)));
        }
        let points = body
            .as_chunks::<12>()
            .0
            .iter()
            .map(|p| {
                let f = |i: usize| f32::from_le_bytes([p[i], p[i + 1], p[i + 2], p[i + 3]]);
                [f(0), f(4), f(8)]
            })
            .collect();
        Ok(Self { t: header.t, sequence: header.sequence, points })
    }
}
impl Uplink for Depth {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::params::{ParamKind, ParamSpec, ParamValue, Params};
    use crate::topics::Direction;

    fn round_trip<M: Message + PartialEq + fmt::Debug>(msg: M) {
        let bytes = msg.encode();
        assert_eq!(M::decode(&bytes).unwrap(), msg);
    }

    fn uplink<M: Uplink>() -> Topic {
        M::TOPIC
    }
    fn downlink<M: Downlink>() -> Topic {
        M::TOPIC
    }

    #[test]
    fn directions_match_topics() {
        for t in [
            uplink::<Capabilities>(),
            uplink::<TaskStatus>(),
            uplink::<Pose>(),
            uplink::<Status>(),
            uplink::<Depth>(),
            uplink::<RemoteState>(),
        ] {
            assert_eq!(t.direction(), Direction::RoverToBase, "{t}");
        }
        for t in [
            downlink::<Task>(),
            downlink::<TaskCommand>(),
            downlink::<Teleop>(),
            downlink::<RemoteCmd>(),
            downlink::<Estop>(),
        ] {
            assert_eq!(t.direction(), Direction::BaseToRover, "{t}");
        }
    }

    fn params() -> Params {
        [("horizon".to_string(), ParamValue::Float(2.0)), ("mode".to_string(), ParamValue::Choice("a".into()))]
            .into_iter()
            .collect()
    }

    #[test]
    fn json_round_trips() {
        round_trip(Capabilities {
            t: 1.5,
            controllers: vec![StageOption {
                id: "dwa".into(),
                version: "1".into(),
                description: "Dynamic window".into(),
                params: vec![ParamSpec {
                    name: "mode".into(),
                    kind: ParamKind::Choice(vec!["a".into(), "b".into()]),
                    default: ParamValue::Choice("a".into()),
                    min: None,
                    max: None,
                    unit: Some("s".into()),
                    help: "h".into(),
                }],
                goals: vec![GoalKind::Local, GoalKind::Gps],
            }],
            planners: vec![],
            default_controller: "dwa".into(),
            software: "v0.1-3-gabc".into(),
        });
        for state in [
            TaskState::Idle,
            TaskState::Running { distance_to_goal: 3.0, elapsed: 1.0 },
            TaskState::Reached,
            TaskState::Paused,
            TaskState::NeedsAttention { reason: Attention::ControllerError("boom".into()), since: 2.0 },
            TaskState::NeedsAttention { reason: Attention::RoverRestarted, since: 0.0 },
        ] {
            round_trip(TaskStatus {
                t: 9.0,
                task_id: Some(4),
                state,
                controller: "dwa".into(),
                controller_version: "1".into(),
                params: params(),
                goal: Goal::Pose { x: 1.0, y: -2.0, yaw: None, tolerance: 0.3 },
                timeout: Some(30.0),
                last_rejected: Some(Rejection { task_id: 5, reason: "no GPS anchor".into(), t: 8.0 }),
            });
        }
        round_trip(Pose { t: 0.1, x: 1.0, y: 2.0, yaw: 0.5, tracked: true });
        round_trip(Status {
            t: 0.0,
            estopped: false,
            link_ok: true,
            detail: "ok".into(),
            gps_anchor: Some(GpsAnchor { latitude: 51.5, longitude: -0.1, heading: 1.0, accuracy: 2.5 }),
        });
        round_trip(RemoteState {
            t: 0.0,
            pose: Pose2 { x: 0.0, y: 0.0, yaw: 0.0 },
            twist: Twist { v: 0.5, w: 0.1 },
            path: vec![Pose2 { x: 1.0, y: 1.0, yaw: 0.2 }],
            obstacles: vec![[2.0, 3.0]],
        });
        for goal in [
            Goal::Stop,
            Goal::Pose { x: 1.0, y: 2.0, yaw: Some(0.3), tolerance: 0.2 },
            Goal::Gps { latitude: 1.0, longitude: 2.0, heading: None, tolerance: 3.0 },
        ] {
            round_trip(Task {
                id: 7,
                goal,
                controller: "dwa".into(),
                params: params(),
                planner: None,
                planner_params: Params::new(),
            });
        }
        round_trip(TaskCommand { task_id: 7, action: TaskAction::Pause });
        round_trip(Teleop { t: 1.0, v: 0.5, w: -0.2 });
        round_trip(RemoteCmd(Teleop { t: 1.0, v: 0.5, w: -0.2 }));
        round_trip(Estop { t: 3.0, engaged: true });
    }

    #[test]
    fn json_is_readable() {
        let text = String::from_utf8(TaskCommand { task_id: 7, action: TaskAction::Clear }.encode()).unwrap();
        assert_eq!(text, r#"{"version":1,"task_id":7,"action":"clear"}"#);
        let text = String::from_utf8(RemoteCmd(Teleop { t: 1.0, v: 0.5, w: 0.0 }).encode()).unwrap();
        assert_eq!(text, r#"{"version":1,"t":1.0,"v":0.5,"w":0.0}"#);
    }

    #[test]
    fn depth_round_trip() {
        let depth = Depth { t: 2.0, sequence: 9, points: vec![[1.0, 2.0, 3.0], [-0.5, 0.25, 8.0]] };
        let bytes = depth.encode();
        let newline = bytes.iter().position(|&b| b == b'\n').unwrap();
        assert_eq!(&bytes[..newline], br#"{"version":1,"t":2.0,"sequence":9,"count":2}"#);
        assert_eq!(bytes.len(), newline + 1 + 24);
        assert_eq!(Depth::decode(&bytes).unwrap(), depth);
        assert!(Depth::decode(&bytes[..bytes.len() - 1]).is_err());
        round_trip(Depth { t: 0.0, sequence: 0, points: vec![] });
    }

    #[test]
    fn rejects_versions_and_garbage() {
        assert_eq!(Pose::decode(br#"{"version":2,"t":0,"x":0,"y":0,"yaw":0,"tracked":true}"#), Err(DecodeError::UnknownVersion(2)));
        assert_eq!(Pose::decode(br#"{"t":0,"x":0,"y":0,"yaw":0,"tracked":true}"#), Err(DecodeError::MissingVersion));
        assert!(matches!(Pose::decode(b"nope"), Err(DecodeError::Malformed(_))));
        assert!(matches!(Pose::decode(br#"{"version":1,"t":0}"#), Err(DecodeError::Malformed(_))));
        assert!(matches!(Depth::decode(b"{\"version\":3,\"t\":0,\"sequence\":0,\"count\":0}\n"), Err(DecodeError::UnknownVersion(3))));
    }
}
