//! Encodings.
//!
//! - Small messages are JSON objects whose first field is `version`, so they
//!   can be read with `z_sub`.
//! - `Depth` is a one-line JSON header (`version`, `t`, `sequence`, `count`),
//!   a `\n`, then `count × 3` little-endian `f32`s.
//!
//! - `Occupancy` is the same shape: a header (`version`, `t`, `resolution`,
//!   `count`), a `\n`, then `count × 2` little-endian `f32`s.
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
    PlannedPath => PlannedPath, Uplink;
    RemoteState => RemoteState, Uplink;
    Task => Task, Downlink;
    TaskCommand => TaskCommand, Downlink;
    Teleop => Teleop, Downlink;
    RemoteCmd => RemoteCmd, Downlink;
    Estop => Estop, Downlink;
    GateCommand => Gate, Downlink;
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

#[derive(Serialize, serde::Deserialize)]
struct WorldMapHeader {
    t: f64,
    sequence: u64,
    #[serde(default)]
    points: usize,
    size: usize,
}

impl Message for WorldMap {
    const TOPIC: Topic = Topic::WorldMap;

    fn encode(&self) -> Vec<u8> {
        let header = WorldMapHeader {
            t: self.t,
            sequence: self.sequence,
            points: self.points.len(),
            size: self.data.len(),
        };
        let mut out = encode_json(&header);
        out.reserve(1 + self.points.len() * 12 + self.data.len());
        out.push(b'\n');
        for point in &self.points {
            for c in point {
                out.extend_from_slice(&c.to_le_bytes());
            }
        }
        out.extend_from_slice(&self.data);
        out
    }

    fn decode(bytes: &[u8]) -> Result<Self, DecodeError> {
        let newline = bytes
            .iter()
            .position(|&b| b == b'\n')
            .ok_or_else(|| malformed("no world_map header line"))?;
        let header: WorldMapHeader = decode_json(&bytes[..newline])?;
        let body = &bytes[newline + 1..];
        let pts_bytes = header
            .points
            .checked_mul(12)
            .ok_or_else(|| malformed("world_map points overflow"))?;
        if Some(body.len()) != pts_bytes.checked_add(header.size) {
            return Err(malformed(format!(
                "world_map: {} bytes for {} points + {} data bytes",
                body.len(),
                header.points,
                header.size
            )));
        }
        let points = body[..pts_bytes]
            .as_chunks::<12>()
            .0
            .iter()
            .map(|p| {
                let f = |i: usize| f32::from_le_bytes([p[i], p[i + 1], p[i + 2], p[i + 3]]);
                [f(0), f(4), f(8)]
            })
            .collect();
        let data = body[pts_bytes..].to_vec();
        Ok(Self { t: header.t, sequence: header.sequence, points, data })
    }
}
impl Uplink for WorldMap {}

#[derive(Serialize, serde::Deserialize)]
struct MeshChunkHeader {
    t: f64,
    id: String,
    vertices: usize,
    faces: usize,
    #[serde(default)]
    classes: usize,
    #[serde(default)]
    colors: usize,
}

impl Message for MeshChunk {
    const TOPIC: Topic = Topic::Mesh;

    fn encode(&self) -> Vec<u8> {
        let header = MeshChunkHeader {
            t: self.t,
            id: self.id.clone(),
            vertices: self.vertices.len(),
            faces: self.indices.len(),
            classes: self.classes.len(),
            colors: self.colors.len(),
        };
        let mut out = encode_json(&header);
        out.reserve(
            1 + self.vertices.len() * 12
                + self.indices.len() * 12
                + self.classes.len()
                + self.colors.len() * 3,
        );
        out.push(b'\n');
        for v in &self.vertices {
            for c in v {
                out.extend_from_slice(&c.to_le_bytes());
            }
        }
        for tri in &self.indices {
            for idx in tri {
                out.extend_from_slice(&idx.to_le_bytes());
            }
        }
        out.extend_from_slice(&self.classes);
        for rgb in &self.colors {
            out.extend_from_slice(rgb);
        }
        out
    }

    fn decode(bytes: &[u8]) -> Result<Self, DecodeError> {
        let newline = bytes
            .iter()
            .position(|&b| b == b'\n')
            .ok_or_else(|| malformed("no mesh header line"))?;
        let header: MeshChunkHeader = decode_json(&bytes[..newline])?;
        let body = &bytes[newline + 1..];
        let v_bytes = header
            .vertices
            .checked_mul(12)
            .ok_or_else(|| malformed("mesh vertices overflow"))?;
        let f_bytes = header
            .faces
            .checked_mul(12)
            .ok_or_else(|| malformed("mesh faces overflow"))?;
        let c_bytes = header
            .colors
            .checked_mul(3)
            .ok_or_else(|| malformed("mesh colors overflow"))?;
        let expected = v_bytes
            .checked_add(f_bytes)
            .and_then(|n| n.checked_add(header.classes))
            .and_then(|n| n.checked_add(c_bytes))
            .ok_or_else(|| malformed("mesh size overflow"))?;
        if body.len() != expected {
            return Err(malformed(format!(
                "mesh: {} bytes for {} verts + {} faces + {} classes + {} colors",
                body.len(),
                header.vertices,
                header.faces,
                header.classes,
                header.colors
            )));
        }
        let vertices = body[..v_bytes]
            .as_chunks::<12>()
            .0
            .iter()
            .map(|p| {
                let f = |i: usize| f32::from_le_bytes([p[i], p[i + 1], p[i + 2], p[i + 3]]);
                [f(0), f(4), f(8)]
            })
            .collect();
        let indices = body[v_bytes..v_bytes + f_bytes]
            .as_chunks::<12>()
            .0
            .iter()
            .map(|p| {
                let u = |i: usize| u32::from_le_bytes([p[i], p[i + 1], p[i + 2], p[i + 3]]);
                [u(0), u(4), u(8)]
            })
            .collect();
        let cls_start = v_bytes + f_bytes;
        let cls_end = cls_start + header.classes;
        let classes = body[cls_start..cls_end].to_vec();
        let colors = body[cls_end..]
            .as_chunks::<3>()
            .0
            .iter()
            .copied()
            .collect();
        Ok(Self { t: header.t, id: header.id, vertices, indices, classes, colors })
    }
}
impl Uplink for MeshChunk {}

/// Maximum WebRTC signaling payload size in bytes (matches `fpv_signaling::MAX_BYTES`).
const WEBRTC_MAX_BYTES: usize = 65_536;

fn decode_webrtc_json(bytes: &[u8]) -> Result<String, DecodeError> {
    if bytes.len() > WEBRTC_MAX_BYTES {
        return Err(malformed(format!(
            "webrtc payload too large: {} > {WEBRTC_MAX_BYTES}",
            bytes.len()
        )));
    }
    std::str::from_utf8(bytes)
        .map(str::to_owned)
        .map_err(|e| malformed(format!("webrtc payload not utf-8: {e}")))
}

macro_rules! webrtc_message {
    ($ty:ty, $topic:expr, $dir:ident) => {
        impl Message for $ty {
            const TOPIC: Topic = $topic;
            fn encode(&self) -> Vec<u8> {
                self.0.as_bytes().to_vec()
            }
            fn decode(bytes: &[u8]) -> Result<Self, DecodeError> {
                decode_webrtc_json(bytes).map(Self)
            }
        }
        impl $dir for $ty {}
    };
}

webrtc_message!(WebrtcOffer, Topic::WebrtcOffer, Uplink);
webrtc_message!(WebrtcIcePhone, Topic::WebrtcIcePhone, Uplink);
webrtc_message!(WebrtcState, Topic::WebrtcState, Uplink);
webrtc_message!(WebrtcRequest, Topic::WebrtcRequest, Downlink);
webrtc_message!(WebrtcAnswer, Topic::WebrtcAnswer, Downlink);
webrtc_message!(WebrtcIceDashboard, Topic::WebrtcIceDashboard, Downlink);

#[derive(Serialize, serde::Deserialize)]
struct OccupancyHeader {
    t: f64,
    resolution: f64,
    count: usize,
    #[serde(default)]
    free_count: usize,
}

impl Message for Occupancy {
    const TOPIC: Topic = Topic::Occupancy;

    fn encode(&self) -> Vec<u8> {
        let header = OccupancyHeader {
            t: self.t,
            resolution: self.resolution,
            count: self.cells.len(),
            free_count: self.free.len(),
        };
        let mut out = encode_json(&header);
        out.reserve(1 + (self.cells.len() + self.free.len()) * 8);
        out.push(b'\n');
        for cell in self.cells.iter().chain(&self.free) {
            for c in cell {
                out.extend_from_slice(&c.to_le_bytes());
            }
        }
        out
    }

    fn decode(bytes: &[u8]) -> Result<Self, DecodeError> {
        let newline = bytes.iter().position(|&b| b == b'\n').ok_or_else(|| malformed("no occupancy header line"))?;
        let header: OccupancyHeader = decode_json(&bytes[..newline])?;
        let body = &bytes[newline + 1..];
        let total = header.count.checked_add(header.free_count).and_then(|n| n.checked_mul(8));
        if Some(body.len()) != total {
            return Err(malformed(format!(
                "occupancy: {} bytes for {} occupied + {} free cells",
                body.len(),
                header.count,
                header.free_count
            )));
        }
        let decode_pairs = |slice: &[u8]| -> Vec<[f32; 2]> {
            slice
                .as_chunks::<8>()
                .0
                .iter()
                .map(|p| {
                    let f = |i: usize| f32::from_le_bytes([p[i], p[i + 1], p[i + 2], p[i + 3]]);
                    [f(0), f(4)]
                })
                .collect()
        };
        let split = header.count * 8;
        let cells = decode_pairs(&body[..split]);
        let free = decode_pairs(&body[split..]);
        Ok(Self { t: header.t, resolution: header.resolution, cells, free })
    }
}
impl Uplink for Occupancy {}

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
            uplink::<WorldMap>(),
            uplink::<MeshChunk>(),
            uplink::<Occupancy>(),
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
    fn occupancy_round_trip() {
        round_trip(Occupancy {
            t: 4.5,
            resolution: 0.1,
            cells: vec![[0.05, -0.15], [1.25, 2.35]],
            free: vec![[0.05, 0.05], [0.15, 0.05]],
        });
        round_trip(Occupancy { t: 5.0, resolution: 0.1, cells: vec![], free: vec![] });

        // Legacy header without `free_count` still decodes cleanly.
        let mut legacy = br#"{"version":1,"t":2.0,"resolution":0.1,"count":1}"#.to_vec();
        legacy.push(b'\n');
        for c in [0.25f32, -0.35] {
            legacy.extend_from_slice(&c.to_le_bytes());
        }
        let decoded = Occupancy::decode(&legacy).unwrap();
        assert_eq!(decoded.cells, vec![[0.25, -0.35]]);
        assert!(decoded.free.is_empty());

        // A body that doesn't match the header's count is rejected.
        let mut bytes = Occupancy { t: 1.0, resolution: 0.1, cells: vec![[0.0, 0.0]], free: vec![] }.encode();
        bytes.pop();
        assert!(Occupancy::decode(&bytes).is_err());
    }

    #[test]
    fn mesh_chunk_round_trip() {
        round_trip(MeshChunk {
            t: 12.5,
            id: "A1B2-C3D4".into(),
            vertices: vec![[1.0, 2.0, 0.0], [3.0, 4.0, 0.0], [1.0, 4.0, 0.0]],
            indices: vec![[0, 1, 2]],
            classes: vec![2],
            colors: vec![[255, 0, 0], [0, 255, 0], [0, 0, 255]],
        });
        round_trip(MeshChunk {
            t: 13.0,
            id: "A1B2-C3D4".into(),
            vertices: vec![],
            indices: vec![],
            classes: vec![],
            colors: vec![],
        });
        // Legacy header without `colors` field still decodes cleanly.
        let mut legacy = br#"{"version":1,"t":1.0,"id":"old","vertices":1,"faces":0,"classes":0}"#.to_vec();
        legacy.push(b'\n');
        for c in [1.0f32, 2.0, 3.0] {
            legacy.extend_from_slice(&c.to_le_bytes());
        }
        let decoded = MeshChunk::decode(&legacy).unwrap();
        assert_eq!(decoded.vertices, vec![[1.0, 2.0, 3.0]]);
        assert!(decoded.colors.is_empty());
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
            footprint: Some([0.25, 0.16]),
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
            gate_open: true,
        });
        round_trip(PlannedPath {
            t: 1.5,
            poses: vec![
                Pose2 { x: 0.0, y: 0.0, yaw: 0.0 },
                Pose2 { x: 1.0, y: 0.5, yaw: 0.46 },
            ],
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
            Goal::Explore { budget_s: Some(120.0) },
            Goal::Explore { budget_s: None },
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
        round_trip(GateCommand { t: 4.0, open: true });
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
