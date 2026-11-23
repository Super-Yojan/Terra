//! Key names. Every key is `{prefix}/{rover_id}/{topic}`; callers never
//! format keys by hand.
use std::fmt;
use std::time::Duration;

use crate::config::ConfigError;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Topic {
    // rover → base
    Capabilities,
    TaskStatus,
    Pose,
    Status,
    Depth,
    WorldMap,
    Mesh,
    Occupancy,
    PlannedPath,
    RemoteState,
    WebrtcOffer,
    WebrtcIcePhone,
    WebrtcState,
    // base → rover
    Task,
    TaskCommand,
    Teleop,
    RemoteCmd,
    Estop,
    Gate,
    WebrtcRequest,
    WebrtcAnswer,
    WebrtcIceDashboard,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    RoverToBase,
    BaseToRover,
}

impl Topic {
    pub const COUNT: usize = 22;
    pub const ALL: [Topic; Self::COUNT] = [
        Self::Capabilities,
        Self::TaskStatus,
        Self::Pose,
        Self::Status,
        Self::Depth,
        Self::WorldMap,
        Self::Mesh,
        Self::Occupancy,
        Self::PlannedPath,
        Self::RemoteState,
        Self::WebrtcOffer,
        Self::WebrtcIcePhone,
        Self::WebrtcState,
        Self::Task,
        Self::TaskCommand,
        Self::Teleop,
        Self::RemoteCmd,
        Self::Estop,
        Self::Gate,
        Self::WebrtcRequest,
        Self::WebrtcAnswer,
        Self::WebrtcIceDashboard,
    ];

    /// The last part of the key.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Capabilities => "capabilities",
            Self::TaskStatus => "task/status",
            Self::Pose => "pose",
            Self::Status => "status",
            Self::Depth => "depth",
            Self::WorldMap => "world_map",
            Self::Mesh => "mesh",
            Self::Occupancy => "occupancy",
            Self::PlannedPath => "planned_path",
            Self::RemoteState => "remote/state",
            Self::WebrtcOffer => "webrtc/offer",
            Self::WebrtcIcePhone => "webrtc/ice/phone",
            Self::WebrtcState => "webrtc/state",
            Self::Task => "task",
            Self::TaskCommand => "task/cmd",
            Self::Teleop => "teleop",
            Self::RemoteCmd => "remote/cmd",
            Self::Estop => "estop",
            Self::Gate => "gate",
            Self::WebrtcRequest => "webrtc/request",
            Self::WebrtcAnswer => "webrtc/answer",
            Self::WebrtcIceDashboard => "webrtc/ice/dashboard",
        }
    }

    pub fn from_key_part(s: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|t| t.as_str() == s)
    }

    pub const fn direction(self) -> Direction {
        match self {
            Self::Capabilities
            | Self::TaskStatus
            | Self::Pose
            | Self::Status
            | Self::Depth
            | Self::WorldMap
            | Self::Mesh
            | Self::Occupancy
            | Self::PlannedPath
            | Self::RemoteState
            | Self::WebrtcOffer
            | Self::WebrtcIcePhone
            | Self::WebrtcState => Direction::RoverToBase,
            Self::Task
            | Self::TaskCommand
            | Self::Teleop
            | Self::RemoteCmd
            | Self::Estop
            | Self::Gate
            | Self::WebrtcRequest
            | Self::WebrtcAnswer
            | Self::WebrtcIceDashboard => Direction::BaseToRover,
        }
    }

    /// True for any of the six WebRTC FPV signaling topics.
    pub const fn is_webrtc(self) -> bool {
        matches!(
            self,
            Self::WebrtcOffer
                | Self::WebrtcIcePhone
                | Self::WebrtcState
                | Self::WebrtcRequest
                | Self::WebrtcAnswer
                | Self::WebrtcIceDashboard
        )
    }

    /// Rover-side topics the link re-sends on its own: the latest published
    /// message goes out again at this period, and right after (re)connecting.
    pub const fn repeat_period(self) -> Option<Duration> {
        match self {
            Self::Capabilities => Some(Duration::from_secs(5)),
            Self::TaskStatus | Self::Status => Some(Duration::from_secs(1)),
            // Sent on change; repeated so a base station that starts later
            // still gets the map and planned route.
            Self::Occupancy | Self::PlannedPath => Some(Duration::from_secs(2)),
            _ => None,
        }
    }

    #[cfg_attr(not(feature = "zenoh"), allow(dead_code))]
    pub(crate) const fn index(self) -> usize {
        self as usize
    }
}

impl fmt::Display for Topic {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// Builds and parses keys under one prefix.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Topics {
    prefix: String,
}

impl Topics {
    /// Rejects a prefix that isn't a plain key (see [`crate::CommsConfig`]).
    pub fn new(prefix: &str) -> Result<Self, ConfigError> {
        check_prefix(prefix)?;
        Ok(Self { prefix: prefix.to_owned() })
    }

    pub fn prefix(&self) -> &str {
        &self.prefix
    }

    pub fn key(&self, rover_id: u64, topic: Topic) -> String {
        format!("{}/{}/{}", self.prefix, rover_id, topic.as_str())
    }

    /// `{prefix}/*/{topic}`: this topic from every rover.
    pub fn any_rover(&self, topic: Topic) -> String {
        format!("{}/*/{}", self.prefix, topic.as_str())
    }

    pub fn parse(&self, key: &str) -> Option<(u64, Topic)> {
        let rest = key.strip_prefix(self.prefix.as_str())?.strip_prefix('/')?;
        let (id, topic) = rest.split_once('/')?;
        if id.is_empty() || !id.bytes().all(|b| b.is_ascii_digit()) {
            return None;
        }
        Some((id.parse().ok()?, Topic::from_key_part(topic)?))
    }
}

pub(crate) fn check_prefix(prefix: &str) -> Result<(), ConfigError> {
    let ok = !prefix.is_empty()
        && prefix.split('/').all(|part| !part.is_empty())
        && !prefix.contains(['*', '?', '#', '$']);
    if ok {
        Ok(())
    } else {
        Err(ConfigError::new(format!(
            "[comms] prefix {prefix:?} must be a plain key: no * ? # $ and no empty parts"
        )))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn key_round_trip() {
        let t = Topics::new("terra/fleet").unwrap();
        for topic in Topic::ALL {
            let key = t.key(42, topic);
            assert_eq!(t.parse(&key), Some((42, topic)), "{key}");
        }
        assert_eq!(t.key(0, Topic::TaskStatus), "terra/fleet/0/task/status");
        assert_eq!(t.any_rover(Topic::Pose), "terra/fleet/*/pose");
    }

    #[test]
    fn parse_rejects_foreign_keys() {
        let t = Topics::new("terra").unwrap();
        for key in ["terra", "terra/1", "terrax/1/pose", "other/1/pose", "terra/x/pose", "terra/+1/pose", "terra/1/nope", "terra/1/pose/extra"] {
            assert_eq!(t.parse(key), None, "{key}");
        }
    }

    #[test]
    fn indices_match_all() {
        for (i, topic) in Topic::ALL.into_iter().enumerate() {
            assert_eq!(topic.index(), i);
        }
    }
}
