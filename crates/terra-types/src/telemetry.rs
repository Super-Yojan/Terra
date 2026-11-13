//! Outbound telemetry topics: what the rover publishes for dashboards. The
//! phone names topics from here and transport admits only these suffixes,
//! each with its own byte cap and delivery rule.
//!
//! Content heartbeats carry the version key of the full payload they refresh:
//! `map/occupancy/heartbeat` uses `schema_version` like occupancy telemetry,
//! `pointcloud/heartbeat` uses `version` like the point cloud. Dashboards
//! (ARGOS `content_heartbeat.rs`) read each key by topic, so the two differ on
//! purpose.

use crate::point_capture;

/// Byte cap for a topic without a larger one.
pub const DEFAULT_MAX_BYTES: usize = 65536;
/// Byte cap for one occupancy grid snapshot.
pub const MAP_OCCUPANCY_MAX_BYTES: usize = 2_000_000;
/// Byte cap for one thinned point cloud.
pub const POINTCLOUD_MAX_BYTES: usize = 262144;
/// Most outstanding records in one `search/report` batch.
pub const SEARCH_REPORT_MAX_RECORDS: usize = 128;

pub const AUTONOMY_STATUS: &str = "autonomy/status";
pub const EXPERIMENT_STATUS: &str = "experiment/status";
pub const NAVIGATION_OUTDOOR: &str = "navigation/outdoor";
pub const POSE: &str = "pose";
pub const POSE_QUALITY: &str = "pose/quality";
pub const LOCALIZATION: &str = "localization";
pub const HARDWARE_STATUS: &str = "hardware/status";
pub const SEARCH_STATUS: &str = "search/status";
pub const SEARCH_REPORT: &str = "search/report";
pub const OBJECTS: &str = "objects";
pub const GOAL_STATUS: &str = "goal/status";
pub const GOAL_PROPOSAL: &str = "goal/proposal";
pub const EXPLORATION_STATUS: &str = "exploration/status";
pub const TASK_STATUS: &str = "task/status";
pub const MAP_OCCUPANCY: &str = "map/occupancy";
pub const MAP_OCCUPANCY_HEARTBEAT: &str = "map/occupancy/heartbeat";
pub const POINTCLOUD: &str = "pointcloud";
pub const POINTCLOUD_HEARTBEAT: &str = "pointcloud/heartbeat";

/// How transport delivers a topic.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Delivery {
    /// Latest value, replayed while fresh.
    State,
    /// Latest value, sent once per producer sample.
    Visualization,
    /// A bounded set of records replayed together; see [`SEARCH_REPORT_MAX_RECORDS`].
    Batch,
    /// One large payload sent on its own queue.
    Bulk,
}

/// One outbound topic and its limits.
#[derive(Clone, Copy, Debug)]
pub struct TelemetryTopic {
    /// Topic suffix under `<prefix>/<rover id>/`.
    pub topic: &'static str,
    /// Largest accepted payload in bytes (per record for a batch).
    pub max_bytes: usize,
    pub delivery: Delivery,
}

const fn state(topic: &'static str) -> TelemetryTopic {
    TelemetryTopic {
        topic,
        max_bytes: DEFAULT_MAX_BYTES,
        delivery: Delivery::State,
    }
}
const fn visualization(topic: &'static str, max_bytes: usize) -> TelemetryTopic {
    TelemetryTopic {
        topic,
        max_bytes,
        delivery: Delivery::Visualization,
    }
}

/// Every outbound telemetry topic.
pub const ALL: [TelemetryTopic; 20] = [
    state(AUTONOMY_STATUS),
    state(EXPERIMENT_STATUS),
    state(NAVIGATION_OUTDOOR),
    state(POSE),
    state(POSE_QUALITY),
    state(LOCALIZATION),
    state(HARDWARE_STATUS),
    state(SEARCH_STATUS),
    TelemetryTopic {
        topic: SEARCH_REPORT,
        max_bytes: DEFAULT_MAX_BYTES,
        delivery: Delivery::Batch,
    },
    state(OBJECTS),
    state(GOAL_STATUS),
    state(GOAL_PROPOSAL),
    state(EXPLORATION_STATUS),
    state(TASK_STATUS),
    state(point_capture::PROGRESS_TOPIC),
    visualization(MAP_OCCUPANCY, MAP_OCCUPANCY_MAX_BYTES),
    visualization(MAP_OCCUPANCY_HEARTBEAT, DEFAULT_MAX_BYTES),
    visualization(POINTCLOUD, POINTCLOUD_MAX_BYTES),
    visualization(POINTCLOUD_HEARTBEAT, DEFAULT_MAX_BYTES),
    TelemetryTopic {
        topic: point_capture::FRAME_TOPIC,
        max_bytes: point_capture::MAX_FRAME_BYTES,
        delivery: Delivery::Bulk,
    },
];

impl TelemetryTopic {
    /// The row for a topic suffix; `None` for anything that is not telemetry.
    pub fn find(topic: &str) -> Option<&'static Self> {
        ALL.iter().find(|t| t.topic == topic)
    }
    /// Whether `payload_len` bytes fit this topic.
    pub fn fits(&self, payload_len: usize) -> bool {
        payload_len <= self.max_bytes
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn topics_are_unique_and_not_requests() {
        for t in &ALL {
            assert_eq!(TelemetryTopic::find(t.topic).unwrap().topic, t.topic);
            assert!(crate::requests::RequestKind::find(t.topic).is_none());
        }
        assert!(TelemetryTopic::find("webrtc/offer").is_none());
    }
}
