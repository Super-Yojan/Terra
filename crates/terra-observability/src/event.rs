use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::time::{SystemTime, UNIX_EPOCH};

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq, Hash)]
#[serde(deny_unknown_fields)]
pub struct EventId {
    pub producer_id: String,
    pub boot_id: String,
    pub sequence: u64,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct EventInput {
    pub rover_id: String,
    pub session_id: Option<String>,
    pub component: String,
    pub kind: String,
    pub severity: String,
    pub wall_time_ns: u64,
    pub monotonic_ns: u64,
    pub cause_id: Option<EventId>,
    pub attributes: Value,
}
impl EventInput {
    pub fn new(rover: &str, component: &str, kind: &str, attributes: Value) -> Self {
        Self {
            rover_id: rover.into(),
            session_id: None,
            component: component.into(),
            kind: kind.into(),
            severity: "info".into(),
            wall_time_ns: wall_time_ns(),
            monotonic_ns: 0,
            cause_id: None,
            attributes,
        }
    }
    pub fn validate(&self) -> bool {
        [&self.rover_id, &self.component, &self.kind]
            .iter()
            .all(|s| !s.is_empty() && s.len() <= 256)
            && self.session_id.as_ref().is_none_or(|s| s.len() <= 256)
            && ["debug", "info", "warning", "error"].contains(&self.severity.as_str())
            && self.attributes.is_object()
    }
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct EventEnvelope {
    pub schema_version: u32,
    pub id: EventId,
    pub input: EventInput,
}
pub fn wall_time_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos()
        .min(u64::MAX as u128) as u64
}
