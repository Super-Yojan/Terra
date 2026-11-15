//! Operator request topics: what a dashboard (or the phone's own UI) may ask
//! the rover to do. Transport admits only these topic suffixes; the runtime
//! reads authority, envelope and stop rules from the same rows.
//!
//! A *neutral* request (a stop, a cancel, a zero teleop) needs no current
//! authority and never expires, so an operator can always halt the rover. A
//! *stop* is applied after every other request received in the same tick.

use serde_json::Value;

/// Largest request payload in bytes, for every topic.
pub const MAX_REQUEST_BYTES: usize = 2048;
/// Largest number of requests waiting for the next control tick.
pub const MAX_QUEUED_REQUESTS: usize = 256;

pub const AUTONOMY: &str = "autonomy";
pub const TELEOP: &str = "teleop";
pub const SAFETY: &str = "safety";
pub const GOAL: &str = "goal";
pub const GOAL_DECISION: &str = "goal/decision";
pub const NAVIGATION_SETTINGS: &str = "navigation/settings";
pub const NAVIGATION_SOURCE: &str = "navigation/source";
pub const NAVIGATION_RESUME: &str = "navigation/resume";
pub const SEARCH: &str = "search";
pub const SEARCH_ACTION: &str = "search/action";
pub const SEARCH_REPORT_ACK: &str = "search/report/ack";
pub const TASK: &str = "task";
pub const TASK_ACTION: &str = "task/action";
pub const DOCK: &str = "dock";

/// Authority fields the runtime removes before the request reaches its decoder.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Strip {
    /// The decoder takes the whole envelope.
    Nothing,
    /// Remove `authority_revision`; the decoder still checks `run_id`.
    Revision,
    /// Remove `run_id` and `authority_revision`.
    Authority,
}

/// One request topic and its admission rules.
#[derive(Clone, Copy, Debug)]
pub struct RequestKind {
    /// Topic suffix, for example `goal/decision`.
    pub topic: &'static str,
    /// Whether the phone's own UI may send it; teleop has its own local input.
    pub local: bool,
    /// Authority fields removed before decoding.
    pub strip: Strip,
    /// The handler keeps its own accepted/rejected result in status
    /// (navigation settings and source), so a refusal is not also reported
    /// as a rejected autonomy request.
    pub records_own_result: bool,
    neutral: fn(&Value) -> bool,
}

fn never(_: &Value) -> bool {
    false
}
fn always(_: &Value) -> bool {
    true
}
fn cancel_action(v: &Value) -> bool {
    v["action"] == "cancel"
}

/// Every request topic.
pub const ALL: [RequestKind; 14] = [
    RequestKind {
        topic: AUTONOMY,
        local: true,
        strip: Strip::Authority,
        records_own_result: false,
        neutral: never,
    },
    RequestKind {
        topic: TELEOP,
        local: false,
        strip: Strip::Nothing,
        records_own_result: false,
        neutral: |v| v["linear"].as_f64() == Some(0.) && v["angular"].as_f64() == Some(0.),
    },
    RequestKind {
        topic: SAFETY,
        local: true,
        strip: Strip::Authority,
        records_own_result: false,
        neutral: |v| v["action"] == "stop",
    },
    RequestKind {
        topic: GOAL,
        local: true,
        strip: Strip::Revision,
        records_own_result: false,
        neutral: |v| v["cancel"] == true,
    },
    RequestKind {
        topic: GOAL_DECISION,
        local: true,
        strip: Strip::Revision,
        records_own_result: false,
        neutral: never,
    },
    RequestKind {
        topic: NAVIGATION_SETTINGS,
        local: true,
        strip: Strip::Nothing,
        records_own_result: true,
        neutral: never,
    },
    RequestKind {
        topic: NAVIGATION_SOURCE,
        local: true,
        strip: Strip::Nothing,
        records_own_result: true,
        neutral: never,
    },
    RequestKind {
        topic: NAVIGATION_RESUME,
        local: true,
        strip: Strip::Nothing,
        records_own_result: true,
        neutral: never,
    },
    RequestKind {
        topic: SEARCH,
        local: true,
        strip: Strip::Revision,
        records_own_result: false,
        neutral: never,
    },
    RequestKind {
        topic: SEARCH_ACTION,
        local: true,
        strip: Strip::Revision,
        records_own_result: false,
        neutral: |v| matches!(v["action"].as_str(), Some("pause" | "cancel")),
    },
    RequestKind {
        topic: SEARCH_REPORT_ACK,
        local: true,
        strip: Strip::Revision,
        records_own_result: false,
        neutral: always,
    },
    RequestKind {
        topic: TASK,
        local: true,
        strip: Strip::Revision,
        records_own_result: false,
        neutral: never,
    },
    RequestKind {
        topic: TASK_ACTION,
        local: true,
        strip: Strip::Revision,
        records_own_result: false,
        neutral: cancel_action,
    },
    RequestKind {
        topic: DOCK,
        local: true,
        strip: Strip::Revision,
        records_own_result: false,
        neutral: cancel_action,
    },
];

impl RequestKind {
    /// The row for a topic suffix; `None` for anything that is not a request.
    pub fn find(topic: &str) -> Option<&'static Self> {
        ALL.iter().find(|k| k.topic == topic)
    }
    /// Whether this request may run without current authority and at any age.
    pub fn neutral(&self, request: &Value) -> bool {
        (self.neutral)(request)
    }
    /// Whether this request is an emergency stop, which outranks every other
    /// request received in the same tick.
    pub fn stop(&self, request: &Value) -> bool {
        self.topic == SAFETY && request["action"] == "stop"
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn topics_are_unique_and_found() {
        for k in &ALL {
            assert_eq!(RequestKind::find(k.topic).unwrap().topic, k.topic);
        }
        assert!(RequestKind::find("hardware").is_none());
        assert!(RequestKind::find("search/status").is_none());
    }

    #[test]
    fn stops_and_cancels_are_neutral() {
        let k = |t| RequestKind::find(t).unwrap();
        assert!(k(SAFETY).neutral(&json!({"action":"stop"})));
        assert!(k(SAFETY).stop(&json!({"action":"stop"})));
        assert!(!k(SAFETY).neutral(&json!({"action":"reset"})));
        assert!(k(TELEOP).neutral(&json!({"linear":0.0,"angular":0.0})));
        assert!(!k(TELEOP).neutral(&json!({"linear":0.1,"angular":0.0})));
        assert!(k(DOCK).neutral(&json!({"action":"cancel"})));
        assert!(!k(GOAL).stop(&json!({"cancel":true})));
    }
}
