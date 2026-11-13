//! The `control_tick` recording line, shared by every host that records runs.

use serde_json::{Map, Value, json};
use terra_runtime::RuntimeOutput;

/// `kind` of a control-tick record.
pub const CONTROL_TICK: &str = "control_tick";

/// One control tick as recorded: what the runtime chose and why. Hosts build it
/// from [`Runtime::step`](terra_runtime::Runtime::step)'s output and
/// [`crate::summarize`] reads the recorded lines back.
pub struct ControlTickRecord<'a> {
    /// Seconds from run start.
    pub time: f64,
    /// Host control-clock time, when it differs from `time`.
    pub source_time: Option<f64>,
    pub sequence: u64,
    /// Fleet rover identity; per-rover metrics treat a missing one as rover 0.
    pub rover_id: Option<u64>,
    pub run_id: &'a str,
    pub output: &'a RuntimeOutput,
    /// Host-specific fields, such as the simulator's pose and mission status.
    pub extra: Map<String, Value>,
}

impl<'a> ControlTickRecord<'a> {
    pub fn new(output: &'a RuntimeOutput, run_id: &'a str, sequence: u64, time: f64) -> Self {
        Self {
            time,
            source_time: None,
            sequence,
            rover_id: None,
            run_id,
            output,
            extra: Map::new(),
        }
    }

    /// The JSON line to record.
    pub fn to_value(&self) -> Value {
        let autonomy = &self.output.autonomy;
        let mut record = json!({
            "kind": CONTROL_TICK,
            "time": self.time,
            "sequence": self.sequence,
            "run_id": self.run_id,
            "clearance": self.output.clearance,
            "selected": autonomy.twist,
            "source_command": autonomy.intent,
            "status": autonomy.status,
            "goal": autonomy.goal,
            // Run-relative expiry, as in the published status.
            "proposal": self.output.status["proposal"],
            "exploration": autonomy.exploration,
            "search": autonomy.search,
            "task": autonomy.task,
            "events": autonomy.events,
        });
        if let Some(t) = self.source_time {
            record["source_time"] = json!(t);
        }
        if let Some(id) = self.rover_id {
            record["rover_id"] = json!(id);
        }
        for (key, value) in &self.extra {
            record[key] = value.clone();
        }
        record
    }
}
