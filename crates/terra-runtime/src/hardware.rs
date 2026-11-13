//! Relay status reported by the hardware adapter, read before reconfiguring
//! navigation or changing the navigation source.
use serde_json::Value;

/// Maximum age of relay status used for source and configuration admission.
pub const HARDWARE_FRESHNESS_SECONDS: f64 = 0.5;

/// The relay status fields the runtime acts on.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct HardwareStatus {
    /// `ready` is `true`.
    pub ready: bool,
    /// `output_active`, when the adapter reports it.
    pub output_active: Option<bool>,
    /// A non-null `fault`.
    pub fault: bool,
}

impl HardwareStatus {
    /// Read the status JSON the apps publish; other fields are ignored and
    /// a missing or mistyped field counts as not reported.
    pub fn from_json(status: &Value) -> Self {
        Self {
            ready: status["ready"] == true,
            output_active: status["output_active"].as_bool(),
            fault: !status["fault"].is_null(),
        }
    }
}

/// One status report and when it was received.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Hardware {
    pub(crate) status: HardwareStatus,
    pub(crate) received_at: f64,
}

impl Hardware {
    fn fresh(&self, now: f64) -> bool {
        (0.0..HARDWARE_FRESHNESS_SECONDS).contains(&(now - self.received_at))
    }
    /// Fresh, ready, and reporting no wheel output.
    pub(crate) fn fresh_and_idle(&self, now: f64) -> bool {
        self.fresh(now) && self.status.ready && self.status.output_active == Some(false)
    }
    /// Fresh, ready, and without a fault.
    pub(crate) fn fresh_and_ready(&self, now: f64) -> bool {
        self.fresh(now) && self.status.ready && !self.status.fault
    }
}
