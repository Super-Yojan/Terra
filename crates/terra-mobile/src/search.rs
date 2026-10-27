//! UniFFI ingress for calibrated detector adapters. Registration is not a detector implementation.
use crate::*;
#[derive(Clone, Debug, uniffi::Record)]
/// Run-scoped target search over an axis-aligned world-space rectangle.
pub struct MobileSearchRequest {
    /// Controller run identity; must match the active run.
    pub run_id: String,
    /// Identity of the search within its run.
    pub search_id: String,
    /// Caller-generated request token used for correlation and replay protection.
    pub token: String,
    /// Detector-supported class token to search for or report.
    pub target_class: String,
    /// Minimum search-bound world X, metres.
    pub min_x: f64,
    /// Minimum search-bound world Y, metres.
    pub min_y: f64,
    /// Maximum search-bound world X, metres.
    pub max_x: f64,
    /// Maximum search-bound world Y, metres.
    pub max_y: f64,
    /// Requested search duration budget, seconds.
    pub time_budget_s: f64,
}
#[derive(Clone, Debug, uniffi::Record)]
/// Calibrated detector evidence localized into the controller world frame.
pub struct MobileTargetObservation {
    /// Controller run identity; must match the active run.
    pub run_id: String,
    /// Identity of the search within its run.
    pub search_id: String,
    /// Source frame identifier used to distinguish confirmation evidence.
    pub frame_id: u64,
    /// Detector-supported class token to search for or report.
    pub target_class: String,
    /// Detector confidence in the inclusive range 0..=1.
    pub confidence: f64,
    /// Observed target world X, metres.
    pub world_x: f64,
    /// Observed target world Y, metres.
    pub world_y: f64,
    /// Observation receipt time, seconds on the controller monotonic clock.
    pub received_at: f64,
    /// Identifier of the source evidence retained in the report.
    pub evidence_id: String,
}
#[derive(Clone, Debug, uniffi::Record)]
/// One object detection (bucket, ball) localized into the controller world frame.
pub struct MobileObjectDetection {
    /// Class token, for example `bucket` or `ball`.
    pub class_name: String,
    /// Optional colour token, for example `blue` or `orange`.
    pub color: Option<String>,
    /// Detector confidence in the inclusive range 0..=1.
    pub confidence: f64,
    /// Source camera frame; one frame counts once per object.
    pub frame_id: u64,
    /// Observed object world X, metres.
    pub world_x: f64,
    /// Observed object world Y, metres.
    pub world_y: f64,
    /// Camera frame time, seconds on the controller monotonic clock.
    pub received_at: f64,
}
#[derive(Clone, Debug, uniffi::Record)]
/// Latest target-search lifecycle state and optional serialized completion report.
pub struct MobileSearchSnapshot {
    /// Controller run identity; must match the active run.
    pub run_id: String,
    /// Identity of the search within its run.
    pub search_id: String,
    /// Class the search is looking for, for example `bucket`.
    pub target_class: String,
    /// Serialized search lifecycle phase name.
    pub phase: String,
    /// Machine-readable search transition or rejection reason.
    pub reason: String,
    /// Elapsed search duration, seconds.
    pub elapsed_s: f64,
    /// Whether a matching detector capability is currently fresh.
    pub detector_available: bool,
    /// Serialized completion report when available.
    pub report_json: Option<String>,
}
fn search_error(message: &str) -> ControllerError {
    ControllerError::InvalidInput {
        message: message.into(),
    }
}
pub(crate) fn dispatch_search(
    b: &mut Brain,
    kind: &str,
    bytes: &[u8],
    timestamp: f64,
) -> Result<(), ControllerError> {
    b.runtime.recording_healthy = b.recorder.as_ref().is_none_or(|r| !r.failed());
    terra_runtime::search::dispatch_search(&mut b.runtime, kind, bytes, timestamp)
        .map_err(Into::into)
}
#[uniffi::export]
impl MobileController {
    /// Call on each detector heartbeat, including frames with no detections. Empty classes withdraw capability.
    /// `timestamp` is monotonic seconds. Returns `InvalidInput` for invalid detector
    /// metadata or time, and `Internal` for a poisoned mutex.
    pub fn register_search_detector(
        &self,
        classes: Vec<String>,
        version: String,
        timestamp: f64,
    ) -> Result<(), ControllerError> {
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .autonomy
            .set_detector_capability(classes, version, timestamp)
            .map_err(search_error)
    }
    /// Validate and submit a search using current pose, map, detector, and authority.
    /// `timestamp` is monotonic seconds. Returns `InvalidInput` for malformed or
    /// inadmissible requests, and `Internal` for a poisoned mutex or serialization failure.
    pub fn start_target_search(
        &self,
        r: MobileSearchRequest,
        timestamp: f64,
    ) -> Result<(), ControllerError> {
        let request = terra_autonomy::SearchRequest {
            version: 1,
            run_id: r.run_id,
            search_id: r.search_id,
            token: r.token,
            target_class: r.target_class,
            bounds: terra_autonomy::SearchBounds {
                min_x: r.min_x,
                min_y: r.min_y,
                max_x: r.max_x,
                max_y: r.max_y,
            },
            time_budget_s: r.time_budget_s,
        };
        let mut b = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        dispatch_search(
            &mut b,
            "search",
            &serde_json::to_vec(&request).map_err(|_| ControllerError::Internal)?,
            timestamp,
        )
    }
    /// Submit one detector observation for arbiter validation and confirmation.
    /// `timestamp` is current monotonic seconds. Returns `InvalidInput` for rejected
    /// evidence (including mismatched identities or stale data), or `Internal` on lock failure.
    pub fn push_target_observation(
        &self,
        o: MobileTargetObservation,
        timestamp: f64,
    ) -> Result<(), ControllerError> {
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .autonomy
            .accept_target_observation(
                terra_autonomy::TargetObservation {
                    run_id: o.run_id,
                    search_id: o.search_id,
                    frame_id: o.frame_id,
                    target_class: o.target_class,
                    confidence: o.confidence,
                    world_x: o.world_x,
                    world_y: o.world_y,
                    received_at: o.received_at,
                    evidence_id: o.evidence_id,
                },
                timestamp,
            )
            .map_err(search_error)
    }
    /// Add one object detection to the run's object map, which the dashboard
    /// shows on the `objects` topic. Latitude and longitude are added here when
    /// the outdoor GPS fix and heading are good enough. Returns `InvalidInput`
    /// for a bad class, position or a confidence below the map threshold.
    pub fn push_object_detection(
        &self,
        d: MobileObjectDetection,
        timestamp: f64,
    ) -> Result<(), ControllerError> {
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .observe_object(
                terra_autonomy::objects::ObjectSighting {
                    class: d.class_name,
                    color: d.color,
                    confidence: d.confidence,
                    frame_id: d.frame_id,
                    timestamp: d.received_at,
                    x: d.world_x,
                    y: d.world_y,
                    geo: None,
                },
                timestamp,
            )
            .map_err(Into::into)
    }
    /// The run's object map as the `objects` topic body, for the phone's debug page.
    pub fn object_map_json(&self) -> String {
        self.brain
            .lock()
            .map(|b| {
                b.objects
                    .telemetry(b.dashboard.as_ref().map_or(0, |d| d.rover_id), &b.run_id)
                    .to_string()
            })
            .unwrap_or_default()
    }
    /// Forget every mapped object, for example after moving buckets during a test.
    pub fn clear_object_map(&self) -> Result<(), ControllerError> {
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .objects
            .clear();
        Ok(())
    }
    /// Read search status at monotonic `timestamp` seconds without stepping motors.
    /// Returns `None` when no status is available, locking fails, or conversion fails.
    pub fn target_search_snapshot(&self, timestamp: f64) -> Option<MobileSearchSnapshot> {
        let b = self.brain.lock().ok()?;
        let s = b.autonomy.search_status(timestamp)?;
        Some(MobileSearchSnapshot {
            run_id: s.run_id,
            search_id: s.search_id,
            target_class: s.target_class,
            phase: serde_json::to_value(s.phase).ok()?.as_str()?.into(),
            reason: s.reason,
            elapsed_s: s.elapsed_s,
            detector_available: s.detector_available,
            report_json: s
                .report
                .as_ref()
                .and_then(|r| serde_json::to_string(r).ok()),
        })
    }
}
