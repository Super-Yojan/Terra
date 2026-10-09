//! UniFFI ingress for calibrated detector adapters. Registration is not a detector implementation.
use crate::*;
#[derive(Clone, Debug, uniffi::Record)]
pub struct MobileSearchRequest {
    pub run_id: String,
    pub search_id: String,
    pub token: String,
    pub target_class: String,
    pub min_x: f64,
    pub min_y: f64,
    pub max_x: f64,
    pub max_y: f64,
    pub time_budget_s: f64,
}
#[derive(Clone, Debug, uniffi::Record)]
pub struct MobileTargetObservation {
    pub run_id: String,
    pub search_id: String,
    pub frame_id: u64,
    pub target_class: String,
    pub confidence: f64,
    pub world_x: f64,
    pub world_y: f64,
    pub received_at: f64,
    pub evidence_id: String,
}
#[derive(Clone, Debug, uniffi::Record)]
pub struct MobileSearchSnapshot {
    pub run_id: String,
    pub search_id: String,
    pub phase: String,
    pub reason: String,
    pub elapsed_s: f64,
    pub detector_available: bool,
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
    let pose = b.pose.unwrap_or_default();
    let map = b.map.clone();
    let e = b.estimator.estimate(timestamp);
    b.autonomy.run_id = b.run_id.clone();
    let input = terra_autonomy::ArbiterInput {
        now: timestamp,
        pose: pose.0,
        pose_time: pose.1,
        map: map.as_ref().map(|m| &m.0),
        map_time: map.as_ref().map(|m| m.1),
        map_revision: map.as_ref().map(|m| m.2).unwrap_or(0),
        measured: terra_navigation::Twist {
            linear: e.forward,
            angular: e.yaw_rate,
        },
        healthy: e.health == Health::Ready && b.recorder.as_ref().is_none_or(|r| !r.failed()),
    };
    match kind {
        "search" => b
            .autonomy
            .start_search(
                terra_autonomy::decode_search_request(bytes)
                    .ok_or_else(|| search_error("invalid search request"))?,
                &input,
            )
            .map_err(search_error),
        "search/action" => b
            .autonomy
            .apply_search_action(
                terra_autonomy::decode_search_action(bytes)
                    .ok_or_else(|| search_error("invalid search action"))?,
                &input,
            )
            .map_err(search_error),
        "search/report/ack" => {
            let r = terra_autonomy::decode_search_report_ack(bytes)
                .ok_or_else(|| search_error("invalid report acknowledgement"))?;
            if b.autonomy.ack_search_report(r) {
                Ok(())
            } else {
                Err(search_error("unknown report"))
            }
        }
        _ => Err(search_error("unsupported search request")),
    }
}
#[uniffi::export]
impl MobileController {
    /// Call on each detector heartbeat, including frames with no detections. Empty classes withdraw capability.
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
    pub fn target_search_snapshot(&self, timestamp: f64) -> Option<MobileSearchSnapshot> {
        let b = self.brain.lock().ok()?;
        let s = b.autonomy.search_status(timestamp)?;
        Some(MobileSearchSnapshot {
            run_id: s.run_id,
            search_id: s.search_id,
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
