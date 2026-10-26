//! UniFFI bridge for L5 task autonomy. The phone runs the language model; Rust
//! builds every prompt, validates every reply and drives the waypoints.
use crate::*;

#[derive(Clone, Debug, uniffi::Record)]
/// One request for the on-phone model. Answer it with
/// [`MobileController::submit_task_response`] using the same `query_id`.
pub struct MobileTaskQuery {
    pub query_id: u64,
    pub task_id: String,
    /// Fixed rules and reply format; use as the model's instructions.
    pub system: String,
    /// This step's task, pose, map summary and history.
    pub prompt: String,
}

#[uniffi::export]
impl MobileController {
    /// Advertise the phone's language model by name, or withdraw it with `None`.
    /// The `task` level (L5) is accepted only while a model is advertised.
    pub fn set_task_model(&self, name: Option<String>) -> Result<(), ControllerError> {
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .autonomy
            .set_task_model(name);
        Ok(())
    }
    /// Take the next model query, if the task is waiting for one. Each query is returned once.
    pub fn take_task_query(&self) -> Option<MobileTaskQuery> {
        let mut b = self.brain.lock().ok()?;
        b.autonomy.take_task_query().map(|q| MobileTaskQuery {
            query_id: q.query_id,
            task_id: q.task_id,
            system: q.system,
            prompt: q.prompt,
        })
    }
    /// Return the model's raw text for `query_id`. It is validated on the next control tick.
    /// `latency_seconds` is how long the model took. Returns false for an unknown query.
    pub fn submit_task_response(&self, query_id: u64, text: String, latency_seconds: f64) -> bool {
        self.brain.lock().is_ok_and(|mut b| {
            b.autonomy
                .submit_task_response(query_id, text, Some(latency_seconds))
        })
    }
    /// Report that the model could not answer `query_id` (unavailable, refused, error).
    pub fn fail_task_query(&self, query_id: u64, reason: String) -> bool {
        self.brain
            .lock()
            .is_ok_and(|mut b| b.autonomy.fail_task_query(query_id, reason))
    }
    /// Name a point the phone recognised (for example a detector hit) in local metres,
    /// so the model can choose it as a landmark target.
    pub fn observe_task_landmark(
        &self,
        label: String,
        x: f64,
        y: f64,
        confidence: f64,
        timestamp: f64,
    ) -> Result<(), ControllerError> {
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .autonomy
            .observe_task_landmark(&label, x, y, confidence, timestamp);
        Ok(())
    }
}
