//! Optional on-phone diagnostics, forwarded to `terra_robot::diagnostics`.
//! Never participates in control health or command admission.
use crate::*;

#[derive(Clone, uniffi::Record)]
pub struct ObservationAttachment {
    pub name: String,
    pub data: Vec<u8>,
    pub encoding: String,
    pub dimensions: Vec<u32>,
    pub units: String,
}

#[uniffi::export]
impl MobileController {
    pub fn start_observability(&self, config_json: String) -> Result<(), ControllerError> {
        self.diagnostics.start(&config_json).map_err(invalid)
    }
    pub fn set_observability_rover(&self, rover_id: String) {
        self.diagnostics.set_rover(rover_id);
    }
    pub fn record_observation(&self, event_json: String) -> bool {
        self.diagnostics.record(&event_json)
    }
    pub fn flush_observability(&self, seconds: f64) -> Result<(), ControllerError> {
        self.diagnostics.flush(seconds).map_err(invalid)
    }
    pub fn record_mapping_evidence(
        &self,
        manifest_json: String,
        attachments: Vec<ObservationAttachment>,
    ) -> bool {
        self.diagnostics.record_mapping_evidence(
            &manifest_json,
            attachments
                .into_iter()
                .map(|a| terra_observability::AttachmentInput {
                    name: a.name,
                    data: a.data,
                    encoding: a.encoding,
                    dimensions: a.dimensions,
                    units: a.units,
                })
                .collect(),
        )
    }
    pub fn observability_settings(&self) -> String {
        self.diagnostics.settings_json()
    }
    pub fn record_observation_gap(&self, reason: String) {
        self.diagnostics.event(
            "observability",
            terra_observability::GAP_KIND,
            serde_json::json!({"reason":reason,"time_basis":"wall_only"}),
            &self.observation_run_id(),
            0.,
        );
    }
    /// The run used by Swift diagnostics and camera evidence; empty while a tick runs.
    pub fn observation_run_id(&self) -> String {
        self.robot
            .try_lock()
            .map(|r| r.run_id().to_owned())
            .unwrap_or_default()
    }
    /// Configure the independent uploader. Invoke off the control queue;
    /// credentials are memory-only.
    pub fn start_observability_upload(
        &self,
        endpoint: String,
        token: String,
    ) -> Result<(), ControllerError> {
        self.diagnostics
            .start_upload(endpoint, token)
            .map_err(invalid)
    }
}
