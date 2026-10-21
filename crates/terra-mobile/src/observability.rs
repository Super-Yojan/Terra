//! Optional diagnostics. Never participates in control health or command admission.
use super::*;
use serde::Deserialize;
use std::{path::PathBuf, sync::RwLock, time::Duration};
use terra_observability::{EventInput, Journal, JournalConfig, RecordOutcome};
struct Attached {
    journal: Arc<Journal>,
    evidence: terra_observability::EvidenceWriter,
    rover: String,
    path: PathBuf,
    uploader: Option<terra_observability::UploadHandle>,
    settings: terra_observability::ObservationSettings,
}
#[derive(Default)]
pub(super) struct ObservationState {
    attached: RwLock<Option<Attached>>,
    last: Mutex<(String, f64)>,
}
#[derive(Deserialize)]
struct Config {
    path: PathBuf,
    producer_id: String,
    rover_id: String,
    #[serde(default)]
    settings: terra_observability::ObservationSettings,
}
impl ObservationState {
    pub(super) fn event(
        &self,
        component: &str,
        kind: &str,
        attributes: serde_json::Value,
        run: &str,
        time: f64,
    ) {
        if let Ok(a) = self.attached.try_read()
            && let Some(a) = a.as_ref()
        {
            let mut event = EventInput::new(&a.rover, component, kind, attributes);
            event.session_id = Some(run.into());
            event.monotonic_ns = (time.max(0.) * 1e9).min(u64::MAX as f64) as u64;
            a.journal.try_record(event);
        }
    }
    pub(super) fn effort(&self, brain: &Brain, time: f64, output: &terra_types::MotorOutput) {
        let Ok(last) = self.last.try_lock() else {
            return;
        };
        if time != last.1 {
            return;
        }
        self.event("control", "control.output", serde_json::json!({"left_effort":output.left,"right_effort":output.right,"estimated_forward":output.estimated_forward,"estimated_yaw_rate":output.estimated_yaw_rate,"target_forward":output.target_forward,"target_yaw_rate":output.target_yaw_rate,"stop_reason":format!("{:?}",output.stop_reason)}), &brain.run_id, time);
    }
    pub(super) fn tick(&self, brain: &Brain, time: f64, estimate: &terra_types::VelocityEstimate) {
        let Ok(mut last) = self.last.try_lock() else {
            return;
        };
        let status: serde_json::Value =
            serde_json::from_str(&brain.autonomy_json).unwrap_or_default();
        let key = serde_json::json!([
            status["status"]["safety"],
            status["status"]["reason"],
            status["status"]["requested_level"],
            status["goal"]["state"]
        ])
        .to_string();
        let transition = key != last.0;
        if transition {
            self.event("autonomy","autonomy.transition",serde_json::json!({"previous":last.0,"reason":status["status"]["reason"],"status":status,"pose_age":brain.pose.map(|p|time-p.1),"map_age":brain.map.as_ref().map(|m|time-m.1),"imu_age":brain.imu_time.map(|t|time-t),"map_timeout":brain.autonomy.map_freshness_timeout(),"pose_timeout":terra_types::timing::POSE_FRESHNESS_SECONDS,"router_connected":brain.dashboard.as_ref().map(|d|!d.failed()),"recorder_failed":brain.recorder.as_ref().is_some_and(|r|r.failed())}),&brain.run_id,time);
            last.0 = key;
        }
        let interval = self
            .attached
            .try_read()
            .ok()
            .and_then(|a| a.as_ref().map(|a| a.settings.control_sample_seconds))
            .unwrap_or_else(|| {
                terra_observability::ObservationSettings::default().control_sample_seconds
            });
        if transition || time - last.1 >= interval {
            self.event("control","control.summary",serde_json::json!({"state":status,"estimated_forward":estimate.forward,"estimated_yaw_rate":estimate.yaw_rate,"sensor_health":format!("{:?}",estimate.health),"pose":brain.pose.map(|p|serde_json::json!({"x":p.0.x,"y":p.0.y,"yaw":p.0.yaw})),"map_revision":brain.map.as_ref().map(|m|m.2)}),&brain.run_id,time);
            last.1 = time;
        }
    }
}
#[uniffi::export]
impl MobileController {
    /// Open the independent local journal on an administrative/background queue.
    pub fn start_observability(&self, config_json: String) -> Result<(), ControllerError> {
        let config: Config =
            serde_json::from_str(&config_json).map_err(|_| ControllerError::InvalidInput {
                message: "invalid observability configuration".into(),
            })?;
        if !config.settings.valid() {
            return Err(ControllerError::InvalidInput {
                message: "invalid observability settings".into(),
            });
        }
        let evidence_path = config.path.with_extension("evidence");
        let journal = Journal::open(JournalConfig {
            path: config.path.clone(),
            producer_id: config.producer_id,
            max_bytes: config.settings.journal_max_bytes,
            retention_seconds: config.settings.retention_seconds,
            ..Default::default()
        })
        .map_err(|_| ControllerError::InvalidInput {
            message: "observability storage unavailable".into(),
        })?;
        let mut attached = self
            .observation
            .attached
            .write()
            .map_err(|_| ControllerError::Internal)?;
        journal.try_record(EventInput::new(
            &config.rover_id,
            "phone",
            "lifecycle.start",
            serde_json::json!({"version":env!("CARGO_PKG_VERSION")}),
        ));
        let journal = Arc::new(journal);
        let evidence = terra_observability::EvidenceWriter::open(
            evidence_path,
            journal.clone(),
            config.settings.attachment_max_bytes,
        )
        .map_err(|_| ControllerError::InvalidInput {
            message: "evidence storage unavailable".into(),
        })?;
        *attached = Some(Attached {
            journal,
            evidence,
            rover: config.rover_id,
            path: config.path,
            uploader: None,
            settings: config.settings,
        });
        Ok(())
    }
    pub fn set_observability_rover(&self, rover_id: String) {
        if let Ok(mut a) = self.observation.attached.try_write()
            && let Some(a) = a.as_mut()
        {
            a.rover = rover_id;
        }
    }
    pub fn record_observation(&self, event_json: String) -> bool {
        if event_json.len() > 128 * 1024 {
            return false;
        }
        let Ok(event) = serde_json::from_str::<EventInput>(&event_json) else {
            return false;
        };
        self.observation
            .attached
            .try_read()
            .ok()
            .and_then(|a| {
                a.as_ref()
                    .map(|a| a.journal.try_record(event) == RecordOutcome::Queued)
            })
            .unwrap_or(false)
    }
    pub fn observability_health(&self) -> String {
        self.observation
            .attached
            .try_read()
            .ok()
            .and_then(|a| {
                a.as_ref()
                    .map(|a| serde_json::json!({"enabled":true,"journal":a.journal.health(),"upload":a.uploader.as_ref().map(|u|u.health())}).to_string())
            })
            .unwrap_or_else(|| "{\"enabled\":false}".into())
    }
    /// Administrative/testing barrier. Never call from a live control tick.
    pub fn flush_observability(&self, seconds: f64) -> Result<(), ControllerError> {
        if !seconds.is_finite() || seconds <= 0. || seconds > 30. {
            return Err(ControllerError::InvalidInput {
                message: "invalid flush timeout".into(),
            });
        }
        let a = self
            .observation
            .attached
            .read()
            .map_err(|_| ControllerError::Internal)?;
        if let Some(a) = a.as_ref() {
            a.evidence
                .flush(Duration::from_secs_f64(seconds))
                .map_err(|_| ControllerError::InvalidInput {
                    message: "evidence flush failed".into(),
                })?;
            a.journal
                .flush(Duration::from_secs_f64(seconds))
                .map_err(|_| ControllerError::InvalidInput {
                    message: "observability flush failed".into(),
                })?;
        }
        Ok(())
    }
}

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
    /// Owned exposure-aligned evidence; file I/O and hashing stay on the evidence worker.
    pub fn record_mapping_evidence(
        &self,
        manifest_json: String,
        attachments: Vec<ObservationAttachment>,
    ) -> bool {
        if manifest_json.len() > 65536 {
            return false;
        }
        let Ok(manifest) = serde_json::from_str::<serde_json::Value>(&manifest_json) else {
            return false;
        };
        let Ok(attached) = self.observation.attached.try_read() else {
            return false;
        };
        let Some(a) = attached.as_ref() else {
            return false;
        };
        let mut event = EventInput::new(
            manifest["rover_id"].as_str().unwrap_or(&a.rover),
            "mapping",
            "mapping.evidence",
            serde_json::json!({}),
        );
        event.session_id = manifest["session_id"].as_str().map(str::to_string);
        event.monotonic_ns = manifest["exposure_ns"].as_u64().unwrap_or(0);
        a.evidence.try_capture(terra_observability::EvidenceBundle {
            event,
            manifest,
            attachments: attachments
                .into_iter()
                .map(|a| terra_observability::AttachmentInput {
                    name: a.name,
                    data: a.data,
                    encoding: a.encoding,
                    dimensions: a.dimensions,
                    units: a.units,
                })
                .collect(),
        }) == terra_observability::CaptureOutcome::Queued
    }
}

#[uniffi::export]
impl MobileController {
    pub fn observability_settings(&self) -> String {
        let settings = self
            .observation
            .attached
            .try_read()
            .ok()
            .and_then(|a| a.as_ref().map(|a| a.settings))
            .unwrap_or_default();
        serde_json::to_string(&settings).unwrap_or_default()
    }
    pub fn record_observation_gap(&self, reason: String) {
        self.observation.event(
            "observability",
            "observability.gap",
            serde_json::json!({"reason":reason,"time_basis":"wall_only"}),
            &self.observation_run_id(),
            0.,
        );
    }
    /// The runtime session used by Swift diagnostics and camera evidence.
    pub fn observation_run_id(&self) -> String {
        self.brain
            .try_lock()
            .map(|b| b.run_id.clone())
            .unwrap_or_default()
    }
    /// Configure the independent uploader. Invoke off the control queue; credentials are memory-only.
    pub fn start_observability_upload(
        &self,
        endpoint: String,
        token: String,
    ) -> Result<(), ControllerError> {
        let mut attached = self
            .observation
            .attached
            .write()
            .map_err(|_| ControllerError::Internal)?;
        let Some(a) = attached.as_mut() else {
            return Err(ControllerError::InvalidInput {
                message: "observability not initialized".into(),
            });
        };
        let reader = terra_observability::JournalReader::open(&a.path).map_err(|_| {
            ControllerError::InvalidInput {
                message: "observability journal unavailable".into(),
            }
        })?;
        a.uploader = Some(
            terra_observability::UploadHandle::start(
                reader,
                terra_observability::UploadConfig {
                    endpoint,
                    token,
                    evidence_directory: Some(a.path.with_extension("evidence")),
                    clock_journal: Some(a.journal.clone()),
                },
            )
            .map_err(|_| ControllerError::InvalidInput {
                message: "invalid observability upload settings".into(),
            })?,
        );
        Ok(())
    }
}
