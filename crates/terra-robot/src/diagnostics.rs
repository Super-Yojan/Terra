//! Optional diagnostics: the on-phone journal, evidence and upload. Never
//! participates in control health or command admission.
use crate::Robot;
use serde::Deserialize;
use std::sync::{Arc, Mutex};
use std::{path::PathBuf, sync::RwLock, time::Duration};
use terra_observability::{EventInput, Journal, JournalConfig, RecordOutcome};
struct Attached {
    router_diagnostics: terra_observability::DiagnosticRecorder,
    journal: Arc<Journal>,
    evidence: terra_observability::EvidenceWriter,
    path: PathBuf,
    uploader: Option<terra_observability::UploadHandle>,
    settings: terra_observability::ObservationSettings,
}
#[derive(Default)]
pub struct Diagnostics {
    last_router: Mutex<Option<(String, std::time::Instant)>>,
    attached: RwLock<Option<Attached>>,
    rover: RwLock<Option<String>>,
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
impl Diagnostics {
    pub fn router_connect_failed(&self, error: &terra_transport::TransportError) {
        // Classify diagnostic text locally; never store text that might contain credentials.
        let category = match error {
            terra_transport::TransportError::InvalidSettings => "invalid_settings",
            terra_transport::TransportError::Network(message) => {
                let text = message.to_lowercase();
                if text.contains("certificate") {
                    "tls_certificate"
                } else if text.contains("auth") {
                    "authentication"
                } else if text.contains("timeout") || text.contains("timed out") {
                    "connect_timeout"
                } else if text.contains("refused") {
                    "connection_refused"
                } else if text.contains("dns") || text.contains("resolve") {
                    "name_resolution"
                } else {
                    "network"
                }
            }
            _ => "transport",
        };
        self.router_event(
            "router.connect.failed",
            serde_json::json!({"stage":"open_session","error_category":category}),
        );
    }
    pub fn router_event(&self, kind: &str, attributes: serde_json::Value) {
        if let Ok(attached) = self.attached.try_read()
            && let Some(a) = attached.as_ref()
        {
            a.router_diagnostics.try_record(kind, attributes);
        }
    }
    pub fn router_status(&self, service: Option<&terra_transport::ControlPlane>) {
        let details = service
            .map(|s| s.diagnostics())
            .unwrap_or_else(|| serde_json::json!({"state":"disconnected"}));
        // A busy telemetry snapshot is not evidence of stale data. Never hide a link failure.
        if details["telemetry_sample_available"] == false && details["state"] != "failed" {
            return;
        }
        let key = serde_json::json!([
            details["state"],
            details["failure_reason"],
            details["telemetry_fresh"]
        ])
        .to_string();
        if let Ok(mut last) = self.last_router.try_lock() {
            if last.as_ref().is_none_or(|(previous, at)| {
                previous != &key || at.elapsed() >= Duration::from_secs(5)
            }) {
                self.router_event("router.status", details);
                *last = Some((key, std::time::Instant::now()));
            }
        }
    }
    pub fn event(
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
            let Ok(rover) = self.rover.try_read() else {
                return;
            };
            let Some(rover) = rover.as_deref() else {
                return;
            };
            let mut event = EventInput::new(rover, component, kind, attributes);
            event.session_id = Some(run.into());
            event.monotonic_ns = (time.max(0.) * 1e9).min(u64::MAX as f64) as u64;
            a.journal.try_record(event);
        }
    }
    pub fn effort(&self, robot: &Robot, time: f64, output: &terra_types::MotorOutput) {
        let Ok(last) = self.last.try_lock() else {
            return;
        };
        if time != last.1 {
            return;
        }
        self.event("control", "control.output", serde_json::json!({"left_effort":output.left,"right_effort":output.right,"estimated_forward":output.estimated_forward,"estimated_yaw_rate":output.estimated_yaw_rate,"target_forward":output.target_forward,"target_yaw_rate":output.target_yaw_rate,"stop_reason":format!("{:?}",output.stop_reason)}), robot.run_id(), time);
    }
    pub fn tick(
        &self,
        robot: &Robot,
        time: f64,
        estimate: &terra_types::VelocityEstimate,
        status: &serde_json::Value,
    ) {
        let Ok(mut last) = self.last.try_lock() else {
            return;
        };
        let key = serde_json::json!([
            status["status"]["safety"],
            status["status"]["reason"],
            status["status"]["requested_level"],
            status["goal"]["state"]
        ])
        .to_string();
        let transition = key != last.0;
        if transition {
            self.event("autonomy","autonomy.transition",serde_json::json!({"previous":last.0,"reason":status["status"]["reason"],"status":status,"pose_age":robot.core().pose().map(|p|time-p.1),"map_age":robot.core().map().map(|m|time-m.1),"imu_age":robot.core().imu_time().map(|t|time-t),"map_timeout":robot.core().autonomy().map_freshness_timeout(),"pose_timeout":crate::pose_quality::FRESHNESS_SECONDS,"router_connected":robot.fleet_connected_state(),"recorder_failed":robot.recorder_failed()}),robot.run_id(),time);
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
            self.event("control","control.summary",serde_json::json!({"state":status,"estimated_forward":estimate.forward,"estimated_yaw_rate":estimate.yaw_rate,"sensor_health":format!("{:?}",estimate.health),"pose":robot.core().pose().map(|p|serde_json::json!({"x":p.0.x,"y":p.0.y,"yaw":p.0.yaw})),"map_revision":robot.core().map().map(|m|m.2)}),robot.run_id(),time);
            last.1 = time;
        }
    }
}
impl Diagnostics {
    /// Attach the journal, evidence store and router diagnostics described
    /// by `config_json` (`path`, `producer_id`, `rover_id`, `settings`).
    pub fn start(&self, config_json: &str) -> Result<(), String> {
        let config: Config = serde_json::from_str(config_json)
            .map_err(|_| "invalid observability configuration".to_string())?;
        if !config.settings.valid() {
            return Err("invalid observability settings".into());
        }
        {
            let mut rover = self.rover.write().map_err(|_| POISONED)?;
            if rover.is_none() {
                *rover = Some(config.rover_id.clone());
            }
        }
        let evidence_path = terra_observability::evidence_directory(&config.path);
        let router_diagnostics =
            terra_observability::DiagnosticRecorder::open(&config.path, "router")
                .map_err(|_| POISONED)?;
        let journal = Journal::open(JournalConfig {
            path: config.path.clone(),
            producer_id: config.producer_id,
            max_bytes: config.settings.journal_max_bytes,
            retention_seconds: config.settings.retention_seconds,
            ..Default::default()
        })
        .map_err(|_| "observability storage unavailable".to_string())?;
        let mut attached = self.attached.write().map_err(|_| POISONED)?;
        let selected = self.rover.read().map_err(|_| POISONED)?;
        journal.try_record(EventInput::new(
            selected.as_deref().unwrap_or(&config.rover_id),
            "phone",
            "lifecycle.start",
            serde_json::json!({"version":env!("CARGO_PKG_VERSION")}),
        ));
        drop(selected);
        let journal = Arc::new(journal);
        let evidence = terra_observability::EvidenceWriter::open(
            evidence_path,
            journal.clone(),
            config.settings.attachment_max_bytes,
        )
        .map_err(|_| "evidence storage unavailable".to_string())?;
        *attached = Some(Attached {
            router_diagnostics,
            journal,
            evidence,
            path: config.path,
            uploader: None,
            settings: config.settings,
        });
        Ok(())
    }
    /// Rover named in every event. Context has no disk or network work and
    /// survives attachment startup or replacement.
    pub fn set_rover(&self, rover_id: String) {
        if let Ok(mut rover) = self.rover.write() {
            *rover = Some(rover_id);
        }
    }
    /// Record one host event (JSON `EventInput`); `false` when not queued.
    pub fn record(&self, event_json: &str) -> bool {
        if event_json.len() > terra_observability::MAX_EVENT_BYTES {
            return false;
        }
        let Ok(event) = serde_json::from_str::<EventInput>(event_json) else {
            return false;
        };
        self.attached
            .try_read()
            .ok()
            .and_then(|a| {
                a.as_ref().map(|a| {
                    if matches!(event.component.as_str(), "router" | "bluetooth" | "phone") {
                        // Whitelist metadata; never duplicate payloads, credentials or location.
                        let attributes: serde_json::Map<String, serde_json::Value> = event
                            .attributes
                            .as_object()
                            .into_iter()
                            .flatten()
                            .filter(|(key, _)| {
                                matches!(
                                    key.as_str(),
                                    "reason"
                                        | "status"
                                        | "generation"
                                        | "mode"
                                        | "local_waypoint_retained"
                                        | "keep_bluetooth"
                                        | "error_domain"
                                        | "error_code"
                                )
                            })
                            .map(|(key, value)| (key.clone(), value.clone()))
                            .collect();
                        a.router_diagnostics.try_record(
                            &event.kind,
                            serde_json::json!({"component":event.component,"attributes":attributes}),
                        );
                    }
                    a.journal.try_record(event) == RecordOutcome::Queued
                })
            })
            .unwrap_or(false)
    }
    pub fn flush(&self, seconds: f64) -> Result<(), String> {
        if !seconds.is_finite() || seconds <= 0. || seconds > 30. {
            return Err("invalid flush timeout".into());
        }
        let a = self.attached.read().map_err(|_| POISONED)?;
        if let Some(a) = a.as_ref() {
            a.evidence
                .flush(Duration::from_secs_f64(seconds))
                .map_err(|_| "evidence flush failed".to_string())?;
            a.journal
                .flush(Duration::from_secs_f64(seconds))
                .map_err(|_| "observability flush failed".to_string())?;
        }
        Ok(())
    }
    /// Queue one camera evidence bundle; `false` when not queued.
    pub fn record_mapping_evidence(
        &self,
        manifest_json: &str,
        attachments: Vec<terra_observability::AttachmentInput>,
    ) -> bool {
        if manifest_json.len() > 65536 {
            return false;
        }
        let Ok(manifest) = serde_json::from_str::<serde_json::Value>(manifest_json) else {
            return false;
        };
        let Ok(attached) = self.attached.try_read() else {
            return false;
        };
        let Some(a) = attached.as_ref() else {
            return false;
        };
        let Ok(rover) = self.rover.try_read() else {
            return false;
        };
        let mut event = EventInput::new(
            manifest["rover_id"]
                .as_str()
                .or(rover.as_deref())
                .unwrap_or(terra_observability::PRODUCER_ROVER),
            "mapping",
            "mapping.evidence",
            serde_json::json!({}),
        );
        event.session_id = manifest["session_id"].as_str().map(str::to_string);
        event.monotonic_ns = manifest["exposure_ns"].as_u64().unwrap_or(0);
        a.evidence.try_capture(terra_observability::EvidenceBundle {
            event,
            manifest,
            attachments,
        }) == terra_observability::CaptureOutcome::Queued
    }
    /// Current settings as JSON (defaults while detached).
    pub fn settings_json(&self) -> String {
        let settings = self
            .attached
            .try_read()
            .ok()
            .and_then(|a| a.as_ref().map(|a| a.settings))
            .unwrap_or_default();
        serde_json::to_string(&settings).unwrap_or_default()
    }
    /// Configure the independent uploader. Invoke off the control queue;
    /// credentials are memory-only.
    pub fn start_upload(&self, endpoint: String, token: String) -> Result<(), String> {
        let mut attached = self.attached.write().map_err(|_| POISONED)?;
        let Some(a) = attached.as_mut() else {
            return Err("observability not initialized".into());
        };
        let reader = terra_observability::JournalReader::open(&a.path)
            .map_err(|_| "observability journal unavailable".to_string())?;
        a.uploader = Some(
            terra_observability::UploadHandle::start(
                reader,
                terra_observability::UploadConfig {
                    endpoint,
                    token,
                    evidence_directory: Some(terra_observability::evidence_directory(&a.path)),
                    clock_journal: Some(a.journal.clone()),
                },
            )
            .map_err(|_| "invalid observability upload settings".to_string())?,
        );
        Ok(())
    }
}

const POISONED: &str = "observability lock was poisoned";
