//! The ARGOS fleet link: requests in, telemetry and video signaling out.
use crate::{Event, Robot, RobotError, capture, invalid_owned};
use serde_json::json;
use terra_types::telemetry;

impl Robot {
    /// An attached, working fleet link owns velocity intent; the phone's own
    /// stick and targets must not overwrite it.
    pub fn fleet_owns_intent(&self) -> bool {
        self.dashboard.as_ref().is_some_and(|d| !d.failed())
    }

    /// Fleet rover identity while a link is attached.
    pub fn rover_id(&self) -> Option<u64> {
        self.dashboard.as_ref().map(|d| d.rover_id())
    }

    pub(crate) fn fleet_connected_state(&self) -> Option<bool> {
        self.dashboard.as_ref().map(|d| !d.failed())
    }

    /// Start of each tick: a failed link is handled as lost, a running point
    /// capture advances and queued ARGOS requests reach the core.
    pub(crate) fn service_link(&mut self, now: f64) -> Result<(), RobotError> {
        if self.dashboard.as_ref().is_some_and(|d| d.failed()) {
            self.apply_link_loss(now);
        }
        let incoming = self
            .dashboard
            .as_ref()
            .filter(|d| !d.failed())
            .map(|d| d.take_actions())
            .unwrap_or_default();
        self.last_tick = now;
        capture::advance(self, now);
        for (kind, payload, age) in incoming {
            self.core.ingest(Event::Remote {
                kind,
                payload,
                received_at: now - age,
            })?;
        }
        let recording = self.recorder.as_ref().is_none_or(|r| !r.failed());
        self.core.set_recording_healthy(recording);
        Ok(())
    }

    fn apply_link_loss(&mut self, now: f64) {
        if let Some(decision) = self.core.lose_link() {
            self.diagnostics.event(
                "router",
                "connection.loss_decision",
                decision,
                self.core.run_id(),
                now,
            );
        }
    }

    /// Open a fleet link with `open` and attach it; a failure to open is journaled.
    pub fn attach_link(
        &mut self,
        open: impl FnOnce() -> Result<terra_transport::ControlPlane, terra_transport::TransportError>,
    ) -> Result<(), RobotError> {
        let service = open().map_err(|e| {
            self.diagnostics.router_connect_failed(&e);
            invalid_owned(e.to_string())
        })?;
        self.map_gate = Default::default();
        capture::reset(self);
        self.core.ingest(Event::Link {
            connected: true,
            received_at: self.last_tick,
        })?;
        // ARGOS rejects a pose/quality report without a numeric rover_id and
        // then falls back to its own yaw-rate guess (#111).
        self.core.set_rover_id(Some(service.rover_id()));
        self.dashboard = Some(service);
        Ok(())
    }

    /// Automatic transport failure, distinct from an operator disconnect.
    pub fn handle_link_loss(&mut self) {
        let now = self.core.imu_time().unwrap_or(0.);
        self.apply_link_loss(now);
        self.detach_link();
    }

    /// Drop network transport without clearing an accepted local mission.
    /// The old link closes on its own thread so teardown never stalls the tick.
    pub fn detach_link(&mut self) {
        self.core.detach_operator_link();
        if let Some(service) = self.dashboard.take() {
            std::thread::spawn(move || drop(service));
        }
    }

    /// The operator closed the link: back to teleop, velocity loop stopped.
    /// Sensors, recording and a latched emergency stop stay. Returns the link
    /// so the caller can drop it outside any lock.
    pub fn disconnect_link(&mut self) -> Option<terra_transport::ControlPlane> {
        let service = self.dashboard.take();
        self.core.disconnect_operator();
        service
    }

    /// `connected`, `disconnected` or `failed`; transport only, not an
    /// ARGOS operator's acknowledgement.
    pub fn link_status(&self) -> &'static str {
        self.diagnostics.router_status(self.dashboard.as_ref());
        match self.dashboard.as_ref() {
            None => "disconnected",
            Some(service) if service.failed() => "failed",
            Some(_) => "connected",
        }
    }

    /// Replace the connectivity policy (JSON, at most 4096 bytes).
    pub fn set_connectivity_policy(&mut self, settings_json: &str) -> Result<(), RobotError> {
        if settings_json.len() > 4096 {
            return Err(invalid_owned("connectivity policy too large".into()));
        }
        let policy: crate::connectivity::ConnectivityPolicySettings =
            serde_json::from_str(settings_json)
                .map_err(|_| invalid_owned("invalid connectivity policy".into()))?;
        policy.validate().map_err(|e| invalid_owned(e.into()))?;
        self.core.set_connectivity_policy(policy)?;
        self.diagnostics.event(
            "router",
            "connectivity.policy_changed",
            json!({"policy": self.core.connectivity(), "revision": self.core.connectivity_revision()}),
            self.core.run_id(),
            self.core.imu_time().unwrap_or(0.),
        );
        Ok(())
    }

    /// Publish positioning quality (version 1 JSON). No-op without a link.
    pub fn publish_localization(&self, payload: &str) -> Result<(), RobotError> {
        if payload.len() > telemetry::LOCALIZATION_INPUT_MAX_BYTES {
            return Err(invalid_owned("localization payload too large".into()));
        }
        let mut value: serde_json::Value = serde_json::from_str(payload)
            .map_err(|_| invalid_owned("invalid localization JSON".into()))?;
        if value["version"].as_u64() != Some(1) {
            return Err(invalid_owned("unsupported localization version".into()));
        }
        value["run_id"] = json!(self.core.run_id());
        value["connectivity_policy"] = json!(self.core.connectivity());
        if let Some(dashboard) = self.dashboard.as_ref() {
            dashboard.publish(telemetry::LOCALIZATION, value.to_string());
        }
        Ok(())
    }

    /// Store a hardware observation (JSON, at most 4096 bytes) and publish
    /// it to ARGOS.
    pub fn publish_hardware_status(&mut self, payload: String) -> Result<(), RobotError> {
        let status = (payload.len() <= 4096)
            .then(|| serde_json::from_str::<serde_json::Value>(&payload).ok())
            .flatten()
            .ok_or_else(|| invalid_owned("invalid hardware status".into()))?;
        let received_at = self
            .core
            .imu_time()
            .unwrap_or(self.last_tick)
            .max(self.last_tick);
        self.core.ingest(Event::Hardware {
            status: crate::HardwareStatus::from_json(&status),
            received_at,
        })?;
        if let Some(d) = self.dashboard.as_ref() {
            d.publish(telemetry::HARDWARE_STATUS, payload);
        }
        Ok(())
    }

    /// Dashboard-to-phone WebRTC signaling since the last call, as
    /// `(topic suffix, JSON)`. Video only; never a drive command.
    pub fn take_signaling(&self) -> Vec<(String, String)> {
        let Some(dashboard) = self.dashboard.as_ref() else {
            return Vec::new();
        };
        dashboard
            .take_signaling()
            .into_iter()
            .filter_map(|(kind, bytes)| Some((kind, String::from_utf8(bytes).ok()?)))
            .collect()
    }

    /// Publish one phone-to-dashboard signaling sample; `false` without a
    /// link or when the transport refuses it.
    pub fn publish_signaling(&self, kind: &str, payload: &str) -> Result<(), RobotError> {
        let Some(dashboard) = self.dashboard.as_ref() else {
            return Err(invalid_owned("dashboard is not connected".into()));
        };
        if !dashboard.publish_signaling(kind, payload) {
            return Err(invalid_owned("rejected webrtc signaling".into()));
        }
        Ok(())
    }
}
