//! Phone-facing autonomy bindings and conversion to onboard arbiter inputs.

use crate::*;

impl MobileController {
    fn install_dashboard(
        &self,
        service: terra_transport::ControlPlane,
    ) -> Result<(), ControllerError> {
        let mut brain = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        brain.map_gate = Default::default();
        brain.cloud_gate = Default::default();
        brain.dashboard = Some(service);
        Ok(())
    }
}

/// Levels the shared arbiter accepts today.
///
/// Target search is advertised dynamically only when a fresh detector is available.
#[uniffi::export]
pub fn supported_autonomy_levels() -> Vec<String> {
    [
        terra_autonomy::Level::Teleop,
        terra_autonomy::Level::AssistedTeleop,
        terra_autonomy::Level::Waypoint,
        terra_autonomy::Level::WaypointDirect,
        terra_autonomy::Level::Explore,
        terra_autonomy::Level::Supervised,
    ]
    .into_iter()
    .map(|level| match level {
        terra_autonomy::Level::Teleop => "teleop",
        terra_autonomy::Level::AssistedTeleop => "assisted_teleop",
        terra_autonomy::Level::Waypoint => "waypoint",
        terra_autonomy::Level::Supervised => "supervised",
        terra_autonomy::Level::WaypointDirect => "waypoint_direct",
        terra_autonomy::Level::Explore => "explore",
        terra_autonomy::Level::TargetSearch => "target_search",
    })
    .map(str::to_string)
    .collect()
}

#[uniffi::export]
impl MobileController {
    /// Bounded world-space visualization points; never a source of motor authority.
    /// Requires version 1 JSON, at most 4096 points, and at most 262144 bytes.
    /// Full content is coalesced; unchanged newer samples can emit heartbeats.
    /// Returns `InvalidInput` for size/schema rejection and `Internal` for malformed
    /// JSON or lock failure. No dashboard means no network publication.
    pub fn publish_point_cloud(&self, payload: String) -> Result<(), ControllerError> {
        if payload.len() > 262144 {
            return Err(ControllerError::InvalidInput {
                message: "point cloud too large".into(),
            });
        }
        let mut value: serde_json::Value =
            serde_json::from_str(&payload).map_err(|_| ControllerError::Internal)?;
        if value["version"].as_u64() != Some(1)
            || value["points"].as_array().is_none_or(|p| p.len() > 4096)
        {
            return Err(ControllerError::InvalidInput {
                message: "invalid point cloud".into(),
            });
        }
        let mut brain = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        // Exact parsed content comparison; preserve original full-payload bytes.
        let frame_id = value["frameID"].as_str().map(str::to_owned);
        let publication = if brain.dashboard.is_some() {
            if let (Some(sequence), Some(_)) = (value["sequence"].as_u64(), frame_id.as_ref()) {
                value.as_object_mut().unwrap().remove("sequence");
                brain.cloud_gate.observe(
                    value,
                    frame_id.clone().unwrap(),
                    sequence,
                    std::time::Instant::now(),
                    std::time::Duration::from_millis(500),
                )
            } else {
                brain.cloud_gate = Default::default();
                publication::Publication::Full
            }
        } else {
            publication::Publication::Skip
        };
        let mut full_rejected = false;
        if let Some(dashboard) = brain.dashboard.as_ref() {
            match publication {
                publication::Publication::Full => {
                    full_rejected = !dashboard.publish_visualization("pointcloud", payload);
                }
                publication::Publication::Heartbeat(sequence) => dashboard.publish(
                    "pointcloud/heartbeat",
                    serde_json::json!({"version":1,"rover_id":dashboard.rover_id,
                        "frameID":frame_id,"sequence":sequence})
                    .to_string(),
                ),
                publication::Publication::Skip => {}
            }
        }
        brain.cloud_gate.full_enqueued(!full_rejected);
        Ok(())
    }
    /// Publish positioning quality without changing the controller's local motion frame.
    /// Requires version 1 JSON at most 4096 bytes. With no dashboard this is a no-op.
    /// Returns `InvalidInput` for size, JSON, or version rejection, or `Internal` on lock failure.
    pub fn publish_localization(&self, payload: String) -> Result<(), ControllerError> {
        if payload.len() > 4096 {
            return Err(ControllerError::InvalidInput {
                message: "localization payload too large".into(),
            });
        }
        let value: serde_json::Value =
            serde_json::from_str(&payload).map_err(|_| ControllerError::InvalidInput {
                message: "invalid localization JSON".into(),
            })?;
        if value["version"].as_u64() != Some(1) {
            return Err(ControllerError::InvalidInput {
                message: "unsupported localization version".into(),
            });
        }
        let brain = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        if let Some(dashboard) = brain.dashboard.as_ref() {
            dashboard.publish("localization", payload);
        }
        Ok(())
    }
    /// Host a loopback-only TCP dashboard endpoint for the selected rover.
    /// Run off the UI thread. Returns `InvalidInput` for endpoint/prefix or network
    /// failure, or `Internal` for a poisoned mutex.
    pub fn host_dashboard(
        &self,
        endpoint: String,
        prefix: String,
        rover_id: u64,
    ) -> Result<(), ControllerError> {
        let service =
            terra_transport::ControlPlane::listen(&endpoint, &prefix, rover_id).map_err(|e| {
                ControllerError::InvalidInput {
                    message: e.to_string(),
                }
            })?;
        self.install_dashboard(service)?;
        Ok(())
    }
    /// Attach an outbound dashboard session to this controller; no hardware arming.
    /// Clears previous remote intent before connecting; run off the UI thread.
    /// Returns `InvalidInput` for transport validation/connection failure or `Internal`
    /// for lock failure. Connection does not arm hardware.
    pub fn connect_dashboard(
        &self,
        endpoint: String,
        prefix: String,
        rover_id: u64,
    ) -> Result<(), ControllerError> {
        self.disconnect_dashboard()?;
        let service = terra_transport::ControlPlane::connect(&endpoint, &prefix, rover_id)
            .map_err(|e| ControllerError::InvalidInput {
                message: e.to_string(),
            })?;
        self.install_dashboard(service)?;
        Ok(())
    }
    /// Clear remote intent and open an outbound dashboard session with Zenoh credentials.
    /// An empty user omits credential configuration. Run off the UI thread.
    /// Returns `InvalidInput` for transport failure or `Internal` for lock failure.
    pub fn connect_dashboard_authenticated(
        &self,
        endpoint: String,
        prefix: String,
        rover_id: u64,
        user: String,
        password: String,
    ) -> Result<(), ControllerError> {
        self.disconnect_dashboard()?;
        let service = terra_transport::ControlPlane::connect_authenticated(
            &endpoint, &prefix, rover_id, &user, &password,
        )
        .map_err(|e| ControllerError::InvalidInput {
            message: e.to_string(),
        })?;
        self.install_dashboard(service)?;
        Ok(())
    }
    /// Drop network transport without clearing an accepted local mission.
    /// Clears queued hardware requests and tokens; closes the old link asynchronously.
    /// Returns `Internal` for a poisoned mutex.
    pub fn detach_dashboard_link(&self) -> Result<(), ControllerError> {
        let mut b = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        b.outdoor.pause("authority_lost");
        let service = b.dashboard.take();
        b.hardware_requests.clear();
        b.hardware_tokens.clear();
        drop(b);
        // Network teardown must not stall the phone control/Bluetooth producer.
        if let Some(service) = service {
            std::thread::spawn(move || drop(service));
        }
        Ok(())
    }
    /// Replace the dashboard transport while preserving accepted local mission state.
    /// Old hardware requests are discarded. Run off the UI thread. Returns `InvalidInput`
    /// for transport failure or `Internal` for lock failure.
    pub fn reconnect_dashboard(
        &self,
        endpoint: String,
        prefix: String,
        rover_id: u64,
    ) -> Result<(), ControllerError> {
        self.detach_dashboard_link()?;
        let service = terra_transport::ControlPlane::connect(&endpoint, &prefix, rover_id)
            .map_err(|e| ControllerError::InvalidInput {
                message: e.to_string(),
            })?;
        self.install_dashboard(service)?;
        Ok(())
    }
    /// Replace the link using Zenoh credentials while preserving accepted local mission state.
    /// An empty user omits credential configuration. Old hardware requests are discarded.
    /// Returns `InvalidInput` for transport failure or `Internal` for lock failure.
    pub fn reconnect_dashboard_authenticated(
        &self,
        endpoint: String,
        prefix: String,
        rover_id: u64,
        user: String,
        password: String,
    ) -> Result<(), ControllerError> {
        self.detach_dashboard_link()?;
        let service = terra_transport::ControlPlane::connect_authenticated(
            &endpoint, &prefix, rover_id, &user, &password,
        )
        .map_err(|e| ControllerError::InvalidInput {
            message: e.to_string(),
        })?;
        self.install_dashboard(service)?;
        Ok(())
    }
    /// Clear remote intent while preserving sensors, recording and a latched emergency stop.
    /// Switches authority back to teleop, resets the velocity loop, and closes the
    /// network link. May block during teardown. Returns `Internal` for lock failure.
    pub fn disconnect_dashboard(&self) -> Result<(), ControllerError> {
        let mut brain = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        brain.outdoor.pause("authority_lost");
        let service = brain.dashboard.take();
        brain.hardware_requests.clear();
        brain.hardware_tokens.clear();
        brain.sequence += 1;
        let token = format!("dashboard-detach-{}", brain.sequence);
        brain.autonomy.set_level(terra_autonomy::LevelRequest {
            level: terra_autonomy::Level::Teleop,
            token,
            budget_seconds: None,
            budget_minutes: None,
        });
        brain.controller.reset();
        drop(brain);
        drop(service);
        Ok(())
    }
    /// Consume the next currently admissible hardware request as JSON.
    /// Revalidates queued arm requests against run, authority revision, safety, and
    /// a 0.5 s receipt lease; adds remaining `valid_for` seconds. Returns an empty
    /// string when none survives validation or the controller lock is poisoned.
    pub fn take_hardware_request(&self) -> String {
        let Ok(mut b) = self.brain.lock() else {
            return String::new();
        };
        let authority = serde_json::from_str(&b.autonomy_json).unwrap_or_default();
        while let Some((raw, received)) = b.hardware_requests.pop_front() {
            let age = received.elapsed().as_secs_f64();
            if let Some(mut request) =
                super::admit_hardware_request(raw.as_bytes(), age, &authority)
            {
                request["valid_for"] = serde_json::json!((0.5 - age).max(0.));
                return request.to_string();
            }
        }
        String::new()
    }
    /// Store fresh hardware observation JSON and publish it to a connected dashboard.
    /// Used when validating configuration changes. Returns `InvalidInput` for
    /// malformed JSON or more than 4096 bytes, and `Internal` for lock failure.
    pub fn publish_hardware_status(&self, payload: String) -> Result<(), ControllerError> {
        if payload.len() > 4096 || serde_json::from_str::<serde_json::Value>(&payload).is_err() {
            return Err(ControllerError::InvalidInput {
                message: "invalid hardware status".into(),
            });
        }
        let mut b = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        b.hardware_observation = Some((
            serde_json::from_str(&payload).unwrap(),
            std::time::Instant::now(),
        ));
        if let Some(d) = b.dashboard.as_ref() {
            d.publish("hardware/status", payload);
        }
        Ok(())
    }
    /// Transport connectivity only, not acknowledgement by an ARGOS operator.
    /// Returns `connected`, `disconnected`, or `failed`; lock failure also yields `failed`.
    pub fn dashboard_status(&self) -> String {
        self.brain
            .lock()
            .map(|brain| {
                match brain.dashboard.as_ref() {
                    None => "disconnected",
                    Some(service) if service.failed() => "failed",
                    Some(_) => "connected",
                }
                .to_string()
            })
            .unwrap_or_else(|_| "failed".into())
    }
    /// Decode and dispatch local autonomy, safety, goal, decision, or search JSON intent.
    /// `timestamp` is current sensor-clock seconds. Successful dispatch does not
    /// guarantee authority or motion. Returns `InvalidInput` for unsupported kind,
    /// decoding failure, rejected goal/search request; `Internal` for lock failure.
    pub fn autonomy_request(
        &self,
        kind: String,
        payload: String,
        timestamp: f64,
    ) -> Result<(), ControllerError> {
        let outcome = (|| {
            let mut b = self.brain.lock().map_err(|_| ControllerError::Internal)?;
            let invalid = || ControllerError::InvalidInput {
                message: "invalid autonomy request".into(),
            };
            match kind.as_str() {
                "navigation/source" | "navigation/resume" => {
                    outdoor_bridge::request(&mut b, &kind, payload.as_bytes(), timestamp, 0.)?
                }
                "autonomy" => b.autonomy.set_level(
                    terra_autonomy::decode_level(payload.as_bytes()).ok_or_else(invalid)?,
                ),
                "safety" => {
                    let healthy = b.autonomy.is_manual()
                        || b.estimator.estimate(timestamp).health == Health::Ready;
                    b.autonomy.set_safety(
                        terra_autonomy::decode_safety(payload.as_bytes()).ok_or_else(invalid)?,
                        healthy,
                    );
                }
                "goal" => {
                    if !outdoor_bridge::dispatch_goal(&mut b, payload.as_bytes(), timestamp, 0.) {
                        return Err(invalid());
                    }
                }
                "goal/decision" => {
                    let pose = b.pose.unwrap_or_default();
                    let map = b.map.clone();
                    let e = b.estimator.estimate(timestamp);
                    let ready = pose_quality::ready(&b, timestamp);
                    b.autonomy.decide_proposal(
                        terra_autonomy::decode_decision(payload.as_bytes()).ok_or_else(invalid)?,
                        &terra_autonomy::ArbiterInput {
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
                            healthy: ready,
                        },
                    );
                }
                "search" | "search/action" | "search/report/ack" => {
                    search::dispatch_search(&mut b, &kind, payload.as_bytes(), timestamp)?
                }
                _ => return Err(invalid()),
            }
            Ok(())
        })();
        let run = self.observation_run_id();
        self.observation.event("operator", if outcome.is_ok() { "command.dispatched" } else { "command.rejected" }, serde_json::json!({"topic":kind,"payload":serde_json::from_str::<serde_json::Value>(&payload).ok(),"error":outcome.as_ref().err().map(|e: &ControllerError|e.to_string())}), &run, timestamp);
        outcome
    }
    /// Set the WGS84 origin using finite decimal-degree latitude and longitude.
    /// Returns `InvalidInput` for invalid coordinates or `Internal` for lock failure.
    pub fn autonomy_origin(&self, latitude: f64, longitude: f64) -> Result<(), ControllerError> {
        terra_waypoint::GeoOrigin::new(latitude, longitude)?;
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .autonomy
            .set_origin(latitude, longitude);
        Ok(())
    }
    /// Install a world-frame occupancy snapshot with source time and revision.
    /// `timestamp` is nonnegative finite sensor-clock seconds; `revision` identifies
    /// the source sample. Dimensions must be 1..=1000 with an exact cell count,
    /// resolution at least 0.01 m, and finite origin coordinates.
    /// Returns `InvalidInput` for failed validation or `Internal` for lock failure.
    pub fn autonomy_map(
        &self,
        grid: OccupancyGrid,
        timestamp: f64,
        revision: u64,
    ) -> Result<(), ControllerError> {
        if !timestamp.is_finite()
            || timestamp < 0.
            || grid.width == 0
            || grid.height == 0
            || grid.width > 1000
            || grid.height > 1000
            || grid.occupancy.len() != grid.width as usize * grid.height as usize
            || !grid.resolution.is_finite()
            || grid.resolution < 0.01
            || !grid.origin_x.is_finite()
            || !grid.origin_y.is_finite()
        {
            return Err(InputError::OutOfRange.into());
        }
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .map = Some((
            terra_mapping::MapSnapshot {
                width: grid.width,
                height: grid.height,
                resolution: grid.resolution,
                origin_x: grid.origin_x,
                origin_y: grid.origin_y,
                occupancy: grid.occupancy,
            },
            timestamp,
            revision,
        ));
        Ok(())
    }
    /// Copy the cached JSON status from the most recent control tick.
    /// Returns an empty string before the first tick or on lock failure.
    pub fn autonomy_status(&self) -> String {
        self.brain
            .lock()
            .map(|b| b.autonomy_json.clone())
            .unwrap_or_default()
    }
    /// Create a bounded run recorder at `path` and start a new run identity.
    /// Resets autonomy, velocity control, and run timing while retaining navigation
    /// geometry. `run_id` must be a valid token. Returns `InvalidInput` for invalid
    /// identity or recorder create/close failure, or `Internal` for lock failure.
    pub fn begin_recording(&self, path: String, run_id: String) -> Result<(), ControllerError> {
        if !terra_autonomy::valid_token(&run_id) {
            return Err(ControllerError::InvalidInput {
                message: "invalid run identity".into(),
            });
        }
        let recorder = terra_experiment::RunRecorder::create(
            std::path::Path::new(&path),
            serde_json::json!({
                "run_id": run_id,
                "platform": "phone",
                "planner": terra_navigation::PlannerConfig::default(),
                "collision_observation": "unavailable",
            }),
            8192,
        )
        .map_err(|e| ControllerError::InvalidInput {
            message: e.to_string(),
        })?;
        let mut brain = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        if let Some(mut old) = brain.recorder.take() {
            old.close().map_err(|e| ControllerError::InvalidInput {
                message: e.to_string(),
            })?;
        }
        brain.recorder = Some(recorder);

        brain.outdoor.cancel();
        brain.outdoor.selected = false;
        brain.outdoor.revision = brain.outdoor.revision.saturating_add(1);
        brain.autonomy = terra_autonomy::AutonomyArbiter::default();
        let planner = brain.navigation.parameters.planner();
        brain.autonomy.configure_planner(planner);
        let turning = brain.navigation.parameters.turning;
        brain.autonomy.configure_turning(turning)?;
        brain.autonomy.run_id = run_id.clone();
        brain.controller.reset();
        brain.run_id = run_id;
        brain.run_start = None;
        brain.sequence = 0;
        Ok(())
    }
    /// Take and close the current recorder; succeeds when none is active.
    /// Returns `InvalidInput` for recorder close failure or `Internal` for lock failure.
    pub fn end_recording(&self) -> Result<(), ControllerError> {
        if let Some(mut r) = self
            .brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .recorder
            .take()
        {
            r.close().map_err(|e| ControllerError::InvalidInput {
                message: e.to_string(),
            })?;
        }
        Ok(())
    }
}

/// Dashboard teardown scenarios, including sensor retention and latched emergency stop.
#[cfg(test)]
mod dashboard_tests {
    use super::*;
    #[test]
    /// Starts a waypoint with ready sensors, disconnects twice, and checks idle teleop authority
    /// while sensor readiness survives teardown.
    fn dashboard_disconnect_clears_goal_and_authority_without_resetting_sensors() {
        let controller = MobileController::new(default_control_settings()).unwrap();
        controller
            .push_imu(ImuReading {
                timestamp: 1.,
                acceleration_forward: 0.,
                acceleration_left: 0.,
                acceleration_up: 0.,
                gyro_roll: 0.,
                gyro_pitch: 0.,
                gyro_yaw: 0.,
            })
            .unwrap();
        controller
            .push_vio(VioReading {
                timestamp: 1.,
                position_x: 0.,
                position_y: 0.,
                position_z: 0.,
                quaternion_x: 0.,
                quaternion_y: 0.,
                quaternion_z: 0.,
                quaternion_w: 1.,
                velocity_x: 0.,
                velocity_y: 0.,
                velocity_z: 0.,
                tracked: true,
            })
            .unwrap();
        controller
            .autonomy_request(
                "autonomy".into(),
                r#"{"level":"waypoint","token":"level"}"#.into(),
                1.,
            )
            .unwrap();
        controller
            .autonomy_request(
                "goal".into(),
                r#"{"frame":"local","x":2,"y":0,"token":"goal"}"#.into(),
                1.,
            )
            .unwrap();
        controller.disconnect_dashboard().unwrap();
        controller.step(1.).unwrap();
        let status: serde_json::Value =
            serde_json::from_str(&controller.autonomy_status()).unwrap();
        assert_eq!(status["status"]["requested_level"], "teleop");
        assert_eq!(status["goal"]["state"], "idle");
        assert_eq!(controller.dashboard_status(), "disconnected");
        assert_eq!(
            controller
                .brain
                .lock()
                .unwrap()
                .estimator
                .estimate(1.)
                .health,
            Health::Ready
        );
        controller.disconnect_dashboard().unwrap();
    }
    #[test]
    /// Latches an emergency stop before dashboard disconnect and verifies the next tick remains stopped.
    fn dashboard_disconnect_preserves_emergency_stop() {
        let controller = MobileController::new(default_control_settings()).unwrap();
        controller
            .autonomy_request(
                "safety".into(),
                r#"{"action":"stop","token":"stop"}"#.into(),
                1.,
            )
            .unwrap();
        controller.disconnect_dashboard().unwrap();
        let output = controller.step(1.).unwrap();
        assert_eq!(output.left_effort, 0.);
        assert_eq!(output.right_effort, 0.);
        let status: serde_json::Value =
            serde_json::from_str(&controller.autonomy_status()).unwrap();
        assert_eq!(status["status"]["safety"], "emergency_stop");
    }
}
