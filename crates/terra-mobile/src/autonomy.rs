use crate::*;

/// Levels the shared arbiter accepts today.
///
/// The match is exhaustive. Frontier exploration (`explore`, issues #32 and
/// #33) is intentionally absent: when that variant lands on `terra_autonomy::Level`,
/// this function stops compiling until the wire name is added here. Phone UIs
/// should render any name this list does not contain as unavailable.
#[uniffi::export]
pub fn supported_autonomy_levels() -> Vec<String> {
    [
        terra_autonomy::Level::Teleop,
        terra_autonomy::Level::AssistedTeleop,
        terra_autonomy::Level::Waypoint,
        terra_autonomy::Level::Supervised,
    ]
    .into_iter()
    .map(|level| match level {
        terra_autonomy::Level::Teleop => "teleop",
        terra_autonomy::Level::AssistedTeleop => "assisted_teleop",
        terra_autonomy::Level::Waypoint => "waypoint",
        terra_autonomy::Level::Supervised => "supervised",
    })
    .map(str::to_string)
    .collect()
}

#[uniffi::export]
impl MobileController {
    /// Bounded world-space visualization points; never a source of motor authority.
    pub fn publish_point_cloud(&self, payload: String) -> Result<(), ControllerError> {
        if payload.len() > 262144 {
            return Err(ControllerError::InvalidInput {
                message: "point cloud too large".into(),
            });
        }
        let value: serde_json::Value =
            serde_json::from_str(&payload).map_err(|_| ControllerError::Internal)?;
        if value["version"].as_u64() != Some(1)
            || value["points"].as_array().is_none_or(|p| p.len() > 4096)
        {
            return Err(ControllerError::InvalidInput {
                message: "invalid point cloud".into(),
            });
        }
        let brain = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        if let Some(dashboard) = brain.dashboard.as_ref() {
            dashboard.publish("pointcloud", payload);
        }
        Ok(())
    }
    /// Publish positioning quality without changing the controller's local motion frame.
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
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .dashboard = Some(service);
        Ok(())
    }
    /// Attach an outbound dashboard session to this controller; no hardware arming.
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
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .dashboard = Some(service);
        Ok(())
    }
    /// Drop network transport without clearing an accepted local mission.
    pub fn detach_dashboard_link(&self) -> Result<(), ControllerError> {
        let mut b = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        let service = b.dashboard.take();
        b.hardware_requests.clear(); b.hardware_tokens.clear();
        drop(b);
        // Network teardown must not stall the phone control/Bluetooth producer.
        if let Some(service) = service { std::thread::spawn(move || drop(service)); }
        Ok(())
    }
    pub fn reconnect_dashboard(&self, endpoint: String, prefix: String, rover_id: u64) -> Result<(), ControllerError> {
        self.detach_dashboard_link()?;
        let service = terra_transport::ControlPlane::connect(&endpoint, &prefix, rover_id)
            .map_err(|e| ControllerError::InvalidInput { message: e.to_string() })?;
        self.brain.lock().map_err(|_| ControllerError::Internal)?.dashboard = Some(service);
        Ok(())
    }
    /// Clear remote intent while preserving sensors, recording and a latched emergency stop.
    pub fn disconnect_dashboard(&self) -> Result<(), ControllerError> {
        let mut brain = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        let service = brain.dashboard.take();
        brain.hardware_requests.clear();
        brain.hardware_tokens.clear();
        brain.sequence += 1;
        let token = format!("dashboard-detach-{}", brain.sequence);
        brain.autonomy.set_level(terra_autonomy::LevelRequest {
            level: terra_autonomy::Level::Teleop,
            token,
        });
        brain.controller.reset();
        drop(brain);
        drop(service);
        Ok(())
    }
    pub fn take_hardware_request(&self) -> String {
        let Ok(mut b) = self.brain.lock() else { return String::new(); };
        let authority = serde_json::from_str(&b.autonomy_json).unwrap_or_default();
        while let Some((raw, received)) = b.hardware_requests.pop_front() {
            let age = received.elapsed().as_secs_f64();
            if let Some(mut request) = super::admit_hardware_request(raw.as_bytes(), age, &authority) {
                request["valid_for"] = serde_json::json!((0.5 - age).max(0.));
                return request.to_string();
            }
        }
        String::new()
    }
    pub fn publish_hardware_status(&self, payload: String) -> Result<(), ControllerError> {
        if payload.len() > 4096 || serde_json::from_str::<serde_json::Value>(&payload).is_err() {
            return Err(ControllerError::InvalidInput { message: "invalid hardware status".into() });
        }
        let b = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        if let Some(d) = b.dashboard.as_ref() { d.publish("hardware/status", payload); }
        Ok(())
    }
    /// Transport connectivity only, not acknowledgement by an ARGOS operator.
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
    pub fn autonomy_request(
        &self,
        kind: String,
        payload: String,
        timestamp: f64,
    ) -> Result<(), ControllerError> {
        let mut b = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        let invalid = || ControllerError::InvalidInput {
            message: "invalid autonomy request".into(),
        };
        match kind.as_str() {
            "autonomy" => b
                .autonomy
                .set_level(terra_autonomy::decode_level(payload.as_bytes()).ok_or_else(invalid)?),
            "safety" => {
                let healthy = b.estimator.estimate(timestamp).health == Health::Ready;
                b.autonomy.set_safety(
                    terra_autonomy::decode_safety(payload.as_bytes()).ok_or_else(invalid)?,
                    healthy,
                );
            }
            "goal" => {
                if !b.autonomy.accept_goal(
                    &terra_waypoint::decode_goal(payload.as_bytes()).ok_or_else(invalid)?,
                    20_000.,
                ) {
                    return Err(invalid());
                }
            }
            "goal/decision" => {
                let pose = b.pose.unwrap_or_default();
                let map = b.map.clone();
                let e = b.estimator.estimate(timestamp);
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
                        healthy: e.health == Health::Ready,
                    },
                );
            }
            "search" | "search/action" | "search/report/ack" => search::dispatch_search(&mut b,&kind,payload.as_bytes(),timestamp)?,
            _ => return Err(invalid()),
        }
        Ok(())
    }
    pub fn autonomy_origin(&self, latitude: f64, longitude: f64) -> Result<(), ControllerError> {
        terra_waypoint::GeoOrigin::new(latitude, longitude)?;
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .autonomy
            .set_origin(latitude, longitude);
        Ok(())
    }
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
    pub fn autonomy_status(&self) -> String {
        self.brain
            .lock()
            .map(|b| b.autonomy_json.clone())
            .unwrap_or_default()
    }
    pub fn begin_recording(&self, path: String, run_id: String) -> Result<(), ControllerError> {
        if !terra_autonomy::valid_token(&run_id) {
            return Err(ControllerError::InvalidInput {
                message: "invalid run identity".into(),
            });
        }
        let recorder=terra_experiment::RunRecorder::create(std::path::Path::new(&path),serde_json::json!({"run_id":run_id,"platform":"phone","planner":terra_navigation::PlannerConfig::default(),"collision_observation":"unavailable"}),8192).map_err(|e|ControllerError::InvalidInput{message:e.to_string()})?;
        let mut brain = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        if let Some(mut old) = brain.recorder.take() {
            old.close().map_err(|e| ControllerError::InvalidInput {
                message: e.to_string(),
            })?;
        }
        brain.recorder = Some(recorder);

        brain.autonomy = terra_autonomy::AutonomyArbiter::default();
        brain.autonomy.run_id = run_id.clone();
        brain.controller.reset();
        brain.run_id = run_id;
        brain.run_start = None;
        brain.sequence = 0;
        Ok(())
    }
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

#[cfg(test)]
mod dashboard_tests {
    use super::*;
    #[test]
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
