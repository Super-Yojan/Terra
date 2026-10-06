use crate::*;
#[uniffi::export]
impl MobileController {
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
