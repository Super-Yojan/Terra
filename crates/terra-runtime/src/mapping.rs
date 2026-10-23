//! Shared map ownership for runtime and platform frame-ingestion adapters.
use std::sync::{Arc, Mutex};
use terra_mapping::*;
#[derive(Clone)]
pub struct MappingState(Arc<Mutex<LocalOccupancyMap>>, MapConfig);
impl MappingState {
    pub fn new(config: MapConfig) -> Result<Self, MapError> {
        Ok(Self(
            Arc::new(Mutex::new(LocalOccupancyMap::new(config)?)),
            config,
        ))
    }
    pub fn config(&self) -> MapConfig {
        self.1
    }
    pub fn clear(&self) {
        self.0.lock().unwrap().clear();
    }
    pub fn recenter(&self, x: f64, y: f64) -> Result<(), MapError> {
        self.0
            .lock()
            .map_err(|_| MapError::InvalidFrame)?
            .recenter(x, y)
    }
    pub fn snapshot(&self) -> MapSnapshot {
        self.0.lock().unwrap().snapshot()
    }
    pub fn integrate_depth(
        &self,
        t: f64,
        k: CameraIntrinsics,
        c: CameraPose,
        pixels: &[f32],
    ) -> Result<(), MapError> {
        self.integrate_depth_with_ground_plane(t, k, c, pixels, GroundPlane::default())
    }
    pub fn integrate_depth_with_ground_plane(
        &self,
        t: f64,
        k: CameraIntrinsics,
        c: CameraPose,
        pixels: &[f32],
        ground: GroundPlane,
    ) -> Result<(), MapError> {
        self.0
            .lock()
            .map_err(|_| MapError::InvalidFrame)?
            .integrate_depth_with_ground_plane(t, k, c, pixels, ground)
    }
}
