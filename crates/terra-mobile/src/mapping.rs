//! UniFFI map facade; depth acquisition and exposure-aligned poses belong to the caller.
use crate::ControllerError;
use std::sync::{Arc, Mutex};
use terra_mapping::{CameraIntrinsics, CameraPose, LocalOccupancyMap, MapConfig, MapError};
use terra_types::{Quaternion, Vector3};
#[derive(Clone, uniffi::Record)]
pub struct OccupancySettings {
    pub width: u32,
    pub height: u32,
    pub resolution: f64,
    pub min_obstacle_height: f64,
    pub max_obstacle_height: f64,
    pub max_range: f64,
    pub pixel_stride: u32,
}
#[uniffi::export]
pub fn default_occupancy_settings() -> OccupancySettings {
    let c = MapConfig::default();
    OccupancySettings {
        width: c.width,
        height: c.height,
        resolution: c.resolution,
        min_obstacle_height: c.min_obstacle_height,
        max_obstacle_height: c.max_obstacle_height,
        max_range: c.max_range,
        pixel_stride: c.pixel_stride,
    }
}
#[derive(Clone, uniffi::Record)]
pub struct MappingDepthFrame {
    pub timestamp: f64,
    pub width: u32,
    pub height: u32,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub camera_x: f64,
    pub camera_y: f64,
    pub camera_z: f64,
    pub quaternion_x: f64,
    pub quaternion_y: f64,
    pub quaternion_z: f64,
    pub quaternion_w: f64,
    pub depth_metres: Vec<f32>,
}
#[derive(Clone, uniffi::Record)]
pub struct OccupancyGrid {
    pub width: u32,
    pub height: u32,
    pub resolution: f64,
    pub origin_x: f64,
    pub origin_y: f64,
    pub occupancy: Vec<i8>,
}
impl From<MapError> for ControllerError {
    fn from(error: MapError) -> Self {
        Self::InvalidInput {
            message: error.to_string(),
        }
    }
}
#[derive(uniffi::Object)]
pub struct MobileOccupancyMap {
    map: Mutex<LocalOccupancyMap>,
}
#[uniffi::export]
impl MobileOccupancyMap {
    #[uniffi::constructor]
    pub fn new(settings: OccupancySettings) -> Result<Arc<Self>, ControllerError> {
        let map = LocalOccupancyMap::new(MapConfig {
            width: settings.width,
            height: settings.height,
            resolution: settings.resolution,
            min_obstacle_height: settings.min_obstacle_height,
            max_obstacle_height: settings.max_obstacle_height,
            max_range: settings.max_range,
            pixel_stride: settings.pixel_stride,
        })?;
        Ok(Arc::new(Self {
            map: Mutex::new(map),
        }))
    }
    pub fn recenter(&self, x: f64, y: f64) -> Result<(), ControllerError> {
        self.map
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .recenter(x, y)?;
        Ok(())
    }
    pub fn integrate_depth(&self, frame: MappingDepthFrame) -> Result<(), ControllerError> {
        self.map
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .integrate_depth(
                frame.timestamp,
                CameraIntrinsics {
                    width: frame.width,
                    height: frame.height,
                    fx: frame.fx,
                    fy: frame.fy,
                    cx: frame.cx,
                    cy: frame.cy,
                },
                CameraPose {
                    position: Vector3 {
                        x: frame.camera_x,
                        y: frame.camera_y,
                        z: frame.camera_z,
                    },
                    orientation: Quaternion {
                        x: frame.quaternion_x,
                        y: frame.quaternion_y,
                        z: frame.quaternion_z,
                        w: frame.quaternion_w,
                    },
                },
                &frame.depth_metres,
            )?;
        Ok(())
    }
    pub fn snapshot(&self) -> Result<OccupancyGrid, ControllerError> {
        let s = self
            .map
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .snapshot();
        Ok(OccupancyGrid {
            width: s.width,
            height: s.height,
            resolution: s.resolution,
            origin_x: s.origin_x,
            origin_y: s.origin_y,
            occupancy: s.occupancy,
        })
    }
    pub fn clear(&self) -> Result<(), ControllerError> {
        self.map
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .clear();
        Ok(())
    }
}
