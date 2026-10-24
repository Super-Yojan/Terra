//! UniFFI map facade; depth acquisition and exposure-aligned poses belong to the caller.
use crate::{ControllerError, MobileController};
use std::sync::{Arc, Mutex};
use terra_mapping::{CameraIntrinsics, CameraPose, GroundPlane, HorizonBand, MapConfig, MapError};
use terra_types::{Quaternion, Vector3};
#[derive(Clone, uniffi::Record)]
/// Rolling-map dimensions and depth-integration limits. Distances are metres.
pub struct OccupancySettings {
    /// Grid width, cells.
    pub width: u32,
    /// Grid height, cells.
    pub height: u32,
    /// Grid cell size, metres per cell.
    pub resolution: f64,
    /// Minimum height above supporting ground counted as an obstacle, metres.
    pub min_obstacle_height: f64,
    /// Maximum height above supporting ground counted as an obstacle, metres.
    pub max_obstacle_height: f64,
    /// Maximum depth integration range, metres.
    pub max_range: f64,
    /// Positive sampling stride through image pixels.
    pub pixel_stride: u32,
}
#[uniffi::export]
/// Return the portable mapper defaults as a phone-facing record.
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
/// Exposure-aligned optical depth image and camera-to-world pose.
/// Optical +X points right, +Y down, +Z forward; world +Z points up.
pub struct MappingDepthFrame {
    /// Depth exposure time, seconds on a nonnegative monotonic clock.
    pub timestamp: f64,
    /// Depth image width, pixels.
    pub width: u32,
    /// Depth image height, pixels.
    pub height: u32,
    /// Horizontal focal length, pixels.
    pub fx: f64,
    /// Vertical focal length, pixels.
    pub fy: f64,
    /// Horizontal principal point in integer-pixel coordinates.
    pub cx: f64,
    /// Vertical principal point in integer-pixel coordinates.
    pub cy: f64,
    /// Camera origin world +X coordinate, metres.
    pub camera_x: f64,
    /// Camera origin world +Y coordinate, metres.
    pub camera_y: f64,
    /// Camera origin world +Z coordinate, metres.
    pub camera_z: f64,
    /// Optical-camera-to-world quaternion X component.
    pub quaternion_x: f64,
    /// Optical-camera-to-world quaternion Y component.
    pub quaternion_y: f64,
    /// Optical-camera-to-world quaternion Z component.
    pub quaternion_z: f64,
    /// Optical-camera-to-world quaternion scalar component.
    pub quaternion_w: f64,
    /// Row-major axial optical depths in metres; length must equal width × height.
    pub depth_metres: Vec<f32>,
}
/// Exposure-aligned supporting ground in the depth frame's fixed world coordinates.
/// Signed height in metres is `(normal · point - offset) / |normal|`.
#[derive(Clone, uniffi::Record)]
pub struct MappingGroundPlane {
    /// Upward ground-normal x component.
    pub normal_x: f64,
    /// Upward ground-normal y component.
    pub normal_y: f64,
    /// Upward ground-normal z component.
    pub normal_z: f64,
    /// Plane equation offset, scaled consistently with the normal.
    pub offset: f64,
}
#[derive(Clone, uniffi::Record)]
/// Owned row-major occupancy snapshot in the controller world frame.
pub struct OccupancyGrid {
    /// Grid width, cells.
    pub width: u32,
    /// Grid height, cells.
    pub height: u32,
    /// Grid cell size, metres per cell.
    pub resolution: f64,
    /// World +X coordinate of the grid lower corner, metres.
    pub origin_x: f64,
    /// World +Y coordinate of the grid lower corner, metres.
    pub origin_y: f64,
    /// Row-major cells, rows increasing in world +Y: -1 unknown, 0..=100 occupancy.
    pub occupancy: Vec<i8>,
}
fn frame_intrinsics(frame: &MappingDepthFrame) -> CameraIntrinsics {
    CameraIntrinsics {
        width: frame.width,
        height: frame.height,
        fx: frame.fx,
        fy: frame.fy,
        cx: frame.cx,
        cy: frame.cy,
    }
}
fn frame_pose(frame: &MappingDepthFrame) -> CameraPose {
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
    }
}
impl From<MapError> for ControllerError {
    fn from(error: MapError) -> Self {
        Self::InvalidInput {
            message: error.to_string(),
        }
    }
}
#[derive(uniffi::Object)]
/// Mutex-protected rolling occupancy mapper exposed through UniFFI.
pub struct MobileOccupancyMap {
    map: Mutex<terra_runtime::mapping::MappingState>,
}
#[uniffi::export]
impl MobileOccupancyMap {
    #[uniffi::constructor]
    /// Construct an unknown occupancy grid centered on the world origin.
    ///
    /// # Errors
    /// Returns `InvalidInput` for invalid dimensions, resolution, obstacle heights,
    /// range, or stride (at most one million cells; resolution 0.01..=10 m).
    pub fn new(settings: OccupancySettings) -> Result<Arc<Self>, ControllerError> {
        let map = terra_runtime::mapping::MappingState::new(MapConfig {
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
    /// Center the rolling window near finite world coordinates `x`, `y` in metres.
    /// Shifts by whole cells, preserves overlap, and initializes new cells as unknown.
    ///
    /// # Errors
    /// Returns `InvalidInput` for invalid or excessive coordinates and `Internal`
    /// for a poisoned mutex.
    pub fn recenter(&self, x: f64, y: f64) -> Result<(), ControllerError> {
        self.map
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .recenter(x, y)?;
        Ok(())
    }
    /// Integrate one exposure-aligned optical depth frame into the rolling map.
    /// Use finite intrinsics, a valid camera pose, and strictly increasing nonnegative
    /// exposure seconds; invalid depth samples are handled by the mapper.
    ///
    /// # Errors
    /// Returns `InvalidInput` for invalid time, pose, dimensions, intrinsics, or
    /// pixel count; returns `Internal` for a poisoned mutex.
    pub fn integrate_depth(&self, frame: MappingDepthFrame) -> Result<(), ControllerError> {
        self.integrate_depth_with_ground_plane(
            frame,
            MappingGroundPlane {
                normal_x: 0.0,
                normal_y: 0.0,
                normal_z: 1.0,
                offset: 0.0,
            },
        )
    }
    /// Integrate depth with obstacle heights relative to the supplied local terrain.
    /// World x/y cells and camera pose remain unchanged, including on slopes.
    ///
    /// # Errors
    /// Returns the same errors as [`Self::integrate_depth`], and `InvalidInput`
    /// for a nonfinite/degenerate plane or a normal over 60 degrees from world up.
    pub fn integrate_depth_with_ground_plane(
        &self,
        frame: MappingDepthFrame,
        ground: MappingGroundPlane,
    ) -> Result<(), ControllerError> {
        self.map
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .integrate_depth_with_ground_plane(
                frame.timestamp,
                frame_intrinsics(&frame),
                frame_pose(&frame),
                &frame.depth_metres,
                GroundPlane {
                    normal: Vector3 {
                        x: ground.normal_x,
                        y: ground.normal_y,
                        z: ground.normal_z,
                    },
                    offset: ground.offset,
                },
            )?;
        Ok(())
    }
    /// Integrate depth without a ground estimate: only rays within
    /// `half_angle_degrees` of world horizontal count, every return among them
    /// within `max_range` metres is an obstacle, and all other rays are ignored.
    ///
    /// # Errors
    /// Returns the same errors as [`Self::integrate_depth`], and `InvalidInput`
    /// for a half-angle outside (0, 90) degrees or a nonpositive range.
    pub fn integrate_depth_in_horizon_band(
        &self,
        frame: MappingDepthFrame,
        half_angle_degrees: f64,
        max_range: f64,
    ) -> Result<(), ControllerError> {
        self.map
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .integrate_depth_in_horizon_band(
                frame.timestamp,
                frame_intrinsics(&frame),
                frame_pose(&frame),
                &frame.depth_metres,
                HorizonBand {
                    half_angle: half_angle_degrees.to_radians(),
                    max_range,
                },
            )?;
        Ok(())
    }
    /// Copy the current occupancy grid and world geometry.
    ///
    /// # Errors
    /// Returns `Internal` for a poisoned mutex.
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
    /// Clear all occupancy evidence and the previous integration timestamp.
    ///
    /// # Errors
    /// Returns `Internal` for a poisoned mutex.
    pub fn clear(&self) -> Result<(), ControllerError> {
        self.map
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .clear();
        Ok(())
    }
}

#[uniffi::export]
impl MobileController {
    pub fn attach_occupancy_map(
        &self,
        map: Arc<MobileOccupancyMap>,
    ) -> Result<(), ControllerError> {
        let handle = map
            .map
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .clone();
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .runtime
            .attach_mapping(handle);
        Ok(())
    }
}
#[derive(uniffi::Object)]
pub struct MobileGroundPolicy {
    state: Mutex<terra_runtime::ground::GroundPolicy>,
}
#[uniffi::export]
impl MobileGroundPolicy {
    #[uniffi::constructor]
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            state: Mutex::new(Default::default()),
        })
    }
    pub fn reset(&self) {
        self.state.lock().unwrap().reset();
    }
    pub fn offset(&self) -> Option<f64> {
        self.state.lock().unwrap().offset
    }
    pub fn clearance(&self) -> Option<f64> {
        self.state.lock().unwrap().clearance
    }
    pub fn observe(
        &self,
        height: Option<f64>,
        camera_height: f64,
        area: f64,
        now: f64,
        up_z: f64,
    ) -> Option<f64> {
        self.state
            .lock()
            .unwrap()
            .observe(height, camera_height, area, now, up_z)
    }
    pub fn plane(
        &self,
        x: f64,
        y: f64,
        z: f64,
        up_x: f64,
        up_y: f64,
        up_z: f64,
    ) -> Option<MappingGroundPlane> {
        self.state
            .lock()
            .unwrap()
            .plane(
                Vector3 { x, y, z },
                Vector3 {
                    x: up_x,
                    y: up_y,
                    z: up_z,
                },
            )
            .map(|p| MappingGroundPlane {
                normal_x: p.normal.x,
                normal_y: p.normal.y,
                normal_z: p.normal.z,
                offset: p.offset,
            })
    }
}
